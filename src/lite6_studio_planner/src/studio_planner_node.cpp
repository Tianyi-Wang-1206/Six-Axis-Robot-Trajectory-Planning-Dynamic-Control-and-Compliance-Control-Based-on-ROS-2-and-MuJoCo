#include "lite6_planner/kinematics_engine.hpp"
#include "lite6_studio_planner/program_compiler.hpp"
#include "lite6_studio_planner/studio_supervisor.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "lite6_interfaces/action/follow_impedance_trajectory.hpp"
#include "lite6_interfaces/msg/compile_diagnostic.hpp"
#include "lite6_interfaces/msg/controller_status.hpp"
#include "lite6_interfaces/msg/gripper_state.hpp"
#include "lite6_interfaces/msg/studio_state.hpp"
#include "lite6_interfaces/motion_contract.hpp"
#include "lite6_interfaces/shm_registers.hpp"
#include "lite6_interfaces/srv/compile_program.hpp"
#include "lite6_interfaces/srv/controller_command.hpp"
#include "lite6_interfaces/srv/gripper_command.hpp"
#include "lite6_interfaces/srv/studio_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <pthread.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace lite6_studio_planner
{
namespace
{
const std::vector<std::string> kJointNames{
  "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

constexpr double kAlignmentThresholdRad = 0.008;
// Block-time lookahead between a pause request and the stop-along-path splice switch:
// covers splice planning, the goal round trip and the CTC's first-sight decision.
constexpr double kPauseSpliceLookaheadSec = 0.12;

template <typename Fn>
struct ScopeExit
{
  Fn fn;
  explicit ScopeExit(Fn f) : fn(std::move(f)) {}
  ScopeExit(const ScopeExit &) = delete;
  ScopeExit & operator=(const ScopeExit &) = delete;
  ~ScopeExit() { fn(); }
};
template <typename Fn>
ScopeExit(Fn) -> ScopeExit<Fn>;
}  // namespace

class StudioPlannerNode : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using FollowImpedanceTrajectory = lite6_interfaces::action::FollowImpedanceTrajectory;
  using CompileProgram = lite6_interfaces::srv::CompileProgram;
  using ControllerCommand = lite6_interfaces::srv::ControllerCommand;
  using StudioCommand = lite6_interfaces::srv::StudioCommand;
  using GripperCommand = lite6_interfaces::srv::GripperCommand;
  using GripperState = lite6_interfaces::msg::GripperState;
  using ControllerStatus = lite6_interfaces::msg::ControllerStatus;
  using StudioState = lite6_interfaces::msg::StudioState;
  using Diagnostic = lite6_interfaces::msg::CompileDiagnostic;

  StudioPlannerNode()
  : Node("lite6_studio_planner_node")
  {
    const auto desc_pkg = ament_index_cpp::get_package_share_directory("lite6_description");
    const auto planner_pkg = ament_index_cpp::get_package_share_directory("lite6_planner");
    if (!kinematics_.load_ik_config(planner_pkg + "/config/ik_config.yaml") ||
        !kinematics_.init_robot_model(
          desc_pkg + "/urdf/lite6_arm.urdf", planner_pkg + "/config/lite6.srdf") ||
        !kinematics_.load_limits(planner_pkg + "/config/kinematic_limits.yaml"))
    {
      throw std::runtime_error("Robot planning model or YAML configuration failed to initialize");
    }
    compiler_ = std::make_unique<ProgramCompiler>(kinematics_);

    telemetry_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    shm_poll_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    compile_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    ctc_cmd_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    if (!state_shm_.open(false)) {
      throw std::runtime_error("Failed to open read-only state shared memory");
    }

    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    state_pub_ = create_publisher<StudioState>("/studio/state", state_qos);
    ctc_cmd_client_ = create_client<ControllerCommand>(
      "/ctc_controller/command", rclcpp::ServicesQoS(), ctc_cmd_group_);

    supervisor_ = std::make_unique<StudioSupervisor>(
      [this](const StudioSupervisor::Snapshot & snap) { publish_snapshot(snap); });

    rclcpp::SubscriptionOptions telemetry_opts;
    telemetry_opts.callback_group = telemetry_group_;

    joint_poll_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      [this]() { poll_joint_state_shm(); },
      shm_poll_group_);
    poll_joint_state_shm();

    controller_sub_ = create_subscription<ControllerStatus>(
      "/ctc_controller/status", rclcpp::QoS(10),
      [this](const ControllerStatus::SharedPtr msg) { on_controller_status(*msg); },
      telemetry_opts);

    gripper_sub_ = create_subscription<GripperState>(
      "/gripper/state", rclcpp::QoS(10),
      [this](const GripperState::SharedPtr msg) { on_gripper_state(*msg); },
      telemetry_opts);

    rigid_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/ctc_controller/follow_joint_trajectory", client_group_);
    imp_client_ = rclcpp_action::create_client<FollowImpedanceTrajectory>(
      this, "/ctc_controller/follow_impedance_trajectory", client_group_);
    gripper_client_ = create_client<GripperCommand>(
      "/gripper/command", rclcpp::ServicesQoS(), client_group_);

    compile_srv_ = create_service<CompileProgram>(
      "/studio/compile",
      [this](
        const std::shared_ptr<CompileProgram::Request> req,
        std::shared_ptr<CompileProgram::Response> resp) { handle_compile(req, resp); },
      rclcpp::ServicesQoS(), compile_group_);

    command_srv_ = create_service<StudioCommand>(
      "/studio/command",
      [this](
        const std::shared_ptr<StudioCommand::Request> req,
        std::shared_ptr<StudioCommand::Response> resp) { handle_command(req, resp); },
      rclcpp::ServicesQoS(), service_group_);

    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() { on_watchdog_timer(); },
      telemetry_group_);

    worker_thread_ = std::thread([this]() { worker_loop(); });
    publish_snapshot(supervisor_->snapshot());
    RCLCPP_INFO(get_logger(), "Robot Studio Planner node started");
  }

  ~StudioPlannerNode() override
  {
    {
      std::lock_guard<std::mutex> lock(worker_wake_mutex_);
      worker_exit_ = true;
    }
    worker_cv_.notify_all();
    cancel_active_controller_goals();
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

private:
  enum class ActionOutcome
  {
    SUCCEEDED,
    CANCELED,
    FAULT,
    SESSION_SUPERSEDED
  };

  // How a running controller goal reacts to a PAUSE request.
  enum class PauseHandling
  {
    CANCEL,      // cancel the goal (controller brakes; resume replans back onto the path)
    SPLICED,     // a stop-along-path tail was adopted; wait for the goal to settle
    LET_FINISH   // the goal ends sooner than a path stop; pause at the block boundary
  };

  void publish_snapshot(const StudioSupervisor::Snapshot & snap)
  {
    StudioState msg;
    msg.stamp = now();
    msg.revision = snap.revision;
    msg.state = snap.state;
    msg.loaded_program_id = snap.loaded_program_id;
    msg.current_line_number = snap.current_line_number;
    msg.current_statement_index = snap.current_statement_index;
    msg.current_block_index = snap.current_block_index;
    msg.total_blocks = snap.total_blocks;
    msg.block_progress_percent = snap.block_progress_percent;
    msg.total_progress_percent = snap.total_progress_percent;
    msg.controller_mode = snap.controller_mode;
    msg.controller_fresh = snap.controller_fresh;
    msg.joint_states_fresh = snap.joint_states_fresh;
    msg.gripper_state = snap.gripper_state;
    msg.gripper_ready = snap.gripper_ready;
    msg.estop_latched = snap.estop_latched;
    msg.estop_reason = snap.estop_reason;
    msg.last_error_code = snap.last_error_code;
    msg.cartesian_pos_error_m = snap.cartesian_pos_error_m;
    msg.cartesian_rot_error_rad = snap.cartesian_rot_error_rad;
    msg.detail = snap.detail;
    msg.controller_epoch = snap.controller_epoch;
    msg.program_revision = snap.program_revision;
    msg.last_compile_request_id = snap.last_compile_request_id;

    std::lock_guard<std::mutex> pub_lock(pub_mutex_);
    if (msg.revision < last_published_revision_) {
      return;
    }
    last_published_revision_ = msg.revision;
    if (state_pub_) {
      state_pub_->publish(msg);
    }
  }

  void send_controller_command_async(
    uint8_t command, uint64_t expected_epoch, const std::string & reason)
  {
    if (!ctc_cmd_client_ || !ctc_cmd_client_->service_is_ready()) {
      return;
    }
    auto req = std::make_shared<ControllerCommand::Request>();
    req->command = command;
    req->expected_epoch = expected_epoch;
    req->reason = reason;
    ctc_cmd_client_->async_send_request(req);
  }

  bool send_controller_command_sync(
    uint8_t command,
    uint64_t expected_epoch,
    const std::string & reason,
    uint8_t & out_ack,
    std::string & out_msg,
    uint8_t * out_mode = nullptr)
  {
    out_ack = ControllerCommand::Response::ACK_NONE;
    if (out_mode) {
      *out_mode = ControllerStatus::MODE_UNKNOWN;
    }
    if (!ctc_cmd_client_->wait_for_service(std::chrono::milliseconds(50))) {
      out_msg = "/ctc_controller/command service unavailable";
      return false;
    }
    auto req = std::make_shared<ControllerCommand::Request>();
    req->command = command;
    req->expected_epoch = expected_epoch;
    req->reason = reason;
    auto future = ctc_cmd_client_->async_send_request(req);
    if (future.wait_for(std::chrono::milliseconds(250)) != std::future_status::ready) {
      ctc_cmd_client_->remove_pending_request(future);
      // No response: outcome unknown (callers must decide from observed controller mode).
      out_ack = ControllerCommand::Response::ACK_RT_TIMEOUT;
      out_msg = "Timed out waiting for /ctc_controller/command response";
      return false;
    }
    const auto resp = future.get();
    out_ack = resp->ack_result;
    out_msg = resp->message;
    if (out_mode) {
      *out_mode = resp->current_mode;
    }
    return resp->accepted;
  }

  void apply_effects(const StudioSupervisor::Effects & effects)
  {
    if (effects.send_estop_cmd) {
      send_controller_command_async(
        ControllerCommand::Request::CMD_ESTOP, 0, "StudioSupervisor E-STOP");
    }
    if (effects.send_enter_normal_cmd) {
      send_controller_command_async(
        ControllerCommand::Request::CMD_ENTER_NORMAL, effects.controller_epoch,
        "StudioSupervisor enter NORMAL");
    }
    if (effects.cancel_controller_goals) {
      cancel_active_controller_goals();
    }
    if (effects.wake_worker) {
      {
        std::lock_guard<std::mutex> lock(worker_wake_mutex_);
        worker_wake_pending_ = true;
      }
      worker_cv_.notify_all();
    }
  }

  void cancel_active_controller_goals()
  {
    rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr rigid_gh;
    rclcpp_action::ClientGoalHandle<FollowImpedanceTrajectory>::SharedPtr imp_gh;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      rigid_gh = active_rigid_goal_;
      imp_gh = active_imp_goal_;
    }
    if (rigid_gh) {
      try {
        (void)rigid_client_->async_cancel_goal(rigid_gh);
      } catch (const std::exception &) {
      }
    }
    if (imp_gh) {
      try {
        (void)imp_client_->async_cancel_goal(imp_gh);
      } catch (const std::exception &) {
      }
    }
  }

  void poll_joint_state_shm()
  {
    if (!state_shm_.valid()) {
      return;
    }
    lite6_interfaces::shm::JointStatePod pod{};
    uint64_t seq = 0;
    if (!state_shm_->joint_state.read(pod, seq) || pod.valid == 0) {
      return;
    }
    const int64_t now_ns = lite6_interfaces::shm::steady_now_ns();
    if (pod.steady_stamp_ns <= 0 ||
        now_ns - pod.steady_stamp_ns > lite6_interfaces::contract::kJointStateStaleTimeoutNs)
    {
      return;
    }
    if (seq == last_joint_seq_ || pod.steady_stamp_ns <= last_joint_steady_ns_) {
      return;
    }
    last_joint_seq_ = seq;
    last_joint_steady_ns_ = pod.steady_stamp_ns;

    std::array<double, 6> q{};
    std::array<double, 6> dq{};
    bool pos_valid = true;
    bool dq_valid = true;
    double max_abs_dq = 0.0;

    for (size_t j = 0; j < 6; ++j) {
      q[j] = pod.position[j];
      dq[j] = pod.velocity[j];
      if (!std::isfinite(q[j])) {
        pos_valid = false;
      }
      if (!std::isfinite(dq[j])) {
        dq_valid = false;
      } else {
        max_abs_dq = std::max(max_abs_dq, std::abs(dq[j]));
      }
    }

    if (pos_valid && dq_valid) {
      std::lock_guard<std::mutex> lock(state_data_mutex_);
      latest_q_ = q;
      latest_dq_ = dq;
      has_latest_joint_ = true;
    }

    const auto effects = supervisor_->observe_joint_state(pos_valid, max_abs_dq, dq_valid);
    apply_effects(effects);
  }

  void on_controller_status(const ControllerStatus & msg)
  {
    if (msg.trajectory_start.sec != 0 || msg.trajectory_start.nanosec != 0) {
      std::lock_guard<std::mutex> lock(traj_status_mutex_);
      last_traj_id_ = msg.trajectory_id;
      last_traj_start_ = msg.trajectory_start;
    }
    const auto effects = supervisor_->observe_controller(msg);
    apply_effects(effects);
  }

  void on_gripper_state(const GripperState & msg)
  {
    {
      std::lock_guard<std::mutex> lock(state_data_mutex_);
      latest_gripper_ = msg;
      has_latest_gripper_ = true;
    }
    supervisor_->observe_gripper(msg.state, msg.ready);
  }

  void on_watchdog_timer()
  {
    const auto effects = supervisor_->watchdog();
    apply_effects(effects);
    publish_snapshot(supervisor_->snapshot());
  }

  bool get_latest_joint_state(std::array<double, 6> & out_q, double & out_max_dq) const
  {
    if (!supervisor_->is_fresh_joint()) {
      return false;
    }
    if (state_shm_.valid()) {
      lite6_interfaces::shm::JointStatePod pod{};
      uint64_t seq = 0;
      const int64_t now_ns = lite6_interfaces::shm::steady_now_ns();
      if (state_shm_->joint_state.read(pod, seq) && pod.valid != 0 &&
          pod.steady_stamp_ns > 0 &&
          now_ns - pod.steady_stamp_ns <= lite6_interfaces::contract::kJointStateStaleTimeoutNs)
      {
        bool finite_ok = true;
        double max_dq = 0.0;
        for (size_t i = 0; i < 6; ++i) {
          if (!std::isfinite(pod.position[i]) || !std::isfinite(pod.velocity[i])) {
            finite_ok = false;
            break;
          }
          out_q[i] = pod.position[i];
          max_dq = std::max(max_dq, std::abs(pod.velocity[i]));
        }
        if (finite_ok) {
          out_max_dq = max_dq;
          return true;
        }
      }
    }
    std::lock_guard<std::mutex> lock(state_data_mutex_);
    if (!has_latest_joint_) {
      return false;
    }
    out_q = latest_q_;
    out_max_dq = 0.0;
    for (double v : latest_dq_) {
      out_max_dq = std::max(out_max_dq, std::abs(v));
    }
    return true;
  }

  void handle_compile(
    const std::shared_ptr<CompileProgram::Request> req,
    std::shared_ptr<CompileProgram::Response> resp)
  {
    resp->request_id = req->request_id;
    resp->program_revision = 0;

    std::lock_guard<std::mutex> compile_lock(compile_mutex_);
    std::string busy_reason;
    // Atomic with RUN/RESUME/STEP/CLEAR_PROGRAM admission inside the supervisor.
    if (!supervisor_->begin_compile(req->auto_load, busy_reason)) {
      Diagnostic d;
      d.severity = Diagnostic::SEVERITY_ERROR;
      d.stage = Diagnostic::STAGE_SEMANTIC;
      d.line_number = 1;
      d.column_number = 1;
      d.statement_index = -1;
      d.code = "ERR_COMPILE_BUSY";
      d.message = busy_reason;
      resp->success = false;
      resp->diagnostics.push_back(d);
      return;
    }
    ScopeExit end_compiling{[this]() { supervisor_->end_compile(); }};

    std::array<double, 6> start_joints{};
    if (req->use_current_robot_start) {
      double dummy_dq = 0.0;
      if (!get_latest_joint_state(start_joints, dummy_dq)) {
        Diagnostic d;
        d.severity = Diagnostic::SEVERITY_ERROR;
        d.stage = Diagnostic::STAGE_SEMANTIC;
        d.line_number = 1;
        d.column_number = 1;
        d.statement_index = -1;
        d.code = "ERR_NO_JOINT_STATE";
        d.message = "Cannot resolve current robot start pose: /joint_states unavailable or stale";
        if (req->auto_load) {
          supervisor_->on_compile_failed(req->request_id);
        }
        resp->success = false;
        resp->diagnostics.push_back(d);
        return;
      }
    } else {
      start_joints = req->custom_start_joints;
    }

    auto compiled = std::make_shared<CompiledProgram>();
    std::vector<Diagnostic> diags;
    const bool ok = compiler_->compile(req->source_code, start_joints, *compiled, diags);
    if (!ok) {
      if (req->auto_load) {
        {
          std::lock_guard<std::mutex> exec_lock(exec_mutex_);
          loaded_program_.reset();
          reset_exec_cursor_unlocked();
        }
        supervisor_->on_compile_failed(req->request_id);
      }
      resp->success = false;
      resp->diagnostics = std::move(diags);
      return;
    }

    if (req->auto_load) {
      uint64_t new_rev = 0;
      {
        std::lock_guard<std::mutex> exec_lock(exec_mutex_);
        loaded_program_ = compiled;
        reset_exec_cursor_unlocked();
        new_rev = ++program_revision_;
      }
      supervisor_->on_compile_succeeded(
        compiled->program_id, static_cast<int32_t>(compiled->total_blocks),
        new_rev, req->request_id);
      resp->program_revision = new_rev;
    } else {
      supervisor_->on_compile_dry_run(req->request_id);
      std::lock_guard<std::mutex> exec_lock(exec_mutex_);
      resp->program_revision = program_revision_;
    }

    resp->success = true;
    resp->program_id = compiled->program_id;
    resp->total_statements = compiled->total_statements;
    resp->total_blocks = compiled->total_blocks;
    resp->estimated_duration_sec = compiled->estimated_duration_sec;
    resp->total_samples_1khz = compiled->total_samples_1khz;
    resp->diagnostics = std::move(diags);
  }

  void handle_command(
    const std::shared_ptr<StudioCommand::Request> req,
    std::shared_ptr<StudioCommand::Response> resp)
  {
    std::lock_guard<std::mutex> cmd_lock(command_mutex_);
    const uint8_t cmd = req->command;

    const auto result = supervisor_->handle_command(cmd, req->expected_revision, req->reason);
    // STEP from COMPILED_READY always starts at block 0: a cursor left behind by a
    // step-to-end, FAULT, or controller-originated E-STOP must not be reused.
    const bool step_from_ready =
      cmd == StudioCommand::Request::CMD_STEP &&
      result.prev_state == StudioState::STATE_COMPILED_READY;
    if (result.accepted &&
        (step_from_ready ||
         cmd == StudioCommand::Request::CMD_RUN ||
         cmd == StudioCommand::Request::CMD_ESTOP ||
         cmd == StudioCommand::Request::CMD_CLEAR_PROGRAM ||
         (cmd == StudioCommand::Request::CMD_STOP &&
          supervisor_->current_state() != StudioState::STATE_STOPPING)))
    {
      std::lock_guard<std::mutex> exec_lock(exec_mutex_);
      reset_exec_cursor_unlocked();
      if (cmd == StudioCommand::Request::CMD_CLEAR_PROGRAM) {
        loaded_program_.reset();
      }
    }
    apply_effects(result.effects);

    resp->accepted = result.accepted;
    resp->revision = result.revision;
    resp->state = result.state;
    resp->controller_epoch = result.controller_epoch;
    resp->message = result.message;

    if (result.accepted && cmd == StudioCommand::Request::CMD_PAUSE &&
        gripper_block_active_.load(std::memory_order_acquire))
    {
      const std::string note =
        "Pause requested; it takes effect after the active gripper command completes";
      supervisor_->note_pause_deferred(note);
      resp->message = note;
    }
  }

  void reset_exec_cursor_unlocked()
  {
    exec_block_idx_ = 0;
    paused_mid_block_ = false;
    paused_sample_idx_ = 0;
    paused_remaining_wait_sec_ = -1.0;
    paused_block_snapshot_ = ExecutionBlock();
    post_impedance_offset_active_ = false;
  }

  template <typename ActionT>
  ActionOutcome run_controller_action(
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    const typename ActionT::Goal & goal,
    uint64_t session,
    double timeout_sec,
    std::function<void(const typename ActionT::Feedback &)> on_feedback,
    bool is_rigid_action,
    bool honor_pause_stop,
    std::string & out_error,
    std::function<PauseHandling(const rclcpp_action::GoalUUID &)> on_pause = nullptr)
  {
    const auto start_tp = std::chrono::steady_clock::now();
    while (!client->action_server_is_ready()) {
      if (!rclcpp::ok() || session != supervisor_->active_session()) {
        return ActionOutcome::SESSION_SUPERSEDED;
      }
      if (honor_pause_stop &&
          (supervisor_->stop_requested(session) || supervisor_->pause_requested(session)))
      {
        return ActionOutcome::CANCELED;
      }
      if (std::chrono::steady_clock::now() - start_tp > std::chrono::seconds(3)) {
        out_error = "Controller action server is not ready";
        return ActionOutcome::FAULT;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    typename rclcpp_action::Client<ActionT>::SendGoalOptions opts;
    auto abandoned = std::make_shared<std::atomic<bool>>(false);
    bool waiting_on_goal_response = true;
    ScopeExit abandon_pending{[&waiting_on_goal_response, abandoned]() {
      if (waiting_on_goal_response) {
        abandoned->store(true, std::memory_order_release);
      }
    }};
    opts.goal_response_callback =
      [client, this, session, abandoned](
        const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr & accepted_gh) {
        if (!accepted_gh) {
          return;
        }
        if (abandoned->load(std::memory_order_acquire) ||
            session != supervisor_->active_session()) {
          try {
            (void)client->async_cancel_goal(accepted_gh);
          } catch (const std::exception &) {
          }
        }
      };
    if (on_feedback) {
      opts.feedback_callback =
        [on_feedback](
          typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
          const std::shared_ptr<const typename ActionT::Feedback> feedback) {
          if (feedback) {
            on_feedback(*feedback);
          }
        };
    }

    auto gh_future = client->async_send_goal(goal, opts);
    while (gh_future.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) {
      if (!rclcpp::ok() || session != supervisor_->active_session()) {
        return ActionOutcome::SESSION_SUPERSEDED;
      }
      if (std::chrono::steady_clock::now() - start_tp > std::chrono::seconds(5)) {
        out_error = "Timed out waiting for controller goal response";
        return ActionOutcome::FAULT;
      }
    }

    auto goal_handle = gh_future.get();
    waiting_on_goal_response = false;
    if (!goal_handle) {
      out_error = "Controller rejected trajectory goal";
      return ActionOutcome::FAULT;
    }

    auto set_active = [&](const auto & gh) {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if constexpr (std::is_same_v<ActionT, FollowJointTrajectory>) {
        active_rigid_goal_ = gh;
      } else {
        active_imp_goal_ = gh;
      }
    };
    auto clear_active = [&]() {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if constexpr (std::is_same_v<ActionT, FollowJointTrajectory>) {
        active_rigid_goal_.reset();
      } else {
        active_imp_goal_.reset();
      }
    };
    (void)is_rigid_action;
    set_active(goal_handle);
    ScopeExit clear_guard{clear_active};

    bool cancel_sent = false;
    // Set once a pause was turned into a path-preserving splice or a natural finish;
    // STOP, E-STOP and session changes still cancel.
    bool pause_handled = false;
    auto maybe_cancel = [&]() {
      if (cancel_sent) {
        return;
      }
      const bool superseded = !rclcpp::ok() || session != supervisor_->active_session();
      const bool stop = honor_pause_stop && supervisor_->stop_requested(session);
      bool pause = honor_pause_stop && supervisor_->pause_requested(session);
      if (pause && !superseded && !stop && on_pause && !pause_handled) {
        pause_handled = on_pause(goal_handle->get_goal_id()) != PauseHandling::CANCEL;
      }
      if (pause_handled) {
        pause = false;
      }
      if (superseded || stop || pause) {
        cancel_sent = true;
        try {
          (void)client->async_cancel_goal(goal_handle);
        } catch (const std::exception &) {
        }
      }
    };

    maybe_cancel();
    auto res_future = client->async_get_result(goal_handle);
    const auto exec_deadline =
      std::chrono::steady_clock::now() +
      std::chrono::milliseconds(static_cast<int64_t>(std::max(5.0, timeout_sec) * 1000.0));

    while (res_future.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) {
      if (!rclcpp::ok()) {
        return ActionOutcome::SESSION_SUPERSEDED;
      }
      maybe_cancel();
      if (session != supervisor_->active_session()) {
        return ActionOutcome::SESSION_SUPERSEDED;
      }
      if (std::chrono::steady_clock::now() > exec_deadline) {
        try {
          (void)client->async_cancel_goal(goal_handle);
        } catch (const std::exception &) {
        }
        out_error = "Trajectory execution exceeded timeout deadline";
        return ActionOutcome::FAULT;
      }
    }

    const auto wrapped = res_future.get();
    if (session != supervisor_->active_session()) {
      return ActionOutcome::SESSION_SUPERSEDED;
    }
    if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
      return ActionOutcome::SUCCEEDED;
    }
    if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
      return ActionOutcome::CANCELED;
    }
    if (wrapped.result && !wrapped.result->error_string.empty()) {
      out_error = wrapped.result->error_string;
    } else {
      out_error = "Controller trajectory execution aborted";
    }
    return ActionOutcome::FAULT;
  }

  void worker_loop()
  {
    pthread_setname_np(pthread_self(), "studio_worker");
    while (true) {
      {
        std::unique_lock<std::mutex> lock(worker_wake_mutex_);
        worker_cv_.wait(lock, [this]() {
          return worker_exit_ || worker_wake_pending_;
        });
        worker_wake_pending_ = false;
        if (worker_exit_) {
          return;
        }
      }

      const auto [st, session] = supervisor_->state_and_session();
      if (st == StudioState::STATE_RECOVERING) {
        execute_recovery(session);
      } else if (st == StudioState::STATE_RUNNING || st == StudioState::STATE_STEPPING) {
        execute_program(session);
      }
    }
  }

  void execute_recovery(uint64_t session)
  {
    bool sent_clear_cmd = false;
    auto last_clear_cmd_tp = std::chrono::steady_clock::time_point{};

    while (rclcpp::ok() && !worker_exit_) {
      const auto [st, active_sess] = supervisor_->state_and_session();
      if (active_sess != session || st != StudioState::STATE_RECOVERING) {
        return;
      }
      const auto sub = supervisor_->recovery_subphase();
      if (sub == StudioSupervisor::RecoverySubPhase::WAIT_SETTLE ||
          sub == StudioSupervisor::RecoverySubPhase::POST_SETTLE)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }

      if (sub == StudioSupervisor::RecoverySubPhase::ENABLE_NORMAL) {
        const auto now_tp = std::chrono::steady_clock::now();
        if (!sent_clear_cmd || (now_tp - last_clear_cmd_tp) > std::chrono::milliseconds(300)) {
          uint8_t ack = ControllerCommand::Response::ACK_NONE;
          uint8_t ack_mode = ControllerStatus::MODE_UNKNOWN;
          std::string err;
          const uint64_t epoch = supervisor_->snapshot().controller_epoch;
          const bool ok = send_controller_command_sync(
            ControllerCommand::Request::CMD_CLEAR_ESTOP,
            epoch,
            "Studio E-STOP recovery via CMD_CLEAR_ESTOP",
            ack,
            err,
            &ack_mode);
          sent_clear_cmd = true;
          last_clear_cmd_tp = now_tp;
          // Transient rejections are retried: SPEED (still moving), BUSY (trajectory
          // or stopping ramp finishing), RT_TIMEOUT (withdrawn or no response).
          // STATE while the controller already reports NORMAL means an earlier
          // CLEAR_ESTOP took effect; the NORMAL observation advances the phase.
          const bool transient =
            ack == ControllerCommand::Response::ACK_REJECTED_SPEED ||
            ack == ControllerCommand::Response::ACK_REJECTED_BUSY ||
            ack == ControllerCommand::Response::ACK_RT_TIMEOUT ||
            (ack == ControllerCommand::Response::ACK_REJECTED_STATE &&
             ack_mode == ControllerStatus::MODE_NORMAL);
          if (!ok && !transient) {
            const auto [cur_st, cur_sess] = supervisor_->state_and_session();
            if (cur_sess == session && cur_st == StudioState::STATE_RECOVERING &&
                supervisor_->recovery_subphase() == StudioSupervisor::RecoverySubPhase::ENABLE_NORMAL)
            {
              supervisor_->on_recovery_failed(
                session, err.empty() ? "Controller rejected CMD_CLEAR_ESTOP" : err);
              return;
            }
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
      }
    }
  }

  bool ensure_controller_normal(uint64_t session, std::string & out_err)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(2500);
    auto last_send = std::chrono::steady_clock::time_point{};
    while (rclcpp::ok() && !worker_exit_) {
      if (session != supervisor_->active_session()) {
        return false;
      }
      const auto snap = supervisor_->snapshot();
      if (snap.controller_mode == ControllerStatus::MODE_NORMAL) {
        return true;
      }
      if (snap.controller_mode == ControllerStatus::MODE_ESTOP) {
        return false;
      }
      if (snap.controller_mode == ControllerStatus::MODE_CARTESIAN_IMPEDANCE) {
        const auto now_tp = std::chrono::steady_clock::now();
        if (last_send == std::chrono::steady_clock::time_point{} ||
            (now_tp - last_send) > std::chrono::milliseconds(300))
        {
          uint8_t ack = ControllerCommand::Response::ACK_NONE;
          uint8_t ack_mode = ControllerStatus::MODE_UNKNOWN;
          std::string msg;
          const bool ok = send_controller_command_sync(
            ControllerCommand::Request::CMD_ENTER_NORMAL,
            snap.controller_epoch,
            "Studio exiting held Cartesian impedance to MODE_NORMAL",
            ack, msg, &ack_mode);
          last_send = now_tp;
          const bool transient =
            ok ||
            ack == ControllerCommand::Response::ACK_REJECTED_BUSY ||
            ack == ControllerCommand::Response::ACK_RT_TIMEOUT ||
            (ack == ControllerCommand::Response::ACK_REJECTED_STATE &&
             (ack_mode == ControllerStatus::MODE_NORMAL ||
              ack_mode == ControllerStatus::MODE_COMPLIANCE_STOPPING));
          if (!transient) {
            out_err = msg.empty() ?
              "Controller rejected CMD_ENTER_NORMAL while exiting impedance hold" : msg;
            return false;
          }
        }
      } else if (snap.controller_mode != ControllerStatus::MODE_COMPLIANCE_STOPPING) {
        out_err = "Controller in unexpected mode while returning to MODE_NORMAL";
        return false;
      }

      if (std::chrono::steady_clock::now() > deadline) {
        out_err = "Timed out waiting for controller to return to MODE_NORMAL after impedance hold";
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  bool ensure_normal_or_fault(uint64_t session, int32_t err_code)
  {
    std::string norm_err;
    if (ensure_controller_normal(session, norm_err)) {
      return true;
    }
    if (!norm_err.empty() && session == supervisor_->active_session() &&
        supervisor_->current_state() != StudioState::STATE_ESTOP)
    {
      StudioSupervisor::Effects eff;
      supervisor_->on_execution_fault(session, err_code, norm_err, false, eff);
      apply_effects(eff);
    }
    return false;
  }

  void execute_program(uint64_t session)
  {
    std::shared_ptr<const CompiledProgram> prog;
    size_t block_idx = 0;
    bool resume_mid = false;
    size_t pause_sample = 0;
    double resume_wait_sec = -1.0;
    bool post_imp_offset = false;
    ExecutionBlock paused_blk;

    {
      std::lock_guard<std::mutex> lock(exec_mutex_);
      prog = loaded_program_;
      block_idx = exec_block_idx_;
      resume_mid = paused_mid_block_;
      pause_sample = paused_sample_idx_;
      resume_wait_sec = paused_remaining_wait_sec_;
      post_imp_offset = post_impedance_offset_active_;
      paused_blk = paused_block_snapshot_;
    }

    if (!prog || prog->blocks.empty()) {
      StudioSupervisor::Effects eff;
      supervisor_->on_execution_fault(session, -1, "No compiled program loaded", false, eff);
      apply_effects(eff);
      return;
    }

    const size_t total_blocks = prog->blocks.size();
    std::vector<double> cum_weight(total_blocks + 1, 0.0);
    for (size_t i = 0; i < total_blocks; ++i) {
      cum_weight[i + 1] = cum_weight[i] + std::max(0.05, prog->blocks[i].duration_sec);
    }
    const double total_weight = std::max(1e-6, cum_weight.back());
    auto calc_total_pct = [&](size_t idx, double block_frac) -> double {
      const double w0 = cum_weight[std::min(idx, total_blocks)];
      const double w1 = (idx < total_blocks) ? cum_weight[idx + 1] : total_weight;
      const double pct = 100.0 * (w0 + std::clamp(block_frac, 0.0, 1.0) * (w1 - w0)) / total_weight;
      return std::clamp(pct, 0.0, 100.0);
    };

    while (rclcpp::ok() && !worker_exit_ && block_idx < total_blocks) {
      if (session != supervisor_->active_session()) {
        return;
      }
      if (supervisor_->stop_requested(session)) {
        if (!ensure_normal_or_fault(session, -38)) {
          return;
        }
        std::lock_guard<std::mutex> lock(exec_mutex_);
        reset_exec_cursor_unlocked();
        supervisor_->on_stop_completed(session);
        return;
      }
      if (supervisor_->pause_requested(session)) {
        if (!ensure_normal_or_fault(session, -38)) {
          return;
        }
        const auto & ref_blk = resume_mid ? paused_blk : prog->blocks[block_idx];
        std::lock_guard<std::mutex> lock(exec_mutex_);
        exec_block_idx_ = block_idx;
        paused_mid_block_ = resume_mid;
        paused_sample_idx_ = pause_sample;
        paused_remaining_wait_sec_ = resume_wait_sec;
        if (resume_mid) {
          paused_block_snapshot_ = paused_blk;
        }
        supervisor_->on_pause_settled(
          session, ref_blk.loc.line, ref_blk.loc.statement_index,
          static_cast<int32_t>(block_idx), 0.0, calc_total_pct(block_idx, 0.0));
        return;
      }

      ExecutionBlock block_to_run;
      bool used_mid_block_replan = false;
      if (resume_mid && paused_blk.type != ExecutionBlockType::WAIT_CMD) {
        std::array<double, 6> q_actual{};
        double max_dq = 0.0;
        if (!get_latest_joint_state(q_actual, max_dq)) {
          StudioSupervisor::Effects eff;
          supervisor_->on_execution_fault(
            session, -2, "Joint states unavailable during resume replanning", false, eff);
          apply_effects(eff);
          return;
        }
        std::string replan_err;
        if (!compiler_->replan_remaining_block(
              paused_blk, pause_sample, q_actual, block_to_run, replan_err))
        {
          StudioSupervisor::Effects eff;
          supervisor_->on_execution_fault(
            session, -3, "Mid-block resume replanning failed: " + replan_err, false, eff);
          apply_effects(eff);
          return;
        }
        used_mid_block_replan = true;
        resume_mid = false;
        pause_sample = 0;
      } else {
        block_to_run = prog->blocks[block_idx];
        if (resume_mid && block_to_run.type == ExecutionBlockType::WAIT_CMD && resume_wait_sec >= 0.0) {
          block_to_run.duration_sec = resume_wait_sec;
          block_to_run.wait_cmd.duration_sec = resume_wait_sec;
        }
        resume_mid = false;
        resume_wait_sec = -1.0;
      }

      // If a previous ImpedanceCartesian(..., hold=true) left the controller in
      // MODE_CARTESIAN_IMPEDANCE, return cleanly to MODE_NORMAL before dispatching
      // the next rigid or impedance trajectory (or alignment bridge).
      if (block_to_run.type == ExecutionBlockType::RIGID_TRAJECTORY ||
          block_to_run.type == ExecutionBlockType::IMPEDANCE_TRAJECTORY)
      {
        if (!ensure_normal_or_fault(session, -38)) {
          return;
        }
      }

      // Pre-execution joint alignment bridge or post-impedance start-pose replanning
      if (!used_mid_block_replan &&
          (block_to_run.type == ExecutionBlockType::RIGID_TRAJECTORY ||
           block_to_run.type == ExecutionBlockType::IMPEDANCE_TRAJECTORY))
      {
        std::array<double, 6> q_actual{};
        double max_dq = 0.0;
        if (!get_latest_joint_state(q_actual, max_dq)) {
          StudioSupervisor::Effects eff;
          supervisor_->on_execution_fault(
            session, -4, "Joint states unavailable before trajectory dispatch", false, eff);
          apply_effects(eff);
          return;
        }
        double max_diff = 0.0;
        for (size_t i = 0; i < 6; ++i) {
          max_diff = std::max(max_diff, std::abs(q_actual[i] - block_to_run.start_joints[i]));
        }

        if (post_imp_offset) {
          if (max_diff > 5e-4 && block_to_run.source_kind != BlockSourceKind::NONE) {
            const auto orig_end_joints = block_to_run.end_joints;
            ExecutionBlock replanned_blk;
            std::vector<Diagnostic> diags;
            if (!compiler_->replan_block_from_start(
                  block_to_run, q_actual, replanned_blk, diags))
            {
              const std::string reason = diags.empty() ?
                "Unknown planning error" : diags.front().message;
              StudioSupervisor::Effects eff;
              supervisor_->on_execution_fault(
                session, -5, "Post-impedance trajectory replanning failed: " + reason, false, eff);
              apply_effects(eff);
              return;
            }
            block_to_run = std::move(replanned_blk);
            if (block_to_run.type == ExecutionBlockType::RIGID_TRAJECTORY) {
              double end_diff = 0.0;
              for (size_t i = 0; i < 6; ++i) {
                end_diff = std::max(
                  end_diff, std::abs(block_to_run.end_joints[i] - orig_end_joints[i]));
              }
              if (end_diff <= 1e-3) {
                post_imp_offset = false;
              }
            }
          } else if (block_to_run.type == ExecutionBlockType::RIGID_TRAJECTORY && max_diff <= 5e-4) {
            post_imp_offset = false;
          }
        } else if (max_diff > kAlignmentThresholdRad) {
          trajectory_msgs::msg::JointTrajectory bridge_traj;
          std::string bridge_err;
          if (!compiler_->plan_alignment_bridge(
                q_actual, block_to_run.start_joints, 0.35, 0.35, bridge_traj, bridge_err))
          {
            StudioSupervisor::Effects eff;
            supervisor_->on_execution_fault(
              session, -5, "Pre-alignment planning failed: " + bridge_err, false, eff);
            apply_effects(eff);
            return;
          }
          FollowJointTrajectory::Goal bridge_goal;
          bridge_goal.trajectory = bridge_traj;
          bridge_goal.trajectory.header.stamp.sec = 0;
          bridge_goal.trajectory.header.stamp.nanosec = 0;

          supervisor_->update_execution_progress(
            session, block_to_run.loc.line, block_to_run.loc.statement_index,
            static_cast<int32_t>(block_idx), 0.0, calc_total_pct(block_idx, 0.0));

          const double bridge_dur =
            bridge_traj.points.back().time_from_start.sec +
            1e-9 * bridge_traj.points.back().time_from_start.nanosec;
          std::string exec_err;
          const auto outcome = run_controller_action<FollowJointTrajectory>(
            rigid_client_, bridge_goal, session, bridge_dur + 8.0,
            nullptr, true, true, exec_err);

          if (outcome == ActionOutcome::SESSION_SUPERSEDED) {
            return;
          }
          if (supervisor_->stop_requested(session)) {
            std::lock_guard<std::mutex> lock(exec_mutex_);
            reset_exec_cursor_unlocked();
            supervisor_->on_stop_completed(session);
            return;
          }
          if (outcome == ActionOutcome::CANCELED) {
            std::lock_guard<std::mutex> lock(exec_mutex_);
            exec_block_idx_ = block_idx;
            paused_mid_block_ = false;
            paused_sample_idx_ = 0;
            supervisor_->on_pause_settled(
              session, block_to_run.loc.line, block_to_run.loc.statement_index,
              static_cast<int32_t>(block_idx), 0.0, calc_total_pct(block_idx, 0.0));
            return;
          }
          if (outcome == ActionOutcome::FAULT) {
            handle_action_fault(session, exec_err);
            return;
          }
        }
      }

      // Dispatch actual execution block
      if (block_to_run.type == ExecutionBlockType::RIGID_TRAJECTORY) {
        if (!execute_rigid_block(session, prog, block_to_run, block_idx, calc_total_pct)) {
          return;
        }
      } else if (block_to_run.type == ExecutionBlockType::IMPEDANCE_TRAJECTORY) {
        if (!execute_impedance_block(session, prog, block_to_run, block_idx, calc_total_pct)) {
          return;
        }
        post_imp_offset = true;
      } else if (block_to_run.type == ExecutionBlockType::GRIPPER_CMD) {
        if (!execute_gripper_block(session, block_to_run, block_idx, calc_total_pct)) {
          return;
        }
      } else if (block_to_run.type == ExecutionBlockType::WAIT_CMD) {
        if (!execute_wait_block(session, block_to_run, block_idx, calc_total_pct)) {
          return;
        }
      } else if (block_to_run.type == ExecutionBlockType::PAYLOAD_CMD) {
        if (!execute_payload_block(session, block_to_run, block_idx, calc_total_pct)) {
          return;
        }
      }

      // Block succeeded: advance cursor
      if (session != supervisor_->active_session()) {
        return;
      }
      ++block_idx;
      {
        std::lock_guard<std::mutex> lock(exec_mutex_);
        exec_block_idx_ = block_idx;
        paused_mid_block_ = false;
        paused_sample_idx_ = 0;
        paused_remaining_wait_sec_ = -1.0;
        post_impedance_offset_active_ = post_imp_offset;
      }

      if (supervisor_->stop_requested(session)) {
        if (!ensure_normal_or_fault(session, -38)) {
          return;
        }
        std::lock_guard<std::mutex> lock(exec_mutex_);
        reset_exec_cursor_unlocked();
        supervisor_->on_stop_completed(session);
        return;
      }

      if (supervisor_->is_step_mode(session)) {
        if (!ensure_normal_or_fault(session, -38)) {
          return;
        }
        const bool more = block_idx < total_blocks;
        const int32_t next_line = more ? prog->blocks[block_idx].loc.line : 0;
        const int32_t next_stmt = more ? prog->blocks[block_idx].loc.statement_index : -1;
        supervisor_->on_step_block_done(
          session, more, static_cast<int32_t>(block_idx), next_line, next_stmt,
          calc_total_pct(block_idx, 0.0));
        return;
      }

      if (supervisor_->pause_requested(session) && block_idx < total_blocks) {
        if (!ensure_normal_or_fault(session, -38)) {
          return;
        }
        supervisor_->on_pause_settled(
          session, prog->blocks[block_idx].loc.line,
          prog->blocks[block_idx].loc.statement_index,
          static_cast<int32_t>(block_idx), 0.0, calc_total_pct(block_idx, 0.0));
        return;
      }
    }

    if (session == supervisor_->active_session()) {
      if (!ensure_normal_or_fault(session, -38)) {
        return;
      }
      {
        std::lock_guard<std::mutex> lock(exec_mutex_);
        reset_exec_cursor_unlocked();
      }
      supervisor_->on_program_completed(session);
    }
  }

  enum class SettleOutcome
  {
    SETTLED,
    ABORTED,
    TIMED_OUT
  };

  // Waits until every joint speed stays at or below the planner settle bound
  // (kPlannerSettleSpeedRadPerSec, strictly below the controller admission bound)
  // for a short continuous window, so 100 Hz SHM sampling jitter never lets a
  // command reach the 1 kHz controller while it would still be speed-rejected.
  SettleOutcome wait_for_planner_settle(uint64_t session, double timeout_sec)
  {
    constexpr auto kContinuous = std::chrono::milliseconds(50);
    const auto start = std::chrono::steady_clock::now();
    auto settled_since = std::chrono::steady_clock::time_point{};
    while (rclcpp::ok() && !worker_exit_) {
      if (session != supervisor_->active_session() || supervisor_->stop_requested(session)) {
        return SettleOutcome::ABORTED;
      }
      const auto now = std::chrono::steady_clock::now();
      std::array<double, 6> q_tmp{};
      double max_dq = 0.0;
      if (get_latest_joint_state(q_tmp, max_dq) &&
          max_dq <= lite6_interfaces::contract::kPlannerSettleSpeedRadPerSec)
      {
        if (settled_since == std::chrono::steady_clock::time_point{}) {
          settled_since = now;
        }
        if (now - settled_since >= kContinuous) {
          return SettleOutcome::SETTLED;
        }
      } else {
        settled_since = std::chrono::steady_clock::time_point{};
      }
      if (std::chrono::duration<double>(now - start).count() > timeout_sec) {
        return SettleOutcome::TIMED_OUT;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return SettleOutcome::ABORTED;
  }

  void handle_action_fault(uint64_t session, const std::string & err)
  {
    // Give /ctc_controller/status ~40 ms to deliver an E-STOP transition triggered by the abort
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    if (session != supervisor_->active_session() ||
        supervisor_->current_state() == StudioState::STATE_ESTOP)
    {
      return;
    }
    StudioSupervisor::Effects eff;
    supervisor_->on_execution_fault(session, -21, err, false, eff);
    apply_effects(eff);
  }

  bool execute_rigid_block(
    uint64_t session,
    const std::shared_ptr<const CompiledProgram> &,
    const ExecutionBlock & blk,
    size_t block_idx,
    const std::function<double(size_t, double)> & calc_total_pct)
  {
    const size_t total_pts = blk.rigid_trajectory.points.size();
    std::atomic<size_t> last_sample{0};

    FollowJointTrajectory::Goal goal;
    goal.trajectory = blk.rigid_trajectory;
    goal.trajectory.header.stamp.sec = 0;
    goal.trajectory.header.stamp.nanosec = 0;
    goal.trajectory.joint_names = kJointNames;

    auto fb_cb = [&](const FollowJointTrajectory::Feedback & fb) {
      if (total_pts == 0) {
        return;
      }
      const double t =
        fb.desired.time_from_start.sec + 1e-9 * fb.desired.time_from_start.nanosec;
      const size_t idx = std::min(
        total_pts - 1, static_cast<size_t>(std::max<long long>(0LL, std::llround(t * 1000.0))));
      last_sample.store(idx);
      const double frac =
        (total_pts > 1) ? static_cast<double>(idx) / static_cast<double>(total_pts - 1) : 1.0;
      const auto & loc = idx < blk.sample_locations.size() ? blk.sample_locations[idx] : blk.loc;
      supervisor_->update_execution_progress(
        session, loc.line, loc.statement_index, static_cast<int32_t>(block_idx),
        100.0 * frac, calc_total_pct(block_idx, frac));
    };

    // PAUSE: stop ON the programmed path by splicing a re-timed tail into the running goal.
    std::optional<size_t> splice_end_sample;
    auto on_pause = [&](const rclcpp_action::GoalUUID & goal_id) {
      return request_pause_splice(blk, goal_id, splice_end_sample);
    };
    std::string err;
    const auto outcome = run_controller_action<FollowJointTrajectory>(
      rigid_client_, goal, session, blk.duration_sec + 8.0, fb_cb, true, true, err, on_pause);
    if (outcome == ActionOutcome::SUCCEEDED && splice_end_sample.has_value()) {
      // The goal settled at the end of the stop tail: a pause in the middle of the block.
      return handle_trajectory_outcome(
        session, blk, block_idx, *splice_end_sample, total_pts, ActionOutcome::CANCELED, err,
        calc_total_pct);
    }
    return handle_trajectory_outcome(
      session, blk, block_idx, last_sample.load(), total_pts, outcome, err, calc_total_pct);
  }

  // Plans a stop-along-path tail for the running rigid goal and splices it into the
  // CTC execution (FollowJointTrajectory goal whose header.stamp is the running goal's
  // actual start). Falls back to CANCEL whenever the splice cannot be confirmed.
  PauseHandling request_pause_splice(
    const ExecutionBlock & blk,
    const rclcpp_action::GoalUUID & goal_id,
    std::optional<size_t> & out_end_sample)
  {
    std::array<uint8_t, 16> traj_id{};
    builtin_interfaces::msg::Time traj_start;
    {
      std::lock_guard<std::mutex> lock(traj_status_mutex_);
      traj_id = last_traj_id_;
      traj_start = last_traj_start_;
    }
    if (traj_id != goal_id || (traj_start.sec == 0 && traj_start.nanosec == 0)) {
      return PauseHandling::CANCEL;  // Not started yet (or start not yet observed).
    }
    const rclcpp::Time start(traj_start, RCL_ROS_TIME);
    auto block_time_now = [&]() { return (now() - start).seconds(); };

    trajectory_msgs::msg::JointTrajectory tail;
    size_t end_sample = 0;
    std::string err;
    const auto status = compiler_->plan_pause_splice(
      blk, block_time_now() + kPauseSpliceLookaheadSec, tail, end_sample, err);
    if (status == ProgramCompiler::PauseSpliceStatus::LET_FINISH) {
      return PauseHandling::LET_FINISH;
    }
    if (status != ProgramCompiler::PauseSpliceStatus::OK) {
      RCLCPP_WARN(get_logger(), "Pause splice planning failed (%s); canceling instead", err.c_str());
      return PauseHandling::CANCEL;
    }
    const double t_switch = tail.points.front().time_from_start.sec +
      1e-9 * tail.points.front().time_from_start.nanosec;
    tail.header.stamp = traj_start;
    tail.joint_names = kJointNames;

    FollowJointTrajectory::Goal splice_goal;
    splice_goal.trajectory = std::move(tail);
    auto gh_future = rigid_client_->async_send_goal(splice_goal);
    auto time_left = [&]() {
      return std::chrono::duration<double>(std::max(0.0, t_switch - block_time_now()));
    };
    if (gh_future.wait_for(time_left()) != std::future_status::ready) {
      return PauseHandling::CANCEL;
    }
    const auto splice_gh = gh_future.get();
    if (!splice_gh) {
      RCLCPP_WARN(get_logger(), "Controller rejected the pause splice; canceling instead");
      return PauseHandling::CANCEL;
    }
    // The CTC resolves the splice at its switch time (result relayed within ~20 ms).
    auto res_future = rigid_client_->async_get_result(splice_gh);
    if (res_future.wait_for(time_left() + std::chrono::milliseconds(300)) !=
        std::future_status::ready)
    {
      return PauseHandling::CANCEL;
    }
    const auto wrapped = res_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(get_logger(), "Pause splice was not adopted in time; canceling instead");
      return PauseHandling::CANCEL;
    }
    out_end_sample = end_sample;
    return PauseHandling::SPLICED;
  }

  bool execute_impedance_block(
    uint64_t session,
    const std::shared_ptr<const CompiledProgram> &,
    const ExecutionBlock & blk,
    size_t block_idx,
    const std::function<double(size_t, double)> & calc_total_pct)
  {
    // Confirm settling below the planner bound so the CTC entry speed guard passes cleanly.
    const auto settle = wait_for_planner_settle(session, 2.0);
    if (settle == SettleOutcome::ABORTED) {
      return handle_trajectory_outcome(
        session, blk, block_idx, 0, blk.impedance_goal.points.size(),
        session != supervisor_->active_session() ?
        ActionOutcome::SESSION_SUPERSEDED : ActionOutcome::CANCELED,
        "", calc_total_pct);
    }
    if (settle == SettleOutcome::TIMED_OUT) {
      StudioSupervisor::Effects eff;
      supervisor_->on_execution_fault(
        session, -39, "Arm did not settle below the planner settle speed before impedance entry",
        false, eff);
      apply_effects(eff);
      return false;
    }

    const size_t total_pts = blk.impedance_goal.points.size();
    std::atomic<size_t> last_sample{0};

    auto fb_cb = [&](const FollowImpedanceTrajectory::Feedback & fb) {
      if (total_pts == 0) {
        return;
      }
      const size_t idx = std::min(total_pts - 1, static_cast<size_t>(fb.current_sample));
      last_sample.store(idx);
      const double frac =
        (total_pts > 1) ? static_cast<double>(idx) / static_cast<double>(total_pts - 1) : 1.0;
      const auto & loc = idx < blk.sample_locations.size() ? blk.sample_locations[idx] : blk.loc;
      supervisor_->update_execution_progress(
        session, loc.line, loc.statement_index, static_cast<int32_t>(block_idx),
        100.0 * frac, calc_total_pct(block_idx, frac));
      supervisor_->update_impedance_errors(session, fb.position_error_m, fb.rotation_error_rad);
    };

    auto imp_goal = blk.impedance_goal;
    imp_goal.expected_controller_epoch = supervisor_->snapshot().controller_epoch;
    std::string err;
    const auto outcome = run_controller_action<FollowImpedanceTrajectory>(
      imp_client_, imp_goal, session, blk.duration_sec + 12.0, fb_cb, false, true, err);
    supervisor_->clear_impedance_errors();
    return handle_trajectory_outcome(
      session, blk, block_idx, last_sample.load(), total_pts, outcome, err, calc_total_pct);
  }

  bool handle_trajectory_outcome(
    uint64_t session,
    const ExecutionBlock & blk,
    size_t block_idx,
    size_t sample_idx,
    size_t total_pts,
    ActionOutcome outcome,
    const std::string & err,
    const std::function<double(size_t, double)> & calc_total_pct)
  {
    if (outcome == ActionOutcome::SESSION_SUPERSEDED) {
      return false;
    }
    if (outcome == ActionOutcome::SUCCEEDED) {
      return true;
    }
    if (supervisor_->stop_requested(session)) {
      if (!ensure_normal_or_fault(session, -38)) {
        return false;
      }
      std::lock_guard<std::mutex> lock(exec_mutex_);
      reset_exec_cursor_unlocked();
      supervisor_->on_stop_completed(session);
      return false;
    }
    if (outcome == ActionOutcome::CANCELED) {
      if (!ensure_normal_or_fault(session, -38)) {
        return false;
      }
      const size_t safe_idx = (total_pts > 0) ? std::min(sample_idx, total_pts - 1) : 0;
      const double frac =
        (total_pts > 1) ? static_cast<double>(safe_idx) / static_cast<double>(total_pts - 1) : 0.0;
      const auto & loc = safe_idx < blk.sample_locations.size() ?
        blk.sample_locations[safe_idx] : blk.loc;
      {
        std::lock_guard<std::mutex> lock(exec_mutex_);
        exec_block_idx_ = block_idx;
        paused_mid_block_ = true;
        paused_sample_idx_ = safe_idx;
        paused_block_snapshot_ = blk;
        if (blk.type == ExecutionBlockType::IMPEDANCE_TRAJECTORY) {
          post_impedance_offset_active_ = true;
        }
      }
      supervisor_->on_pause_settled(
        session, loc.line, loc.statement_index, static_cast<int32_t>(block_idx),
        100.0 * frac, calc_total_pct(block_idx, frac));
      return false;
    }
    handle_action_fault(session, err);
    return false;
  }

  bool execute_gripper_block(
    uint64_t session,
    const ExecutionBlock & blk,
    size_t block_idx,
    const std::function<double(size_t, double)> & calc_total_pct)
  {
    // PAUSE is not honored inside a gripper block; let the command handler say so.
    gripper_block_active_.store(true, std::memory_order_release);
    ScopeExit clear_gripper_flag{[this]() {
      gripper_block_active_.store(false, std::memory_order_release);
    }};
    supervisor_->update_execution_progress(
      session, blk.loc.line, blk.loc.statement_index,
      static_cast<int32_t>(block_idx), 10.0, calc_total_pct(block_idx, 0.1));

    if (!gripper_client_->wait_for_service(std::chrono::seconds(2))) {
      StudioSupervisor::Effects eff;
      supervisor_->on_execution_fault(
        session, -30, "Gripper service /gripper/command is unavailable", false, eff);
      apply_effects(eff);
      return false;
    }

    auto req = std::make_shared<GripperCommand::Request>();
    req->command = static_cast<uint8_t>(blk.gripper_cmd.action);
    auto future = gripper_client_->async_send_request(req);
    const auto req_start = std::chrono::steady_clock::now();
    while (future.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) {
      if (session != supervisor_->active_session() || !rclcpp::ok()) {
        return false;
      }
      if (std::chrono::steady_clock::now() - req_start > std::chrono::seconds(3)) {
        StudioSupervisor::Effects eff;
        supervisor_->on_execution_fault(
          session, -31, "Timed out waiting for /gripper/command service response", false, eff);
        apply_effects(eff);
        return false;
      }
    }

    const auto resp = future.get();
    if (!resp || !resp->accepted) {
      StudioSupervisor::Effects eff;
      supervisor_->on_execution_fault(
        session, -32,
        resp ? ("Gripper command rejected: " + resp->detail) : "Null gripper response",
        false, eff);
      apply_effects(eff);
      return false;
    }

    if (!blk.gripper_cmd.wait_for_completion) {
      return true;
    }

    const uint64_t target_seq = resp->sequence;
    const auto wait_deadline =
      std::chrono::steady_clock::now() +
      std::chrono::milliseconds(
        static_cast<int64_t>(std::max(0.5, blk.gripper_cmd.timeout_sec) * 1000.0));

    while (rclcpp::ok() && session == supervisor_->active_session()) {
      if (supervisor_->stop_requested(session)) {
        if (!ensure_normal_or_fault(session, -38)) {
          return false;
        }
        std::lock_guard<std::mutex> lock(exec_mutex_);
        reset_exec_cursor_unlocked();
        supervisor_->on_stop_completed(session);
        return false;
      }

      GripperState gs;
      bool have_gs = false;
      {
        std::lock_guard<std::mutex> lock(state_data_mutex_);
        if (has_latest_gripper_) {
          gs = latest_gripper_;
          have_gs = true;
        }
      }

      if (have_gs && gs.applied_sequence >= target_seq) {
        const bool seq_exact = (gs.applied_sequence == target_seq);
        if (gs.state == GripperState::FAULT ||
            (seq_exact &&
             (gs.last_cmd_result == GripperState::CMD_RES_FAULT ||
              gs.last_cmd_result == GripperState::CMD_RES_BLOCKED)))
        {
          StudioSupervisor::Effects eff;
          supervisor_->on_execution_fault(
            session, -33, "Gripper command failed: " + gs.detail, false, eff);
          apply_effects(eff);
          return false;
        }
        if (seq_exact &&
            gs.last_cmd_result == GripperState::CMD_RES_STOPPED &&
            blk.gripper_cmd.action != GripperActionType::STOP)
        {
          StudioSupervisor::Effects eff;
          supervisor_->on_execution_fault(
            session, -33, "Gripper motion stopped externally before completion", false, eff);
          apply_effects(eff);
          return false;
        }
        bool done = seq_exact && (gs.last_cmd_result == GripperState::CMD_RES_SUCCEEDED);
        if (!done) {
          if (blk.gripper_cmd.action == GripperActionType::OPEN &&
              gs.state == GripperState::OPEN)
          {
            done = true;
          } else if (
            blk.gripper_cmd.action == GripperActionType::CLOSE &&
            (gs.state == GripperState::CLOSED || gs.state == GripperState::GRASPED))
          {
            done = true;
          } else if (
            blk.gripper_cmd.action == GripperActionType::STOP &&
            gs.state == GripperState::STOPPED)
          {
            done = true;
          }
        }
        if (done) {
          supervisor_->update_execution_progress(
            session, blk.loc.line, blk.loc.statement_index,
            static_cast<int32_t>(block_idx), 100.0, calc_total_pct(block_idx, 1.0));
          return true;
        }
      }

      if (std::chrono::steady_clock::now() > wait_deadline) {
        StudioSupervisor::Effects eff;
        supervisor_->on_execution_fault(
          session, -34, "Gripper completion wait timed out", false, eff);
        apply_effects(eff);
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  bool execute_wait_block(
    uint64_t session,
    const ExecutionBlock & blk,
    size_t block_idx,
    const std::function<double(size_t, double)> & calc_total_pct)
  {
    const double dur = std::max(0.0, blk.wait_cmd.duration_sec);
    const auto start_tp = std::chrono::steady_clock::now();
    while (rclcpp::ok() && session == supervisor_->active_session()) {
      const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_tp).count();
      if (elapsed >= dur) {
        supervisor_->update_execution_progress(
          session, blk.loc.line, blk.loc.statement_index,
          static_cast<int32_t>(block_idx), 100.0, calc_total_pct(block_idx, 1.0));
        return true;
      }
      if (supervisor_->stop_requested(session)) {
        if (!ensure_normal_or_fault(session, -38)) {
          return false;
        }
        std::lock_guard<std::mutex> lock(exec_mutex_);
        reset_exec_cursor_unlocked();
        supervisor_->on_stop_completed(session);
        return false;
      }
      if (supervisor_->pause_requested(session)) {
        if (!ensure_normal_or_fault(session, -38)) {
          return false;
        }
        const double remaining = std::max(0.0, dur - elapsed);
        const double frac = (dur > 1e-6) ? std::clamp(elapsed / dur, 0.0, 1.0) : 1.0;
        {
          std::lock_guard<std::mutex> lock(exec_mutex_);
          exec_block_idx_ = block_idx;
          paused_mid_block_ = true;
          paused_remaining_wait_sec_ = remaining;
          paused_block_snapshot_ = blk;
        }
        supervisor_->on_pause_settled(
          session, blk.loc.line, blk.loc.statement_index,
          static_cast<int32_t>(block_idx), 100.0 * frac, calc_total_pct(block_idx, frac));
        return false;
      }
      const double frac = (dur > 1e-6) ? std::clamp(elapsed / dur, 0.0, 1.0) : 1.0;
      supervisor_->update_execution_progress(
        session, blk.loc.line, blk.loc.statement_index,
        static_cast<int32_t>(block_idx), 100.0 * frac, calc_total_pct(block_idx, frac));
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  bool execute_payload_block(
    uint64_t session,
    const ExecutionBlock & blk,
    size_t block_idx,
    const std::function<double(size_t, double)> & calc_total_pct)
  {
    supervisor_->update_execution_progress(
      session, blk.loc.line, blk.loc.statement_index,
      static_cast<int32_t>(block_idx), 20.0, calc_total_pct(block_idx, 0.2));

    // Confirm settling below the planner bound so the controller's busy flag has cleared.
    const auto settle = wait_for_planner_settle(session, 2.0);
    if (settle == SettleOutcome::ABORTED) {
      if (session == supervisor_->active_session() && supervisor_->stop_requested(session)) {
        if (ensure_normal_or_fault(session, -38)) {
          std::lock_guard<std::mutex> lock(exec_mutex_);
          reset_exec_cursor_unlocked();
          supervisor_->on_stop_completed(session);
        }
      }
      return false;
    }
    if (settle == SettleOutcome::TIMED_OUT) {
      StudioSupervisor::Effects eff;
      supervisor_->on_execution_fault(
        session, -40, "Arm did not settle below the planner settle speed before SetPayload",
        false, eff);
      apply_effects(eff);
      return false;
    }

    if (!ctc_cmd_client_->wait_for_service(std::chrono::milliseconds(250))) {
      StudioSupervisor::Effects eff;
      supervisor_->on_execution_fault(
        session, -35, "/ctc_controller/command service unavailable for SetPayload", false, eff);
      apply_effects(eff);
      return false;
    }

    auto req = std::make_shared<ControllerCommand::Request>();
    req->command = ControllerCommand::Request::CMD_SET_PAYLOAD;
    req->expected_epoch = supervisor_->snapshot().controller_epoch;
    req->payload_mass = blk.payload_cmd.mass_kg;
    req->payload_com_tcp = blk.payload_cmd.com_tcp;
    req->reason = "Studio DSL SetPayload";

    auto future = ctc_cmd_client_->async_send_request(req);
    if (future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
      ctc_cmd_client_->remove_pending_request(future);
      StudioSupervisor::Effects eff;
      supervisor_->on_execution_fault(
        session, -36, "Timed out waiting for /ctc_controller/command SetPayload response",
        false, eff);
      apply_effects(eff);
      return false;
    }

    const auto resp = future.get();
    if (!resp || !resp->accepted) {
      StudioSupervisor::Effects eff;
      supervisor_->on_execution_fault(
        session, -37,
        resp ? ("Controller rejected SetPayload: " + resp->message) : "Null SetPayload response",
        false, eff);
      apply_effects(eff);
      return false;
    }

    supervisor_->update_execution_progress(
      session, blk.loc.line, blk.loc.statement_index,
      static_cast<int32_t>(block_idx), 100.0, calc_total_pct(block_idx, 1.0));
    return true;
  }

  lite6_planner::KinematicsEngine kinematics_;
  std::unique_ptr<ProgramCompiler> compiler_;
  std::unique_ptr<StudioSupervisor> supervisor_;

  rclcpp::CallbackGroup::SharedPtr telemetry_group_;
  rclcpp::CallbackGroup::SharedPtr shm_poll_group_;
  rclcpp::CallbackGroup::SharedPtr compile_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr client_group_;
  rclcpp::CallbackGroup::SharedPtr ctc_cmd_group_;

  lite6_interfaces::shm::ShmHandle<lite6_interfaces::shm::StateShmSegment> state_shm_;
  uint64_t last_joint_seq_ = 0;
  int64_t last_joint_steady_ns_ = 0;
  rclcpp::TimerBase::SharedPtr joint_poll_timer_;

  rclcpp::Publisher<StudioState>::SharedPtr state_pub_;
  rclcpp::Client<ControllerCommand>::SharedPtr ctc_cmd_client_;
  rclcpp::Subscription<ControllerStatus>::SharedPtr controller_sub_;
  rclcpp::Subscription<GripperState>::SharedPtr gripper_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr rigid_client_;
  rclcpp_action::Client<FollowImpedanceTrajectory>::SharedPtr imp_client_;
  rclcpp::Client<GripperCommand>::SharedPtr gripper_client_;
  rclcpp::Service<CompileProgram>::SharedPtr compile_srv_;
  rclcpp::Service<StudioCommand>::SharedPtr command_srv_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::mutex pub_mutex_;
  uint64_t last_published_revision_ = 0;
  std::mutex command_mutex_;
  std::mutex compile_mutex_;
  std::atomic<bool> gripper_block_active_{false};

  mutable std::mutex state_data_mutex_;
  std::array<double, 6> latest_q_{};
  std::array<double, 6> latest_dq_{};
  bool has_latest_joint_ = false;
  GripperState latest_gripper_;
  bool has_latest_gripper_ = false;

  std::mutex goal_mutex_;
  rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr active_rigid_goal_;
  rclcpp_action::ClientGoalHandle<FollowImpedanceTrajectory>::SharedPtr active_imp_goal_;

  std::mutex exec_mutex_;
  std::shared_ptr<const CompiledProgram> loaded_program_;
  uint64_t program_revision_ = 0;
  size_t exec_block_idx_ = 0;
  bool paused_mid_block_ = false;
  // Latest controller-reported trajectory correlation (ControllerStatus).
  std::mutex traj_status_mutex_;
  std::array<uint8_t, 16> last_traj_id_{};
  builtin_interfaces::msg::Time last_traj_start_;
  size_t paused_sample_idx_ = 0;
  double paused_remaining_wait_sec_ = -1.0;
  ExecutionBlock paused_block_snapshot_;
  bool post_impedance_offset_active_ = false;

  std::mutex worker_wake_mutex_;
  std::condition_variable worker_cv_;
  bool worker_wake_pending_ = false;
  bool worker_exit_ = false;
  std::thread worker_thread_;
};

}  // namespace lite6_studio_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lite6_studio_planner::StudioPlannerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
