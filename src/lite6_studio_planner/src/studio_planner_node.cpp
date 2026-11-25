#include "lite6_kinematics/kinematics_engine.hpp"
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
#include "lite6_interfaces/srv/splice_joint_trajectory.hpp"
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

constexpr double kAlignmentThresholdRad = 5e-4;
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
  using SpliceJointTrajectory = lite6_interfaces::srv::SpliceJointTrajectory;
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
    const auto kin_pkg = ament_index_cpp::get_package_share_directory("lite6_kinematics");
    if (!kinematics_.load_ik_config(kin_pkg + "/config/ik_config.yaml") ||
        !kinematics_.init_robot_model(
          desc_pkg + "/urdf/lite6_arm.urdf", kin_pkg + "/config/lite6.srdf") ||
        !kinematics_.load_limits(kin_pkg + "/config/kinematic_limits.yaml"))
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

    const auto shm_status = state_shm_.open_status(false);
    if (shm_status == lite6_interfaces::shm::ShmOpenStatus::INCOMPATIBLE ||
        shm_status == lite6_interfaces::shm::ShmOpenStatus::OS_ERROR)
    {
      throw std::runtime_error(
        std::string("Failed to open read-only state shared memory: ") +
        lite6_interfaces::shm::shm_open_status_name(shm_status));
    }
    if (shm_status == lite6_interfaces::shm::ShmOpenStatus::NOT_FOUND) {
      RCLCPP_INFO(
        get_logger(),
        "State shared memory not created yet; waiting lazily for hardware interface");
    }

    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    state_pub_ = create_publisher<StudioState>("/studio/state", state_qos);
    ctc_cmd_client_ = create_client<ControllerCommand>(
      "/ctc_controller/command", rclcpp::ServicesQoS().keep_last(64), ctc_cmd_group_);

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
    splice_client_ = create_client<SpliceJointTrajectory>(
      "/ctc_controller/splice_joint_trajectory", rclcpp::ServicesQoS(), client_group_);
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
    RCLCPP_INFO(get_logger(), "Trajectory Programming Planner node started");
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
  template <typename ActionT>
  struct InFlightAction
  {
    std::mutex mutex;
    bool cancelled = false;
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr handle;
  };

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
    msg.planner_epoch = snap.planner_epoch;
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
      // No response: outcome unknown (callers must reconcile via observed telemetry).
      out_ack = ControllerCommand::Response::ACK_OUTCOME_UNKNOWN;
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

  void send_gripper_stop(bool wait_ack)
  {
    if (!gripper_client_ || !gripper_client_->service_is_ready()) {
      return;
    }
    auto req = std::make_shared<GripperCommand::Request>();
    req->command = GripperCommand::Request::STOP;
    std::optional<rclcpp::Client<GripperCommand>::FutureAndRequestId> future;
    {
      // Serialize gripper STOP with guarded worker dispatches so a superseded worker
      // can never issue OPEN/CLOSE after an E-STOP or STOP gripper halt.
      std::lock_guard<std::mutex> lock(dispatch_mutex_);
      future.emplace(gripper_client_->async_send_request(req));
    }
    if (wait_ack && future.has_value()) {
      if (future->future.wait_for(std::chrono::milliseconds(250)) != std::future_status::ready) {
        gripper_client_->remove_pending_request(*future);
      }
    }
  }

  void apply_effects(const StudioSupervisor::Effects & effects)
  {
    if (effects.send_estop_cmd) {
      send_controller_command_async(
        ControllerCommand::Request::CMD_ESTOP, 0, "Trajectory supervisor E-STOP");
    }
    if ((effects.send_estop_cmd || effects.cancel_controller_goals) &&
        gripper_block_active_.load(std::memory_order_acquire))
    {
      send_gripper_stop(false);
    }
    if (effects.send_enter_normal_cmd) {
      send_controller_command_async(
        ControllerCommand::Request::CMD_ENTER_NORMAL, effects.controller_epoch,
        "Trajectory supervisor enter NORMAL");
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
    std::shared_ptr<InFlightAction<FollowJointTrajectory>> rigid_disp;
    std::shared_ptr<InFlightAction<FollowImpedanceTrajectory>> imp_disp;
    {
      std::lock_guard<std::mutex> lock(dispatch_mutex_);
      rigid_disp = active_rigid_dispatch_;
      imp_disp = active_imp_dispatch_;
    }
    if (rigid_disp) {
      rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr rigid_gh;
      {
        std::lock_guard<std::mutex> lock(rigid_disp->mutex);
        rigid_disp->cancelled = true;
        rigid_gh = rigid_disp->handle;
      }
      if (rigid_gh) {
        try {
          (void)rigid_client_->async_cancel_goal(rigid_gh);
        } catch (const std::exception &) {
        }
      }
    }
    if (imp_disp) {
      rclcpp_action::ClientGoalHandle<FollowImpedanceTrajectory>::SharedPtr imp_gh;
      {
        std::lock_guard<std::mutex> lock(imp_disp->mutex);
        imp_disp->cancelled = true;
        imp_gh = imp_disp->handle;
      }
      if (imp_gh) {
        try {
          (void)imp_client_->async_cancel_goal(imp_gh);
        } catch (const std::exception &) {
        }
      }
    }
  }

  void poll_joint_state_shm()
  {
    if (!state_shm_.valid()) {
      const auto status = state_shm_.open_status(false);
      if (status != lite6_interfaces::shm::ShmOpenStatus::OK) {
        if (status == lite6_interfaces::shm::ShmOpenStatus::INCOMPATIBLE ||
            status == lite6_interfaces::shm::ShmOpenStatus::OS_ERROR)
        {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "State shared memory open failed (%s); refusing to attach",
            lite6_interfaces::shm::shm_open_status_name(status));
        }
        return;
      }
    }
    lite6_interfaces::shm::JointStatePod pod{};
    uint64_t seq = 0;
    if (!state_shm_->joint_state.read(pod, seq) || pod.valid == 0) {
      return;
    }
    const int64_t now_ns = lite6_interfaces::shm::steady_now_ns();
    if (!lite6_interfaces::shm::is_stamp_fresh(
          pod.steady_stamp_ns, now_ns, lite6_interfaces::contract::kJointStateStaleTimeoutNs))
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
      latest_joint_steady_ns_ = pod.steady_stamp_ns;
      has_latest_joint_ = true;
    }

    const auto effects =
      supervisor_->observe_joint_state(pos_valid, max_abs_dq, dq_valid, pod.steady_stamp_ns);
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
    if (ctc_cmd_client_) {
      ctc_cmd_client_->prune_requests_older_than(
        std::chrono::system_clock::now() - std::chrono::seconds(5));
    }
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
          lite6_interfaces::shm::is_stamp_fresh(
            pod.steady_stamp_ns, now_ns, lite6_interfaces::contract::kJointStateStaleTimeoutNs))
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
    const int64_t now_ns = lite6_interfaces::shm::steady_now_ns();
    if (!has_latest_joint_ ||
        !lite6_interfaces::shm::is_stamp_fresh(
          latest_joint_steady_ns_, now_ns, lite6_interfaces::contract::kJointStateStaleTimeoutNs))
    {
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
    resp->planner_epoch = supervisor_->planner_epoch();
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
    const bool auto_load = req->auto_load;
    ScopeExit end_compiling{[this, auto_load]() { supervisor_->end_compile(auto_load); }};

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
    constexpr auto kCompileWallTimeout = std::chrono::seconds(45);
    const auto compile_deadline = std::chrono::steady_clock::now() + kCompileWallTimeout;
    std::atomic<bool> timed_out{false};
    auto compile_cancel = [this, compile_deadline, &timed_out]() {
      if (!rclcpp::ok() || worker_exit_.load(std::memory_order_acquire)) {
        return true;
      }
      if (std::chrono::steady_clock::now() >= compile_deadline) {
        timed_out.store(true, std::memory_order_relaxed);
        return true;
      }
      return false;
    };

    auto handle_compile_failure = [&](std::vector<Diagnostic> & fail_diags) {
      if (req->auto_load) {
        {
          std::lock_guard<std::mutex> exec_lock(exec_mutex_);
          loaded_program_.reset();
          reset_exec_cursor_unlocked();
        }
        supervisor_->on_compile_failed(req->request_id);
      }
      DSLParser::enforce_diagnostic_budget(fail_diags);
      resp->success = false;
      resp->diagnostics = std::move(fail_diags);
    };

    bool ok = false;
    try {
      ok = compiler_->compile(req->source_code, start_joints, *compiled, diags, compile_cancel);
    } catch (const std::bad_alloc &) {
      compiled.reset();
      diags.clear();
      RCLCPP_ERROR(get_logger(), "Compilation aborted due to memory allocation limit (std::bad_alloc)");
      Diagnostic d;
      d.severity = Diagnostic::SEVERITY_ERROR;
      d.stage = Diagnostic::STAGE_PLANNING;
      d.line_number = 1;
      d.column_number = 1;
      d.statement_index = -1;
      d.code = "ERR_COMPILE_MEMORY";
      d.message = "Compilation exceeded available memory budget";
      diags.push_back(std::move(d));
      handle_compile_failure(diags);
      return;
    } catch (const std::exception & e) {
      compiled.reset();
      diags.clear();
      RCLCPP_ERROR(get_logger(), "Unexpected exception during compilation: %s", e.what());
      Diagnostic d;
      d.severity = Diagnostic::SEVERITY_ERROR;
      d.stage = Diagnostic::STAGE_PLANNING;
      d.line_number = 1;
      d.column_number = 1;
      d.statement_index = -1;
      d.code = "ERR_COMPILE_INTERNAL";
      d.message = DSLParser::sanitize_bounded_text(
        std::string("Internal compilation error: ") + e.what(),
        DSLParser::kMaxDiagnosticMessageChars);
      diags.push_back(std::move(d));
      handle_compile_failure(diags);
      return;
    } catch (...) {
      compiled.reset();
      diags.clear();
      RCLCPP_ERROR(get_logger(), "Unknown non-standard exception during compilation");
      Diagnostic d;
      d.severity = Diagnostic::SEVERITY_ERROR;
      d.stage = Diagnostic::STAGE_PLANNING;
      d.line_number = 1;
      d.column_number = 1;
      d.statement_index = -1;
      d.code = "ERR_COMPILE_INTERNAL";
      d.message = "Unknown internal compilation error";
      diags.push_back(std::move(d));
      handle_compile_failure(diags);
      return;
    }

    if (!ok) {
      if (timed_out.load(std::memory_order_relaxed)) {
        for (auto & d : diags) {
          if (d.code == "ERR_CANCELLED") {
            d.code = "ERR_COMPILE_TIMEOUT";
            d.message = "Compilation exceeded wall-clock time budget (45 s)";
          }
        }
      }
      handle_compile_failure(diags);
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

    DSLParser::enforce_diagnostic_budget(diags);
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

    // Hold exec_mutex_ and dispatch_mutex_ across the supervisor transition (lock order:
    // command_mutex_ -> exec_mutex_ -> dispatch_mutex_ -> supervisor_->mutex_) so the worker
    // can never wake on a stale cursor OR dispatch a goal/gripper/payload command after
    // STOP/ESTOP/PAUSE has been accepted.
    StudioSupervisor::CommandResult result;
    {
      std::lock_guard<std::mutex> exec_lock(exec_mutex_);
      {
        std::lock_guard<std::mutex> dispatch_lock(dispatch_mutex_);
        result = supervisor_->handle_command(
          cmd, req->expected_planner_epoch, req->expected_revision, req->reason);
      }
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
            result.state != StudioState::STATE_STOPPING)))
      {
        reset_exec_cursor_unlocked();
        if (cmd == StudioCommand::Request::CMD_CLEAR_PROGRAM) {
          loaded_program_.reset();
        }
      }
    }
    apply_effects(result.effects);

    resp->accepted = result.accepted;
    resp->planner_epoch = result.planner_epoch;
    resp->revision = result.revision;
    resp->state = result.state;
    resp->controller_epoch = result.controller_epoch;
    resp->message = result.message;

    if (result.accepted && cmd == StudioCommand::Request::CMD_PAUSE &&
        gripper_block_active_.load(std::memory_order_acquire))
    {
      const std::string note =
        "Pause requested; it takes effect after the active gripper command completes";
      supervisor_->note_pause_deferred(
        note, gripper_active_timeout_sec_.load(std::memory_order_acquire));
      resp->message = note;
    }
  }

  void reset_exec_cursor_unlocked()
  {
    exec_block_idx_ = 0;
    paused_mid_block_ = false;
    paused_sample_idx_ = 0;
    paused_remaining_wait_sec_ = -1.0;
    paused_block_snapshot_.reset();
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
    std::function<PauseHandling(const rclcpp_action::GoalUUID &)> on_pause = nullptr,
    bool * out_goal_dispatched = nullptr)
  {
    if (out_goal_dispatched) {
      *out_goal_dispatched = false;
    }
    const auto start_tp = std::chrono::steady_clock::now();
    while (!client->action_server_is_ready()) {
      if (!rclcpp::ok() || worker_exit_.load() || session != supervisor_->active_session()) {
        return ActionOutcome::SESSION_SUPERSEDED;
      }
      if (honor_pause_stop && !supervisor_->can_dispatch(session, true)) {
        return ActionOutcome::CANCELED;
      }
      if (std::chrono::steady_clock::now() - start_tp > std::chrono::seconds(3)) {
        out_error = "Controller action server is not ready";
        return ActionOutcome::FAULT;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    auto dispatch = std::make_shared<InFlightAction<ActionT>>();

    struct FeedbackGate {
      std::mutex mutex;
      bool active = true;
      bool completed = false;
      typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr accepted_goal;
      std::function<void(const typename ActionT::Feedback &)> callback;
    };
    auto feedback_gate = std::make_shared<FeedbackGate>();
    feedback_gate->callback = std::move(on_feedback);
    ScopeExit close_feedback{[feedback_gate, client]() {
      typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr accepted;
      bool completed;
      {
        // Wait for in-flight feedback before the caller's execution context is destroyed.
        std::lock_guard<std::mutex> lock(feedback_gate->mutex);
        feedback_gate->active = false;
        feedback_gate->callback = nullptr;
        accepted = std::move(feedback_gate->accepted_goal);
        completed = feedback_gate->completed;
      }
      if (accepted) {
        if (!completed) {
          try {
            (void)client->async_cancel_goal(accepted);
          } catch (const std::exception &) {
          }
        }
        client->stop_callbacks(accepted);
      }
    }};

    typename rclcpp_action::Client<ActionT>::SendGoalOptions opts;
    opts.goal_response_callback =
      [client, feedback_gate, dispatch](
        const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr & accepted) {
        if (!accepted) {
          return;
        }
        bool cancel_requested = false;
        {
          std::lock_guard<std::mutex> lock(dispatch->mutex);
          dispatch->handle = accepted;
          cancel_requested = dispatch->cancelled;
        }
        bool abandoned;
        {
          std::lock_guard<std::mutex> lock(feedback_gate->mutex);
          abandoned = !feedback_gate->active;
          if (!abandoned) {
            feedback_gate->accepted_goal = accepted;
          }
        }
        if (abandoned || cancel_requested) {
          try {
            (void)client->async_cancel_goal(accepted);
          } catch (const std::exception &) {
          }
          if (abandoned) {
            client->stop_callbacks(accepted);
          }
        }
      };
    if (feedback_gate->callback) {
      opts.feedback_callback =
        [feedback_gate](
          typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
          const std::shared_ptr<const typename ActionT::Feedback> feedback) {
          std::lock_guard<std::mutex> lock(feedback_gate->mutex);
          if (feedback && feedback_gate->active) {
            feedback_gate->callback(*feedback);
          }
        };
    }

    (void)is_rigid_action;
    std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr> gh_future;
    {
      // Guarded dispatch: atomically revalidate session/STOP/PAUSE and register the in-flight
      // dispatch under dispatch_mutex_ so STOP/ESTOP/PAUSE can never be raced by a new goal.
      std::lock_guard<std::mutex> lock(dispatch_mutex_);
      if (!rclcpp::ok() || worker_exit_.load() || session != supervisor_->active_session()) {
        return ActionOutcome::SESSION_SUPERSEDED;
      }
      if (honor_pause_stop && !supervisor_->can_dispatch(session, true)) {
        return ActionOutcome::CANCELED;
      }
      if constexpr (std::is_same_v<ActionT, FollowJointTrajectory>) {
        active_rigid_dispatch_ = dispatch;
      } else {
        active_imp_dispatch_ = dispatch;
      }
      if (out_goal_dispatched) {
        *out_goal_dispatched = true;
      }
      gh_future = client->async_send_goal(goal, opts);
    }
    ScopeExit clear_guard{[this, dispatch]() {
      std::lock_guard<std::mutex> lock(dispatch_mutex_);
      if constexpr (std::is_same_v<ActionT, FollowJointTrajectory>) {
        if (active_rigid_dispatch_ == dispatch) {
          active_rigid_dispatch_.reset();
        }
      } else {
        if (active_imp_dispatch_ == dispatch) {
          active_imp_dispatch_.reset();
        }
      }
    }};

    while (gh_future.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) {
      if (!rclcpp::ok() || worker_exit_.load() || session != supervisor_->active_session()) {
        std::lock_guard<std::mutex> lock(dispatch->mutex);
        dispatch->cancelled = true;
        return ActionOutcome::SESSION_SUPERSEDED;
      }
      if (honor_pause_stop && supervisor_->stop_requested(session)) {
        std::lock_guard<std::mutex> lock(dispatch->mutex);
        dispatch->cancelled = true;
      }
      if (std::chrono::steady_clock::now() - start_tp > std::chrono::seconds(5)) {
        out_error = "Timed out waiting for controller goal response";
        return ActionOutcome::FAULT;
      }
    }

    auto goal_handle = gh_future.get();
    if (!goal_handle) {
      if (!rclcpp::ok() || worker_exit_.load() || session != supervisor_->active_session()) {
        return ActionOutcome::SESSION_SUPERSEDED;
      }
      if (honor_pause_stop && supervisor_->stop_requested(session)) {
        return ActionOutcome::CANCELED;
      }
      out_error = "Controller rejected trajectory goal";
      return ActionOutcome::FAULT;
    }

    {
      std::lock_guard<std::mutex> lock(dispatch->mutex);
      dispatch->handle = goal_handle;
    }

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
    {
      std::lock_guard<std::mutex> lock(feedback_gate->mutex);
      feedback_gate->completed = true;
    }
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
      if (st == StudioState::STATE_STOPPING) {
        if (ensure_normal_or_fault(session, -38)) {
          std::lock_guard<std::mutex> lock(exec_mutex_);
          reset_exec_cursor_unlocked();
          supervisor_->on_stop_completed(session);
        }
      } else if (st == StudioState::STATE_RECOVERING) {
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
            "Trajectory planner E-STOP recovery via CMD_CLEAR_ESTOP",
            ack,
            err,
            &ack_mode);
          sent_clear_cmd = true;
          last_clear_cmd_tp = now_tp;
          // Transient rejections are retried: SPEED (still moving), BUSY (trajectory
          // or stopping ramp finishing), RT_TIMEOUT (withdrawn), OUTCOME_UNKNOWN (reconcile
          // via observed controller mode before retrying).
          // STATE while the controller already reports NORMAL means an earlier
          // CLEAR_ESTOP took effect; the NORMAL observation advances the phase.
          const bool transient =
            ack == ControllerCommand::Response::ACK_REJECTED_SPEED ||
            ack == ControllerCommand::Response::ACK_REJECTED_BUSY ||
            ack == ControllerCommand::Response::ACK_RT_TIMEOUT ||
            ack == ControllerCommand::Response::ACK_OUTCOME_UNKNOWN ||
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
      if (snap.controller_fresh && snap.controller_mode == ControllerStatus::MODE_NORMAL) {
        return true;
      }
      if (snap.controller_mode == ControllerStatus::MODE_ESTOP) {
        return false;
      }
      if (snap.controller_fresh &&
          snap.controller_mode == ControllerStatus::MODE_CARTESIAN_IMPEDANCE) {
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
            "Trajectory planner exiting held Cartesian impedance to MODE_NORMAL",
            ack, msg, &ack_mode);
          last_send = now_tp;
          const bool transient =
            ok ||
            ack == ControllerCommand::Response::ACK_REJECTED_BUSY ||
            ack == ControllerCommand::Response::ACK_RT_TIMEOUT ||
            ack == ControllerCommand::Response::ACK_OUTCOME_UNKNOWN ||
            (ack == ControllerCommand::Response::ACK_REJECTED_STATE &&
             (ack_mode == ControllerStatus::MODE_NORMAL ||
              ack_mode == ControllerStatus::MODE_COMPLIANCE_STOPPING));
          if (!transient) {
            out_err = msg.empty() ?
              "Controller rejected CMD_ENTER_NORMAL while exiting impedance hold" : msg;
            return false;
          }
        }
      } else if (snap.controller_fresh &&
                 snap.controller_mode != ControllerStatus::MODE_COMPLIANCE_STOPPING &&
                 snap.controller_mode != ControllerStatus::MODE_CARTESIAN_IMPEDANCE_TRAJECTORY &&
                 snap.controller_mode != ControllerStatus::MODE_NORMAL) {
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
    std::shared_ptr<const ExecutionBlock> paused_blk;

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
    auto calc_total_pct = [cum_weight = std::move(cum_weight), total_blocks, total_weight](size_t idx, double block_frac) -> double {
      const double w0 = cum_weight[std::min(idx, total_blocks)];
      const double w1 = (idx < total_blocks) ? cum_weight[idx + 1] : total_weight;
      const double pct = 100.0 * (w0 + std::clamp(block_frac, 0.0, 1.0) * (w1 - w0)) / total_weight;
      return std::clamp(pct, 0.0, 100.0);
    };

    auto is_planning_cancelled = [this, session]() {
      return !rclcpp::ok() || worker_exit_.load(std::memory_order_acquire) ||
             !supervisor_->can_dispatch(session, true);
    };
    auto handle_pre_dispatch_interrupt = [&](const ExecutionBlock & ref_blk) -> bool {
      if (!rclcpp::ok() || worker_exit_.load(std::memory_order_acquire) ||
          session != supervisor_->active_session())
      {
        return true;
      }
      if (supervisor_->stop_requested(session)) {
        if (ensure_normal_or_fault(session, -38)) {
          std::lock_guard<std::mutex> lock(exec_mutex_);
          reset_exec_cursor_unlocked();
          supervisor_->on_stop_completed(session);
        }
        return true;
      }
      if (supervisor_->pause_requested(session)) {
        if (ensure_normal_or_fault(session, -38)) {
          std::lock_guard<std::mutex> lock(exec_mutex_);
          if (supervisor_->stop_requested(session)) {
            reset_exec_cursor_unlocked();
          } else {
            exec_block_idx_ = block_idx;
            paused_mid_block_ = resume_mid;
            paused_sample_idx_ = pause_sample;
            paused_remaining_wait_sec_ = resume_wait_sec;
            paused_block_snapshot_ = resume_mid ? paused_blk : nullptr;
          }
          supervisor_->on_pause_settled(
            session, ref_blk.loc.line, ref_blk.loc.statement_index,
            static_cast<int32_t>(block_idx), 0.0, calc_total_pct(block_idx, 0.0));
        }
        return true;
      }
      return false;
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
        const auto & ref_blk = resume_mid ? *paused_blk : prog->blocks[block_idx];
        std::lock_guard<std::mutex> lock(exec_mutex_);
        if (supervisor_->stop_requested(session)) {
          reset_exec_cursor_unlocked();
        } else {
          exec_block_idx_ = block_idx;
          paused_mid_block_ = resume_mid;
          paused_sample_idx_ = pause_sample;
          paused_remaining_wait_sec_ = resume_wait_sec;
          if (resume_mid) {
            paused_block_snapshot_ = paused_blk;
          }
        }
        supervisor_->on_pause_settled(
          session, ref_blk.loc.line, ref_blk.loc.statement_index,
          static_cast<int32_t>(block_idx), 0.0, calc_total_pct(block_idx, 0.0));
        return;
      }

      auto block_ptr = std::shared_ptr<const ExecutionBlock>(prog, &prog->blocks[block_idx]);
      bool used_mid_block_replan = false;
      if (resume_mid && paused_blk->type != ExecutionBlockType::WAIT_CMD) {
        std::array<double, 6> q_actual{};
        double max_dq = 0.0;
        if (!get_latest_joint_state(q_actual, max_dq)) {
          StudioSupervisor::Effects eff;
          supervisor_->on_execution_fault(
            session, -2, "Joint states unavailable during resume replanning", false, eff);
          apply_effects(eff);
          return;
        }
        auto replanned = std::make_shared<ExecutionBlock>();
        std::string replan_err;
        const bool replanned_ok = compiler_->replan_remaining_block(
          *paused_blk, pause_sample, q_actual, *replanned, replan_err, is_planning_cancelled);
        if (handle_pre_dispatch_interrupt(*paused_blk)) {
          return;
        }
        if (!replanned_ok) {
          StudioSupervisor::Effects eff;
          supervisor_->on_execution_fault(
            session, -3, "Mid-block resume replanning failed: " + replan_err, false, eff);
          apply_effects(eff);
          return;
        }
        block_ptr = std::move(replanned);
        used_mid_block_replan = true;
        resume_mid = false;
        pause_sample = 0;
      } else {
        if (resume_mid && block_ptr->type == ExecutionBlockType::WAIT_CMD && resume_wait_sec >= 0.0) {
          auto remaining = std::make_shared<ExecutionBlock>(*block_ptr);
          remaining->duration_sec = resume_wait_sec;
          remaining->wait_cmd.duration_sec = resume_wait_sec;
          block_ptr = std::move(remaining);
        }
        resume_mid = false;
        resume_wait_sec = -1.0;
      }

      // If a previous ImpedanceCartesian(..., hold=true) left the controller in
      // MODE_CARTESIAN_IMPEDANCE, return cleanly to MODE_NORMAL before dispatching
      // the next rigid or impedance trajectory (or alignment bridge).
      if (block_ptr->type == ExecutionBlockType::RIGID_TRAJECTORY ||
          block_ptr->type == ExecutionBlockType::IMPEDANCE_TRAJECTORY)
      {
        if (!ensure_normal_or_fault(session, -38)) {
          return;
        }
      }

      // Pre-execution joint alignment bridge or post-impedance start-pose replanning
      if (!used_mid_block_replan &&
          (block_ptr->type == ExecutionBlockType::RIGID_TRAJECTORY ||
           block_ptr->type == ExecutionBlockType::IMPEDANCE_TRAJECTORY))
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
          max_diff = std::max(max_diff, std::abs(q_actual[i] - block_ptr->start_joints[i]));
        }

        if (post_imp_offset) {
          if (max_diff > 5e-4 && block_ptr->source_kind != BlockSourceKind::NONE) {
            const auto orig_end_joints = block_ptr->end_joints;
            ExecutionBlock replanned_blk;
            std::vector<Diagnostic> diags;
            const bool replanned_ok = compiler_->replan_block_from_start(
              *block_ptr, q_actual, replanned_blk, diags, is_planning_cancelled);
            if (handle_pre_dispatch_interrupt(*block_ptr)) {
              return;
            }
            if (!replanned_ok) {
              std::string reason = "Unknown planning error";
              for (const auto & d : diags) {
                if (d.severity == Diagnostic::SEVERITY_ERROR) {
                  reason = d.message;
                  break;
                }
              }
              if (reason == "Unknown planning error" && !diags.empty()) {
                reason = diags.front().message;
              }
              StudioSupervisor::Effects eff;
              supervisor_->on_execution_fault(
                session, -5, "Post-impedance trajectory replanning failed: " + reason, false, eff);
              apply_effects(eff);
              return;
            }
            block_ptr = std::make_shared<ExecutionBlock>(std::move(replanned_blk));
            if (block_ptr->type == ExecutionBlockType::RIGID_TRAJECTORY) {
              double end_diff = 0.0;
              for (size_t i = 0; i < 6; ++i) {
                end_diff = std::max(
                  end_diff, std::abs(block_ptr->end_joints[i] - orig_end_joints[i]));
              }
              if (end_diff <= 1e-3) {
                post_imp_offset = false;
              }
            }
          } else if (block_ptr->type == ExecutionBlockType::RIGID_TRAJECTORY && max_diff <= 5e-4) {
            post_imp_offset = false;
          }
        } else if (max_diff > kAlignmentThresholdRad) {
          trajectory_msgs::msg::JointTrajectory bridge_traj;
          std::string bridge_err;
          const bool bridge_ok = compiler_->plan_alignment_bridge(
            q_actual, block_ptr->start_joints, 0.35, 0.35, bridge_traj, bridge_err,
            is_planning_cancelled);
          if (handle_pre_dispatch_interrupt(*block_ptr)) {
            return;
          }
          if (!bridge_ok) {
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
            session, block_ptr->loc.line, block_ptr->loc.statement_index,
            static_cast<int32_t>(block_idx), 0.0, calc_total_pct(block_idx, 0.0));

          const double bridge_dur =
            bridge_traj.points.back().time_from_start.sec +
            1e-9 * bridge_traj.points.back().time_from_start.nanosec;
          std::string exec_err;
          const auto outcome = run_controller_action<FollowJointTrajectory>(
            rigid_client_, bridge_goal, session, bridge_dur + 12.0,
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
            if (supervisor_->stop_requested(session)) {
              reset_exec_cursor_unlocked();
            } else {
              exec_block_idx_ = block_idx;
              paused_mid_block_ = false;
              paused_sample_idx_ = 0;
              paused_block_snapshot_.reset();
            }
            supervisor_->on_pause_settled(
              session, block_ptr->loc.line, block_ptr->loc.statement_index,
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
      if (block_ptr->type == ExecutionBlockType::RIGID_TRAJECTORY) {
        if (!execute_rigid_block(session, block_ptr, block_idx, calc_total_pct)) {
          return;
        }
      } else if (block_ptr->type == ExecutionBlockType::IMPEDANCE_TRAJECTORY) {
        if (!execute_impedance_block(session, block_ptr, block_idx, calc_total_pct)) {
          return;
        }
        post_imp_offset = true;
      } else if (block_ptr->type == ExecutionBlockType::GRIPPER_CMD) {
        if (!execute_gripper_block(session, *block_ptr, block_idx, calc_total_pct)) {
          return;
        }
      } else if (block_ptr->type == ExecutionBlockType::WAIT_CMD) {
        if (!execute_wait_block(session, block_ptr, block_idx, calc_total_pct)) {
          return;
        }
      } else if (block_ptr->type == ExecutionBlockType::PAYLOAD_CMD) {
        if (!execute_payload_block(session, *block_ptr, block_idx, calc_total_pct)) {
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
      if (session != supervisor_->active_session() ||
          supervisor_->stop_requested(session) ||
          supervisor_->pause_requested(session))
      {
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

  struct RigidFeedbackTiming
  {
    std::mutex mutex;
    bool valid = false;
    double traj_time_sec = 0.0;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  struct PauseSpliceState
  {
    std::optional<size_t> confirmed_end_sample;
    std::optional<size_t> unconfirmed_end_sample;
    std::optional<rclcpp::Client<SpliceJointTrajectory>::FutureAndRequestId>
      unconfirmed_response_future;
  };

  bool execute_rigid_block(
    uint64_t session,
    const std::shared_ptr<const ExecutionBlock> & block,
    size_t block_idx,
    const std::function<double(size_t, double)> & calc_total_pct)
  {
    const auto & blk = *block;
    const size_t total_pts = blk.rigid_trajectory.points.size();
    std::atomic<size_t> last_sample{0};
    RigidFeedbackTiming fb_timing;

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
      {
        std::lock_guard<std::mutex> lock(fb_timing.mutex);
        fb_timing.traj_time_sec = t;
        fb_timing.stamp = rclcpp::Time(fb.header.stamp, RCL_ROS_TIME);
        fb_timing.valid = true;
      }
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
    PauseSpliceState splice_state;
    ScopeExit cleanup_splice_req{[this, &splice_state]() {
      if (splice_state.unconfirmed_response_future.has_value()) {
        splice_client_->remove_pending_request(*splice_state.unconfirmed_response_future);
        splice_state.unconfirmed_response_future.reset();
      }
    }};
    auto on_pause = [&](const rclcpp_action::GoalUUID & goal_id) {
      return request_pause_splice(session, blk, goal_id, fb_timing, splice_state);
    };
    std::string err;
    bool goal_dispatched = false;
    const auto outcome = run_controller_action<FollowJointTrajectory>(
      rigid_client_, goal, session, blk.duration_sec + 12.0, fb_cb, true, true, err,
      on_pause, &goal_dispatched);
    if (outcome == ActionOutcome::SUCCEEDED) {
      if (splice_state.confirmed_end_sample.has_value()) {
        // The goal settled at the end of the stop tail: a pause in the middle of the block.
        return handle_trajectory_outcome(
          session, block, block_idx, *splice_state.confirmed_end_sample, total_pts,
          ActionOutcome::CANCELED, err, calc_total_pct, true);
      }
      if (splice_state.unconfirmed_end_sample.has_value()) {
        // Resolve splice timeout / unknown-outcome race: if the splice was actually adopted by
        // CTC RT, the parent goal settled at the stop tail rather than the programmed block end.
        // Never silently skip the rest of the block.
        bool adopted = false;
        if (splice_state.unconfirmed_response_future.has_value()) {
          if (splice_state.unconfirmed_response_future->future.wait_for(
                std::chrono::milliseconds(100)) == std::future_status::ready)
          {
            const auto resp = splice_state.unconfirmed_response_future->future.get();
            adopted = resp && resp->adopted;
          } else {
            splice_client_->remove_pending_request(*splice_state.unconfirmed_response_future);
          }
          splice_state.unconfirmed_response_future.reset();
        }
        if (!adopted) {
          std::array<double, 6> q_actual{};
          double max_dq = 0.0;
          if (get_latest_joint_state(q_actual, max_dq)) {
            double end_err = 0.0;
            for (size_t i = 0; i < 6; ++i) {
              end_err = std::max(end_err, std::abs(q_actual[i] - blk.end_joints[i]));
            }
            if (end_err > 0.01) {
              adopted = true;
            }
          }
        }
        if (adopted) {
          RCLCPP_WARN(
            get_logger(),
            "Resolved late pause-splice adoption after parent completion; pausing mid-block");
          return handle_trajectory_outcome(
            session, block, block_idx, *splice_state.unconfirmed_end_sample, total_pts,
            ActionOutcome::CANCELED, err, calc_total_pct, true);
        }
      }
    }
    return handle_trajectory_outcome(
      session, block, block_idx, last_sample.load(), total_pts, outcome, err, calc_total_pct,
      goal_dispatched);
  }

  // Plans a stop-along-path tail for the running rigid goal and submits it through the
  // dedicated /ctc_controller/splice_joint_trajectory service with explicit parent_goal_id
  // and switch_time. Falls back to CANCEL whenever the splice cannot be confirmed.
  PauseHandling request_pause_splice(
    uint64_t session,
    const ExecutionBlock & blk,
    const rclcpp_action::GoalUUID & goal_id,
    RigidFeedbackTiming & fb_timing,
    PauseSpliceState & out_splice)
  {
    if (!splice_client_ || !splice_client_->service_is_ready()) {
      return PauseHandling::CANCEL;
    }
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
    auto block_time_now = [&]() {
      const rclcpp::Time t_now = now();
      {
        // Anchor block elapsed time to the latest 50 Hz controller feedback (which comes
        // directly from the RT loop's accumulated execution->elapsed) to avoid RTF/clock drift.
        std::lock_guard<std::mutex> lock(fb_timing.mutex);
        if (fb_timing.valid && fb_timing.stamp.nanoseconds() > 0) {
          const double age = (t_now - fb_timing.stamp).seconds();
          if (age >= -0.05 && age <= 0.25) {
            return fb_timing.traj_time_sec + std::max(0.0, age);
          }
        }
      }
      return (t_now - start).seconds();
    };

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
    auto req = std::make_shared<SpliceJointTrajectory::Request>();
    req->parent_goal_id = goal_id;
    req->expected_controller_epoch = supervisor_->snapshot().controller_epoch;
    req->switch_time = tail.points.front().time_from_start;
    tail.header.stamp.sec = 0;
    tail.header.stamp.nanosec = 0;
    tail.joint_names = kJointNames;
    req->trajectory = std::move(tail);

    std::optional<rclcpp::Client<SpliceJointTrajectory>::FutureAndRequestId> splice_future;
    {
      // Guarded dispatch: STOP / E-STOP / session supersede prevent issuing a pause splice.
      std::lock_guard<std::mutex> lock(dispatch_mutex_);
      if (!rclcpp::ok() || worker_exit_.load(std::memory_order_acquire) ||
          !supervisor_->can_dispatch(session, /*honor_pause=*/false))
      {
        return PauseHandling::CANCEL;
      }
      // Record unconfirmed_end_sample before dispatching so a timeout race cannot silently skip
      // the remainder of the block if CTC RT adopts the splice before parent cancellation lands.
      out_splice.unconfirmed_end_sample = end_sample;
      splice_future.emplace(splice_client_->async_send_request(req));
    }
    auto time_left = [&]() {
      return std::chrono::duration<double>(std::max(0.0, t_switch - block_time_now()));
    };
    if (splice_future->future.wait_for(time_left() + std::chrono::milliseconds(300)) !=
        std::future_status::ready)
    {
      out_splice.unconfirmed_response_future = std::move(splice_future);
      return PauseHandling::CANCEL;
    }
    const auto resp = splice_future->future.get();
    if (!resp) {
      out_splice.unconfirmed_end_sample.reset();
      RCLCPP_WARN(get_logger(), "Null response from /ctc_controller/splice_joint_trajectory");
      return PauseHandling::CANCEL;
    }
    if (resp->adopted) {
      out_splice.confirmed_end_sample = end_sample;
      out_splice.unconfirmed_end_sample.reset();
      return PauseHandling::SPLICED;
    }
    if (resp->result_code == SpliceJointTrajectory::Response::RESULT_OUTCOME_UNKNOWN) {
      // Outcome uncertain: keep unconfirmed_end_sample so parent completion reconciles end pose.
      RCLCPP_WARN(
        get_logger(),
        "Pause splice outcome unknown (%s); canceling parent goal and reconciling on completion",
        resp->message.c_str());
      return PauseHandling::CANCEL;
    }
    // RESULT_RT_TIMEOUT and RESULT_REJECTED_* guarantee the splice was NOT adopted.
    out_splice.unconfirmed_end_sample.reset();
    RCLCPP_WARN(
      get_logger(),
      "Pause splice was not adopted (code=%u: %s); canceling instead",
      static_cast<unsigned>(resp->result_code), resp->message.c_str());
    return PauseHandling::CANCEL;
  }

  bool execute_impedance_block(
    uint64_t session,
    const std::shared_ptr<const ExecutionBlock> & block,
    size_t block_idx,
    const std::function<double(size_t, double)> & calc_total_pct)
  {
    const auto & blk = *block;
    // Confirm settling below the planner bound so the CTC entry speed guard passes cleanly.
    const auto settle = wait_for_planner_settle(session, 2.0);
    if (settle == SettleOutcome::ABORTED) {
      return handle_trajectory_outcome(
        session, block, block_idx, 0, blk.impedance_goal.points.size(),
        session != supervisor_->active_session() ?
        ActionOutcome::SESSION_SUPERSEDED : ActionOutcome::CANCELED,
        "", calc_total_pct, false);
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
    bool goal_dispatched = false;
    const auto outcome = run_controller_action<FollowImpedanceTrajectory>(
      imp_client_, imp_goal, session, blk.duration_sec + 17.0, fb_cb, false, true, err,
      nullptr, &goal_dispatched);
    supervisor_->clear_impedance_errors();
    return handle_trajectory_outcome(
      session, block, block_idx, last_sample.load(), total_pts, outcome, err, calc_total_pct,
      goal_dispatched);
  }

  bool handle_trajectory_outcome(
    uint64_t session,
    const std::shared_ptr<const ExecutionBlock> & block,
    size_t block_idx,
    size_t sample_idx,
    size_t total_pts,
    ActionOutcome outcome,
    const std::string & err,
    const std::function<double(size_t, double)> & calc_total_pct,
    bool mid_block = true)
  {
    const auto & blk = *block;
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
      const size_t safe_idx = (mid_block && total_pts > 0) ?
        std::min(sample_idx, total_pts - 1) : 0;
      const double frac = (mid_block && total_pts > 1) ?
        static_cast<double>(safe_idx) / static_cast<double>(total_pts - 1) : 0.0;
      const auto & loc = (mid_block && safe_idx < blk.sample_locations.size()) ?
        blk.sample_locations[safe_idx] : blk.loc;
      {
        std::lock_guard<std::mutex> lock(exec_mutex_);
        if (supervisor_->stop_requested(session)) {
          reset_exec_cursor_unlocked();
        } else {
          exec_block_idx_ = block_idx;
          paused_mid_block_ = mid_block;
          paused_sample_idx_ = safe_idx;
          paused_block_snapshot_ = mid_block ? block : nullptr;
          if (mid_block && blk.type == ExecutionBlockType::IMPEDANCE_TRAJECTORY) {
            post_impedance_offset_active_ = true;
          }
        }
        supervisor_->on_pause_settled(
          session, loc.line, loc.statement_index, static_cast<int32_t>(block_idx),
          100.0 * frac, calc_total_pct(block_idx, frac));
      }
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
    gripper_active_timeout_sec_.store(
      std::max(0.5, blk.gripper_cmd.timeout_sec), std::memory_order_release);
    gripper_block_active_.store(true, std::memory_order_release);
    ScopeExit clear_gripper_flag{[this]() {
      gripper_block_active_.store(false, std::memory_order_release);
      gripper_active_timeout_sec_.store(0.0, std::memory_order_release);
    }};
    supervisor_->update_execution_progress(
      session, blk.loc.line, blk.loc.statement_index,
      static_cast<int32_t>(block_idx), 10.0, calc_total_pct(block_idx, 0.1));

    const auto wait_srv_start = std::chrono::steady_clock::now();
    while (!gripper_client_->service_is_ready()) {
      if (!rclcpp::ok() || worker_exit_.load(std::memory_order_acquire) ||
          session != supervisor_->active_session())
      {
        return false;
      }
      if (supervisor_->stop_requested(session)) {
        if (ensure_normal_or_fault(session, -38)) {
          std::lock_guard<std::mutex> lock(exec_mutex_);
          reset_exec_cursor_unlocked();
          supervisor_->on_stop_completed(session);
        }
        return false;
      }
      if (std::chrono::steady_clock::now() - wait_srv_start > std::chrono::seconds(2)) {
        StudioSupervisor::Effects eff;
        supervisor_->on_execution_fault(
          session, -30, "Gripper service /gripper/command is unavailable", false, eff);
        apply_effects(eff);
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    auto req = std::make_shared<GripperCommand::Request>();
    req->command = static_cast<uint8_t>(blk.gripper_cmd.action);
    std::optional<rclcpp::Client<GripperCommand>::FutureAndRequestId> future;
    {
      // Guarded dispatch: revalidate session/STOP under dispatch_mutex_ so a superseded
      // worker can never issue OPEN/CLOSE after an E-STOP or STOP gripper halt.
      std::lock_guard<std::mutex> lock(dispatch_mutex_);
      if (!rclcpp::ok() || worker_exit_.load(std::memory_order_acquire) ||
          session != supervisor_->active_session())
      {
        return false;
      }
      if (!supervisor_->stop_requested(session)) {
        future.emplace(gripper_client_->async_send_request(req));
      }
    }
    if (!future.has_value()) {
      if (ensure_normal_or_fault(session, -38)) {
        std::lock_guard<std::mutex> lock(exec_mutex_);
        reset_exec_cursor_unlocked();
        supervisor_->on_stop_completed(session);
      }
      return false;
    }
    const auto req_start = std::chrono::steady_clock::now();
    while (future->future.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) {
      if (session != supervisor_->active_session() || !rclcpp::ok() ||
          worker_exit_.load(std::memory_order_acquire))
      {
        gripper_client_->remove_pending_request(*future);
        return false;
      }
      if (supervisor_->stop_requested(session)) {
        gripper_client_->remove_pending_request(*future);
        if (blk.gripper_cmd.action != GripperActionType::STOP) {
          send_gripper_stop(true);
        }
        if (!ensure_normal_or_fault(session, -38)) {
          return false;
        }
        std::lock_guard<std::mutex> lock(exec_mutex_);
        reset_exec_cursor_unlocked();
        supervisor_->on_stop_completed(session);
        return false;
      }
      if (std::chrono::steady_clock::now() - req_start > std::chrono::seconds(3)) {
        gripper_client_->remove_pending_request(*future);
        StudioSupervisor::Effects eff;
        supervisor_->on_execution_fault(
          session, -31, "Timed out waiting for /gripper/command service response", false, eff);
        apply_effects(eff);
        return false;
      }
    }

    const auto resp = future->future.get();
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
        if (blk.gripper_cmd.action != GripperActionType::STOP) {
          send_gripper_stop(true);
        }
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
        if (gs.applied_sequence != target_seq || gs.last_command != req->command) {
          StudioSupervisor::Effects eff;
          supervisor_->on_execution_fault(
            session, -33, "Gripper command was superseded before completion", false, eff);
          apply_effects(eff);
          return false;
        }
        if (gs.last_cmd_result == GripperState::CMD_RES_FAULT ||
            gs.last_cmd_result == GripperState::CMD_RES_BLOCKED ||
            (gs.last_cmd_result == GripperState::CMD_RES_STOPPED &&
             blk.gripper_cmd.action != GripperActionType::STOP))
        {
          StudioSupervisor::Effects eff;
          supervisor_->on_execution_fault(
            session, -33, "Gripper command failed: " + gs.detail, false, eff);
          apply_effects(eff);
          return false;
        }
        const bool done = gs.last_cmd_result == GripperState::CMD_RES_SUCCEEDED ||
          (blk.gripper_cmd.action == GripperActionType::STOP &&
           gs.last_cmd_result == GripperState::CMD_RES_STOPPED);
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
    const std::shared_ptr<const ExecutionBlock> & block,
    size_t block_idx,
    const std::function<double(size_t, double)> & calc_total_pct)
  {
    const auto & blk = *block;
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
          if (supervisor_->stop_requested(session)) {
            reset_exec_cursor_unlocked();
          } else {
            exec_block_idx_ = block_idx;
            paused_mid_block_ = true;
            paused_remaining_wait_sec_ = remaining;
            paused_block_snapshot_ = block;
          }
          supervisor_->on_pause_settled(
            session, blk.loc.line, blk.loc.statement_index,
            static_cast<int32_t>(block_idx), 100.0 * frac, calc_total_pct(block_idx, frac));
        }
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
      if (session == supervisor_->active_session()) {
        if (supervisor_->stop_requested(session)) {
          if (ensure_normal_or_fault(session, -38)) {
            std::lock_guard<std::mutex> lock(exec_mutex_);
            reset_exec_cursor_unlocked();
            supervisor_->on_stop_completed(session);
          }
        } else if (supervisor_->pause_requested(session)) {
          if (ensure_normal_or_fault(session, -38)) {
            std::lock_guard<std::mutex> lock(exec_mutex_);
            exec_block_idx_ = block_idx;
            paused_mid_block_ = false;
            paused_sample_idx_ = 0;
            paused_block_snapshot_.reset();
            supervisor_->on_pause_settled(
              session, blk.loc.line, blk.loc.statement_index,
              static_cast<int32_t>(block_idx), 0.0, calc_total_pct(block_idx, 0.0));
          }
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

    const auto wait_srv_start = std::chrono::steady_clock::now();
    while (!ctc_cmd_client_->service_is_ready()) {
      if (!rclcpp::ok() || worker_exit_.load(std::memory_order_acquire) ||
          session != supervisor_->active_session())
      {
        return false;
      }
      if (supervisor_->stop_requested(session)) {
        if (ensure_normal_or_fault(session, -38)) {
          std::lock_guard<std::mutex> lock(exec_mutex_);
          reset_exec_cursor_unlocked();
          supervisor_->on_stop_completed(session);
        }
        return false;
      }
      if (supervisor_->pause_requested(session)) {
        if (ensure_normal_or_fault(session, -38)) {
          std::lock_guard<std::mutex> lock(exec_mutex_);
          exec_block_idx_ = block_idx;
          paused_mid_block_ = false;
          paused_sample_idx_ = 0;
          paused_block_snapshot_.reset();
          supervisor_->on_pause_settled(
            session, blk.loc.line, blk.loc.statement_index,
            static_cast<int32_t>(block_idx), 0.0, calc_total_pct(block_idx, 0.0));
        }
        return false;
      }
      if (std::chrono::steady_clock::now() - wait_srv_start > std::chrono::milliseconds(250)) {
        StudioSupervisor::Effects eff;
        supervisor_->on_execution_fault(
          session, -35, "/ctc_controller/command service unavailable for SetPayload", false, eff);
        apply_effects(eff);
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    const auto pre_snap = supervisor_->snapshot();
    const uint64_t pre_cmd_seq = pre_snap.last_cmd_seq;
    auto req = std::make_shared<ControllerCommand::Request>();
    req->command = ControllerCommand::Request::CMD_SET_PAYLOAD;
    req->expected_epoch = pre_snap.controller_epoch;
    req->payload_mass = blk.payload_cmd.mass_kg;
    req->payload_com_tcp = blk.payload_cmd.com_tcp;
    req->reason = "Trajectory DSL SetPayload";

    std::optional<rclcpp::Client<ControllerCommand>::FutureAndRequestId> future;
    {
      // Guarded dispatch: do not issue CMD_SET_PAYLOAD after STOP/PAUSE/session supersede.
      std::lock_guard<std::mutex> lock(dispatch_mutex_);
      if (!rclcpp::ok() || worker_exit_.load(std::memory_order_acquire) ||
          session != supervisor_->active_session())
      {
        return false;
      }
      if (supervisor_->can_dispatch(session, /*honor_pause=*/true)) {
        future.emplace(ctc_cmd_client_->async_send_request(req));
      }
    }
    if (!future.has_value()) {
      if (supervisor_->stop_requested(session)) {
        if (ensure_normal_or_fault(session, -38)) {
          std::lock_guard<std::mutex> lock(exec_mutex_);
          reset_exec_cursor_unlocked();
          supervisor_->on_stop_completed(session);
        }
      } else if (supervisor_->pause_requested(session)) {
        if (ensure_normal_or_fault(session, -38)) {
          std::lock_guard<std::mutex> lock(exec_mutex_);
          exec_block_idx_ = block_idx;
          paused_mid_block_ = false;
          paused_sample_idx_ = 0;
          paused_block_snapshot_.reset();
          supervisor_->on_pause_settled(
            session, blk.loc.line, blk.loc.statement_index,
            static_cast<int32_t>(block_idx), 0.0, calc_total_pct(block_idx, 0.0));
        }
      }
      return false;
    }

    auto reconcile_payload_unknown = [&](uint64_t expected_cmd_seq, std::string & detail) -> bool {
      const auto reconcile_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
      while (rclcpp::ok() && !worker_exit_.load(std::memory_order_acquire)) {
        if (session != supervisor_->active_session()) {
          return false;
        }
        const auto snap = supervisor_->snapshot();
        if (snap.controller_fresh && snap.controller_epoch == pre_snap.controller_epoch) {
          const bool payload_matches =
            std::abs(snap.payload_mass - blk.payload_cmd.mass_kg) <= 1e-6 &&
            std::abs(snap.payload_com_tcp[0] - blk.payload_cmd.com_tcp[0]) <= 1e-6 &&
            std::abs(snap.payload_com_tcp[1] - blk.payload_cmd.com_tcp[1]) <= 1e-6 &&
            std::abs(snap.payload_com_tcp[2] - blk.payload_cmd.com_tcp[2]) <= 1e-6;
          if (expected_cmd_seq > 0) {
            if (snap.last_cmd_seq == expected_cmd_seq &&
                snap.last_cmd_code == ControllerCommand::Request::CMD_SET_PAYLOAD)
            {
              if (snap.last_cmd_result == ControllerStatus::ACK_APPLIED) {
                return true;
              }
              detail = "ControllerStatus confirmed SetPayload rejection (ack=" +
                std::to_string(snap.last_cmd_result) + ")";
              return false;
            }
            if (snap.last_cmd_seq > expected_cmd_seq && payload_matches) {
              return true;
            }
          } else if (snap.last_cmd_seq > pre_cmd_seq) {
            if (snap.last_cmd_code == ControllerCommand::Request::CMD_SET_PAYLOAD) {
              if (snap.last_cmd_result == ControllerStatus::ACK_APPLIED) {
                return true;
              }
              detail = "ControllerStatus confirmed SetPayload rejection (ack=" +
                std::to_string(snap.last_cmd_result) + ")";
              return false;
            }
            if (payload_matches) {
              return true;
            }
          }
        }
        if (std::chrono::steady_clock::now() > reconcile_deadline) {
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      detail = "SetPayload outcome unknown and not confirmed by ControllerStatus within 500 ms";
      return false;
    };

    if (future->future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
      ctc_cmd_client_->remove_pending_request(*future);
      std::string reconcile_err;
      if (reconcile_payload_unknown(0, reconcile_err)) {
        RCLCPP_WARN(
          get_logger(),
          "Reconciled SetPayload RPC timeout via ControllerStatus telemetry");
        supervisor_->update_execution_progress(
          session, blk.loc.line, blk.loc.statement_index,
          static_cast<int32_t>(block_idx), 100.0, calc_total_pct(block_idx, 1.0));
        return true;
      }
      if (session != supervisor_->active_session()) {
        return false;
      }
      StudioSupervisor::Effects eff;
      supervisor_->on_execution_fault(
        session, -36, reconcile_err, false, eff);
      apply_effects(eff);
      return false;
    }

    const auto resp = future->future.get();
    if (!resp) {
      StudioSupervisor::Effects eff;
      supervisor_->on_execution_fault(
        session, -37, "Null SetPayload response", false, eff);
      apply_effects(eff);
      return false;
    }
    if (!resp->accepted) {
      if (resp->ack_result == ControllerCommand::Response::ACK_OUTCOME_UNKNOWN) {
        std::string reconcile_err;
        if (reconcile_payload_unknown(resp->cmd_seq, reconcile_err)) {
          RCLCPP_WARN(
            get_logger(),
            "Reconciled SetPayload ACK_OUTCOME_UNKNOWN via ControllerStatus telemetry");
          supervisor_->update_execution_progress(
            session, blk.loc.line, blk.loc.statement_index,
            static_cast<int32_t>(block_idx), 100.0, calc_total_pct(block_idx, 1.0));
          return true;
        }
        if (session != supervisor_->active_session()) {
          return false;
        }
        StudioSupervisor::Effects eff;
        supervisor_->on_execution_fault(session, -36, reconcile_err, false, eff);
        apply_effects(eff);
        return false;
      }
      StudioSupervisor::Effects eff;
      supervisor_->on_execution_fault(
        session, -37, "Controller rejected SetPayload: " + resp->message, false, eff);
      apply_effects(eff);
      return false;
    }

    supervisor_->update_execution_progress(
      session, blk.loc.line, blk.loc.statement_index,
      static_cast<int32_t>(block_idx), 100.0, calc_total_pct(block_idx, 1.0));
    return true;
  }

  lite6_kinematics::KinematicsEngine kinematics_;
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
  rclcpp::Client<SpliceJointTrajectory>::SharedPtr splice_client_;
  rclcpp::Client<GripperCommand>::SharedPtr gripper_client_;
  rclcpp::Service<CompileProgram>::SharedPtr compile_srv_;
  rclcpp::Service<StudioCommand>::SharedPtr command_srv_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::mutex pub_mutex_;
  uint64_t last_published_revision_ = 0;
  std::mutex command_mutex_;
  std::mutex compile_mutex_;
  std::atomic<bool> gripper_block_active_{false};
  std::atomic<double> gripper_active_timeout_sec_{0.0};

  mutable std::mutex state_data_mutex_;
  std::array<double, 6> latest_q_{};
  std::array<double, 6> latest_dq_{};
  int64_t latest_joint_steady_ns_ = 0;
  bool has_latest_joint_ = false;
  GripperState latest_gripper_;
  bool has_latest_gripper_ = false;

  // Serializes guarded worker dispatches (action goals, splice, gripper, payload) against
  // STOP/PAUSE/ESTOP command acceptance and cancellation registration.
  std::mutex dispatch_mutex_;
  std::shared_ptr<InFlightAction<FollowJointTrajectory>> active_rigid_dispatch_;
  std::shared_ptr<InFlightAction<FollowImpedanceTrajectory>> active_imp_dispatch_;

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
  std::shared_ptr<const ExecutionBlock> paused_block_snapshot_;
  bool post_impedance_offset_active_ = false;

  std::mutex worker_wake_mutex_;
  std::condition_variable worker_cv_;
  bool worker_wake_pending_ = false;
  std::atomic<bool> worker_exit_{false};
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
