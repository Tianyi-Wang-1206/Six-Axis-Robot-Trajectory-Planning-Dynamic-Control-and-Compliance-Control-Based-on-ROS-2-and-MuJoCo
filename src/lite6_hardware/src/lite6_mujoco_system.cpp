#include "lite6_hardware/lite6_mujoco_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "lite6_interfaces/motion_contract.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <pthread.h>
#include <sched.h>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/resource.h>

namespace lite6_hardware
{
namespace
{
[[gnu::noinline]] void prefault_physics_stack()
{
  constexpr size_t kStackPrefaultBytes = 64 * 1024;
  volatile unsigned char stack_pages[kStackPrefaultBytes];
  for (size_t i = 0; i < kStackPrefaultBytes; i += 4096) {
    stack_pages[i] = 0;
  }
  const volatile unsigned char touch = stack_pages[0];
  (void)touch;
}
}  // namespace

Lite6MujocoSystem::~Lite6MujocoSystem()
{
  if (diagnostics_timer_) {diagnostics_timer_->cancel();}
  stop_physics();
  release_model();
}

int64_t Lite6MujocoSystem::steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    Clock::now().time_since_epoch()).count();
}

void Lite6MujocoSystem::stop_physics()
{
  gripper_.deactivate();
  stop_requested_.store(true);
  if (physics_thread_.joinable()) {physics_thread_.join();}
  diagnostics_->active.store(false);
  if (state_shm_ && state_shm_->valid()) {
    lite6_interfaces::shm::VisualStatePod pod{};
    uint64_t seq = 0;
    if (state_shm_->get()->visual_state.read(pod, seq)) {
      pod.active = 0;
      pod.published_steady_ns = steady_now_ns();
      state_shm_->get()->visual_state.write(pod);
    }
    state_shm_->get()->visual_alive_ns.store(steady_now_ns(), std::memory_order_release);
  }
}

void Lite6MujocoSystem::release_model()
{
  if (m_data) {mj_deleteData(m_data); m_data = nullptr;}
  if (m_model) {mj_deleteModel(m_model); m_model = nullptr;}
}

hardware_interface::CallbackReturn Lite6MujocoSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
#pragma GCC diagnostic pop

  stop_physics();
  release_model();
  commands_.reset();
  qpos_indices_.clear();
  qvel_indices_.clear();
  ctrl_indices_.clear();

  if (info_.joints.size() != kJoints + 2) {
    RCLCPP_ERROR(rclcpp::get_logger("Lite6MujocoSystem"), "Expected six arm joints and two state-only gripper joints");
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    const auto priority = info_.hardware_parameters.find("physics_thread_priority");
    if (priority != info_.hardware_parameters.end()) {
      size_t consumed = 0;
      physics_thread_priority_ = std::stoi(priority->second, &consumed);
      if (consumed != priority->second.size()) {throw std::invalid_argument("priority");}
    }
    if (physics_thread_priority_ < 0 || physics_thread_priority_ > 99) {
      throw std::invalid_argument("priority");
    }
  } catch (const std::exception &) {
    RCLCPP_ERROR(rclcpp::get_logger("Lite6MujocoSystem"),
      "physics_thread_priority must be an integer in [0, 99] (0 selects SCHED_OTHER)");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 1. Initialize hardware interface state and command vectors
  size_t num_joints = kJoints;
  hw_commands_effort_.resize(num_joints, 0.0);
  hw_states_position_.resize(num_joints, 0.0);
  hw_states_velocity_.resize(num_joints, 0.0);
  hw_states_effort_.resize(num_joints, 0.0);

  // 2. Find and load the MuJoCo model
  std::string pkg_path;
  try {
    pkg_path = ament_index_cpp::get_package_share_directory("lite6_description");
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("Lite6MujocoSystem"), "Cannot find lite6_description package!");
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  std::string xml_path = pkg_path + "/mujoco/scene.xml";
  char error_msg[1000] = "Could not load binary model";
  
  m_model = mj_loadXML(xml_path.c_str(), 0, error_msg, 1000);
  if (!m_model) {
    RCLCPP_FATAL(rclcpp::get_logger("Lite6MujocoSystem"), "Load model error: %s", error_msg);
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  m_data = mj_makeData(m_model);
  if (!m_data || !std::isfinite(m_model->opt.timestep) ||
    std::abs(m_model->opt.timestep - 0.0001) > 1e-12)
  {
    RCLCPP_FATAL(rclcpp::get_logger("Lite6MujocoSystem"),
      "MuJoCo data allocation failed or model timestep is not the required 0.0001 s");
    release_model();
    return hardware_interface::CallbackReturn::ERROR;
  }

  const std::array<const char *, 2> object_joints{
    "grasp_cylinder_free", "grasp_box_free"};
  for (size_t i = 0; i < object_joints.size(); ++i) {
    const int joint = mj_name2id(m_model, mjOBJ_JOINT, object_joints[i]);
    if (joint < 0 || m_model->jnt_type[joint] != mjJNT_FREE) {
      RCLCPP_FATAL(rclcpp::get_logger("Lite6MujocoSystem"),
        "Missing scene free joint: %s", object_joints[i]);
      release_model();
      return hardware_interface::CallbackReturn::ERROR;
    }
    object_qpos_indices_[i] = m_model->jnt_qposadr[joint];
  }

  // 3. Mapping joints and motors
  friction_models_.clear();
  friction_models_.reserve(num_joints);
  RCLCPP_INFO(rclcpp::get_logger("Lite6MujocoSystem"), "Mapping joints and motors...");
  for (size_t index = 0; index < kJoints; ++index) {
    const auto & joint = info_.joints[index];
    const auto friction = std::find_if(
      kLite6LuGreParameters.begin(), kLite6LuGreParameters.end(),
      [&joint](const JointFrictionParameters & entry) {return joint.name == entry.name;});
    if (friction == kLite6LuGreParameters.end()) {
      RCLCPP_FATAL(rclcpp::get_logger("Lite6MujocoSystem"),
        "No LuGre friction parameters for joint '%s'!", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    friction_models_.emplace_back(friction->parameters);

    // 3.1 Find the joints (joint1 ~ joint6)
    int joint_id = mj_name2id(m_model, mjOBJ_JOINT, joint.name.c_str());
    if (joint_id == -1) {
      RCLCPP_FATAL(rclcpp::get_logger("Lite6MujocoSystem"), "Joint '%s' not found in MuJoCo XML!", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    qpos_indices_.push_back(m_model->jnt_qposadr[joint_id]); // Position data offset in qpos array
    qvel_indices_.push_back(m_model->jnt_dofadr[joint_id]);  // Velocity/force data offset in qvel/qfrc arrays

    // 3.2 Find the corresponding actuators (motor1 ~ motor6)
    std::string motor_name = joint.name;
    size_t pos = motor_name.find("joint");
    if (pos != std::string::npos) {
        motor_name.replace(pos, 5, "motor");
    }

    int act_id = mj_name2id(m_model, mjOBJ_ACTUATOR, motor_name.c_str());
    if (act_id == -1) {
      RCLCPP_FATAL(rclcpp::get_logger("Lite6MujocoSystem"), "Motor '%s' not found in MuJoCo XML!", motor_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    ctrl_indices_.push_back(act_id);
    
    RCLCPP_INFO(rclcpp::get_logger("Lite6MujocoSystem"), 
      "Mapped URDF %s -> XML Joint ID: %d, XML Motor ID: %d", 
      joint.name.c_str(), joint_id, act_id);
  }

  for (size_t i = 0; i < kJoints; ++i) {
    perturb_bodies_[i] = mj_name2id(m_model, mjOBJ_BODY, ("link" + std::to_string(i + 1)).c_str());
  }

  try {
    gripper_.initialize(m_model, m_data);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("Lite6MujocoSystem"), "Gripper initialization: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  mj_forward(m_model, m_data);
  StateSnapshot initial;
  publish_snapshot(initial);
  for (size_t i = 0; i < kJoints; ++i) {
    hw_states_position_[i] = initial.position[i];
    hw_states_velocity_[i] = initial.velocity[i];
    hw_states_effort_[i] = initial.effort[i];
  }
  RCLCPP_INFO(rclcpp::get_logger("Lite6MujocoSystem"), "MuJoCo Hardware Interface initialized successfully!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Lite6MujocoSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  using lite6_interfaces::shm::ShmOpenStatus;
  using lite6_interfaces::shm::shm_open_status_name;
  if (!state_shm_->valid()) {
    const auto st = state_shm_->open_status(true);
    if (st != ShmOpenStatus::OK) {
      RCLCPP_ERROR(
        rclcpp::get_logger("Lite6MujocoSystem"),
        "Failed to open state shared memory (%s)", shm_open_status_name(st));
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  if (!perturb_shm_->valid()) {
    const auto st = perturb_shm_->open_status(false);
    if (st == ShmOpenStatus::OK) {
      perturb_shm_ptr_->store(perturb_shm_->get(), std::memory_order_release);
    } else if (st == ShmOpenStatus::NOT_FOUND) {
      RCLCPP_INFO(
        rclcpp::get_logger("Lite6MujocoSystem"),
        "Perturbation shared memory not present yet; will attach lazily when viewer starts");
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("Lite6MujocoSystem"),
        "Failed to open read-only perturbation shared memory (%s)", shm_open_status_name(st));
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else {
    perturb_shm_ptr_->store(perturb_shm_->get(), std::memory_order_release);
  }
  if (!sysid_shm_->valid()) {
    const auto st = sysid_shm_->open_status(true);
    if (st != ShmOpenStatus::OK) {
      RCLCPP_ERROR(
        rclcpp::get_logger("Lite6MujocoSystem"),
        "Failed to open SysID shared memory (%s)", shm_open_status_name(st));
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  if (const auto node = get_node()) {
    clock_ = node->get_clock();
  }
  if (m_model && m_data) {
    StateSnapshot initial;
    publish_snapshot(initial);
    const int64_t now_steady = steady_now_ns();
    const int64_t now_ros = clock_ ? clock_->now().nanoseconds() : 0;
    write_joint_state_shm(initial, now_ros, now_steady);
  }
  state_shm_->get()->visual_alive_ns.store(steady_now_ns(), std::memory_order_release);
  configure_diagnostics();
  gripper_.configure(get_node());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Lite6MujocoSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  stop_physics();
  if (!m_model || !m_data) {return hardware_interface::CallbackReturn::ERROR;}
  commands_.reset();
  command_sequence_ = 0;
  std::fill(hw_commands_effort_.begin(), hw_commands_effort_.end(), 0.0);
  last_mouse_seq_ = 0;
  mjv_defaultPerturb(&last_mouse_target_.perturb);
  last_mouse_target_.issued_ns = 0;
  diagnostics_->fault.store(Fault::NONE);
  diagnostics_->last_write_ns.store(0);
  diagnostics_->max_write_interval_ns.store(0);
  StateSnapshot initial;
  publish_snapshot(initial);
  const int64_t now_steady = steady_now_ns();
  const int64_t now_ros = clock_ ? clock_->now().nanoseconds() : 0;
  write_joint_state_shm(initial, now_ros, now_steady);
  if (state_shm_ && state_shm_->valid()) {
    state_shm_->get()->visual_alive_ns.store(now_steady, std::memory_order_release);
  }
  rlimit memlock_limit{};
  if (getrlimit(RLIMIT_MEMLOCK, &memlock_limit) == 0 &&
      memlock_limit.rlim_cur == RLIM_INFINITY)
  {
    (void)mlockall(MCL_CURRENT | MCL_FUTURE);
  }
  stop_requested_.store(false);
  rt_sched_active_.store(false, std::memory_order_release);
  activation_ns_ = now_steady;
  gripper_.activate();
  diagnostics_->active.store(true);
  try {
    physics_thread_ = std::thread(&Lite6MujocoSystem::physics_loop, this);
  } catch (const std::exception & error) {
    stop_requested_.store(true);
    diagnostics_->active.store(false);
    RCLCPP_ERROR(rclcpp::get_logger("Lite6MujocoSystem"),
      "Cannot start physics thread: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  sched_param scheduling{};
  scheduling.sched_priority = physics_thread_priority_;
  const int result = pthread_setschedparam(physics_thread_.native_handle(),
    physics_thread_priority_ > 0 ? SCHED_FIFO : SCHED_OTHER, &scheduling);
  rt_sched_active_.store(
    physics_thread_priority_ > 0 && result == 0, std::memory_order_release);
  if (result != 0) {
    RCLCPP_WARN(rclcpp::get_logger("Lite6MujocoSystem"),
      "Physics thread scheduling request failed: %s; monitor /simulation/diagnostics for lag",
      std::strerror(result));
  }
  RCLCPP_INFO(rclcpp::get_logger("Lite6MujocoSystem"),
    "Independent physics enabled: fixed 0.1 ms steps paced by steady_clock; "
    "initial pose held until the first effective effort command");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Lite6MujocoSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  stop_physics();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Lite6MujocoSystem::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  stop_physics();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Lite6MujocoSystem::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  stop_physics();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Lite6MujocoSystem::on_error(
  const rclcpp_lifecycle::State &)
{
  stop_physics();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Lite6MujocoSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < kJoints; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]));
  }
  for (size_t i = 0; i < 2; ++i) {
    const std::string name = i == 0 ? "gripper_left_finger" : "gripper_right_finger";
    state_interfaces.emplace_back(name, "position", &gripper_feedback_.position[i]);
    state_interfaces.emplace_back(name, "velocity", &gripper_feedback_.velocity[i]);
    state_interfaces.emplace_back(name, "acceleration", &gripper_feedback_.acceleration[i]);
    state_interfaces.emplace_back(name, "effort", &gripper_feedback_.effort[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Lite6MujocoSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < kJoints; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_effort_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type Lite6MujocoSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (diagnostics_->fault.load() != Fault::NONE) {return hardware_interface::return_type::ERROR;}
  // Only a complete snapshot crosses threads. Never read the live mjData here.
  const auto snapshot = states_.read();
  const int64_t observed_ns = steady_now_ns();
  const int64_t sample_ns = snapshot.started ? snapshot.boundary_ns : snapshot.published_ns;
  const int64_t age_ns = observed_ns - sample_ns;
  if (diagnostics_->active.load() &&
      (sample_ns <= 0 || age_ns < 0 ||
       age_ns > lite6_interfaces::contract::kJointStateStaleTimeoutNs))
  {
    diagnostics_->fault.store(Fault::STALE_STATE);
    return hardware_interface::return_type::ERROR;
  }
  gripper_feedback_ = snapshot.gripper;
  for (size_t i = 0; i < kJoints; ++i) {
    hw_states_position_[i] = snapshot.position[i];
    hw_states_velocity_[i] = snapshot.velocity[i];
    hw_states_effort_[i] = snapshot.effort[i];
  }
  const int64_t observed_ros_ns = clock_ ? clock_->now().nanoseconds() : 0;
  write_joint_state_shm(snapshot, observed_ros_ns - age_ns, sample_ns);
  if (snapshot.started && sysid_shm_ && sysid_shm_->valid() &&
      sysid_shm_->get()->consumer_active.load(std::memory_order_acquire) != 0)
  {
    // Pair ROS and steady-clock observations here. Never label an old physical
    // snapshot as a new state merely because another controller cycle ran.
    const double physics_lag = std::max(0.0, 1e-9 * (observed_ns - snapshot.boundary_ns));
    const int64_t lag_ns = static_cast<int64_t>(std::llround(physics_lag * 1e9));
    lite6_interfaces::shm::SysIdSamplePod sample{};
    sample.stamp_ns = observed_ros_ns - lag_ns;
    sample.read_stamp_ns = observed_ros_ns;
    sample.simulation_time = snapshot.simulation_time;
    sample.step = snapshot.steps;
    sample.command_sequence = snapshot.command_sequence;
    sample.control_period = period.seconds();
    sample.physics_lag = physics_lag;
    sample.command_age = std::max(0.0, 1e-9 * (snapshot.boundary_ns - snapshot.command_ns));
    for (size_t i = 0; i < kJoints; ++i) {
      sample.position[i] = snapshot.position[i];
      sample.velocity[i] = snapshot.velocity[i];
      sample.effort[i] = snapshot.effort[i];
    }
    for (size_t i = 0; i < 2; ++i) {
      sample.gripper_position[i] = snapshot.gripper.position[i];
      sample.gripper_velocity[i] = snapshot.gripper.velocity[i];
      sample.gripper_acceleration[i] = snapshot.gripper.acceleration[i];
    }
    sysid_shm_->get()->ring.push(sample);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Lite6MujocoSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (diagnostics_->fault.load() != Fault::NONE) {return hardware_interface::return_type::ERROR;}
  if (!diagnostics_->active.load()) {return hardware_interface::return_type::OK;}
  EffortCommand command;
  for (size_t i = 0; i < kJoints; ++i) {
    command.effort[i] = hw_commands_effort_[i];
    if (!std::isfinite(command.effort[i])) {
      diagnostics_->fault.store(Fault::INVALID_COMMAND);
      return hardware_interface::return_type::ERROR;
    }
  }
  command.sequence = ++command_sequence_;
  command.issued_ns = steady_now_ns();
  if (!commands_.push(command)) {
    diagnostics_->fault.store(Fault::COMMAND_OVERFLOW);
    return hardware_interface::return_type::ERROR;
  }
  const auto previous_write = diagnostics_->last_write_ns.load();
  if (previous_write != 0) {
    // write() is the sole producer; retain short delays between timer samples.
    diagnostics_->max_write_interval_ns.store(std::max(
      diagnostics_->max_write_interval_ns.load(), command.issued_ns - previous_write));
  }
  diagnostics_->last_write_ns.store(command.issued_ns);
  return hardware_interface::return_type::OK;
}

void Lite6MujocoSystem::refresh_mouse_target(int64_t now_ns)
{
  const auto * seg = perturb_shm_ptr_->load(std::memory_order_acquire);
  if (seg == nullptr) {return;}
  lite6_interfaces::shm::MousePerturbationPod message{};
  if (!seg->mouse_perturbation.read_if_updated(message, last_mouse_seq_)) {
    return;
  }
  MouseTarget target;
  mjv_defaultPerturb(&target.perturb);
  auto finite = [](const double * begin, const double * end) {
      return std::all_of(begin, end,
        [](double v) {return std::isfinite(v) && std::abs(v) <= 100.0;});
    };
  const bool valid =
    lite6_interfaces::shm::is_stamp_fresh(message.issued_ns, now_ns, 200000000LL) &&
    std::find(perturb_bodies_.begin(), perturb_bodies_.end(), message.body_id) !=
      perturb_bodies_.end() &&
    (message.active == mjPERT_TRANSLATE || message.active == mjPERT_ROTATE) &&
    finite(message.refpos, message.refpos + 3) &&
    finite(message.refselpos, message.refselpos + 3) &&
    finite(message.refquat, message.refquat + 4) &&
    finite(message.localpos, message.localpos + 3) &&
    std::isfinite(message.localmass) && message.localmass > 0 && message.localmass < 100;
  if (valid) {
    target.issued_ns = message.issued_ns;
    target.perturb.select = message.body_id;
    target.perturb.active = message.active;
    mju_copy3(target.perturb.refpos, message.refpos);
    mju_copy3(target.perturb.refselpos, message.refselpos);
    mju_copy4(target.perturb.refquat, message.refquat);
    if (mju_normalize4(target.perturb.refquat) < 1e-6) {
      target.perturb.active = 0;
    }
    mju_copy3(target.perturb.localpos, message.localpos);
    target.perturb.localmass = message.localmass;
  }
  last_mouse_target_ = target;
}

bool Lite6MujocoSystem::advance_physics(int64_t boundary_ns, EffortCommand & applied)
{
  // Only this thread touches live mjData. Recompute the native spring against
  // the current physics state, rather than holding a force computed at 60 Hz.
  mju_zero(m_data->xfrc_applied, 6 * m_model->nbody);
  const auto now = steady_now_ns();
  refresh_mouse_target(now);
  if (last_mouse_target_.issued_ns >= activation_ns_ &&
    last_mouse_target_.issued_ns <= boundary_ns &&
    lite6_interfaces::shm::is_stamp_fresh(last_mouse_target_.issued_ns, now, 200000000LL) &&
    last_mouse_target_.perturb.active)
  {
    mjv_applyPerturbForce(m_model, m_data, &last_mouse_target_.perturb);
  }
  // A command takes effect at the first step START at or after its submission.
  // In particular, a new command cannot be applied to overdue historical steps.
  EffortCommand next;
  while (commands_.pop_due(boundary_ns, next)) {applied = next;}
  for (size_t i = 0; i < kJoints; ++i) {
    m_data->ctrl[ctrl_indices_[i]] = applied.effort[i];
    const int dof = qvel_indices_[i];
    m_data->qfrc_applied[dof] = -friction_models_[i].step(
      m_data->qvel[dof], m_model->opt.timestep);
  }
  gripper_.step(boundary_ns, now);
  const double expected_time = m_data->time + m_model->opt.timestep;
  mj_step(m_model, m_data);
  bool valid = gripper_.finite() && std::isfinite(m_data->time) && std::abs(m_data->time - expected_time) < 1e-9;
  for (size_t i = 0; i < kJoints; ++i) {
    valid = valid && std::isfinite(m_data->qpos[qpos_indices_[i]]) &&
      std::isfinite(m_data->qvel[qvel_indices_[i]]) &&
      std::isfinite(m_data->qfrc_actuator[qvel_indices_[i]]);
  }
  if (!valid) {diagnostics_->fault.store(Fault::INVALID_STATE);}
  return valid;
}

void Lite6MujocoSystem::write_visual_shm(const StateSnapshot & snapshot, bool physics_active)
{
  if (!state_shm_ || !state_shm_->valid()) {return;}
  lite6_interfaces::shm::VisualStatePod pod{};
  for (size_t i = 0; i < kJoints; ++i) {
    pod.position[i] = snapshot.position[i];
    pod.velocity[i] = snapshot.velocity[i];
  }
  for (size_t i = 0; i < 2; ++i) {
    pod.gripper_position[i] = snapshot.gripper.position[i];
    pod.gripper_velocity[i] = snapshot.gripper.velocity[i];
  }
  std::copy_n(snapshot.object_qpos.begin(), 14, pod.object_qpos);
  pod.simulation_time = snapshot.simulation_time;
  pod.published_steady_ns = snapshot.published_ns;
  pod.steps = snapshot.steps;
  pod.active = physics_active ? 1 : 0;
  pod.valid = 1;
  state_shm_->get()->visual_state.write(pod);
}

void Lite6MujocoSystem::write_joint_state_shm(
  const StateSnapshot & snapshot, int64_t ros_stamp_ns, int64_t steady_stamp_ns)
{
  if (!state_shm_ || !state_shm_->valid() ||
      snapshot.published_ns <= last_joint_snapshot_ns_) {return;}
  last_joint_snapshot_ns_ = snapshot.published_ns;
  lite6_interfaces::shm::JointStatePod pod{};
  for (size_t i = 0; i < kJoints; ++i) {
    pod.position[i] = snapshot.position[i];
    pod.velocity[i] = snapshot.velocity[i];
    pod.effort[i] = snapshot.effort[i];
  }
  for (size_t i = 0; i < 2; ++i) {
    pod.gripper_position[i] = snapshot.gripper.position[i];
    pod.gripper_velocity[i] = snapshot.gripper.velocity[i];
  }
  pod.ros_stamp_ns = ros_stamp_ns;
  pod.steady_stamp_ns = steady_stamp_ns;
  pod.sequence = ++joint_state_sequence_;
  pod.valid = 1;
  state_shm_->get()->joint_state.write(pod);
}

void Lite6MujocoSystem::publish_snapshot(StateSnapshot & snapshot, bool publish_telemetry)
{
  for (size_t i = 0; i < kJoints; ++i) {
    snapshot.position[i] = m_data->qpos[qpos_indices_[i]];
    snapshot.velocity[i] = m_data->qvel[qvel_indices_[i]];
    snapshot.effort[i] = m_data->qfrc_actuator[qvel_indices_[i]];
  }
  for (size_t i = 0; i < object_qpos_indices_.size(); ++i) {
    std::copy_n(m_data->qpos + object_qpos_indices_[i], 7,
      snapshot.object_qpos.begin() + 7 * i);
  }
  snapshot.gripper = gripper_.feedback();
  snapshot.simulation_time = m_data->time;
  snapshot.published_ns = steady_now_ns();
  if (snapshot.started) {
    snapshot.max_lag = std::max(snapshot.max_lag,
      std::max(0.0, 1e-9 * (snapshot.published_ns - snapshot.boundary_ns)));
  }
  states_.publish(snapshot);
  if (publish_telemetry) {
    diagnostics_->snapshots.publish(snapshot);
    const bool vis_active = diagnostics_->active.load() && snapshot.started &&
      diagnostics_->fault.load() == Fault::NONE;
    write_visual_shm(snapshot, vis_active);
  }
}

void Lite6MujocoSystem::physics_loop()
{
  pthread_setname_np(pthread_self(), "lite6_physics");
  // Reduce Linux timer coalescing slack so 100 us substeps wake closer to
  // their deadline even on standard kernels without PREEMPT_RT.
  (void)prctl(PR_SET_TIMERSLACK, 1UL, 0, 0, 0);
  // Pre-fault physics thread stack pages before entering the 10 kHz RT loop.
  prefault_physics_stack();
  try {
    StateSnapshot snapshot;
    EffortCommand applied;
    while (!stop_requested_.load() && diagnostics_->fault.load() == Fault::NONE) {
      if (!snapshot.started) {
        publish_snapshot(snapshot, false);
        // Preserve the original startup hold so gravity does not move the arm
        // while controller_manager is still loading the effort controller.
        if (!commands_.pop(applied)) {
          std::this_thread::sleep_for(std::chrono::nanoseconds(kStepNs));
          continue;
        }
        bool controller_ready = std::any_of(applied.effort.begin(), applied.effort.end(),
          [](double effort) {return std::abs(effort) > 0.01;});
        if (!controller_ready && state_shm_ && state_shm_->valid()) {
          lite6_interfaces::shm::ShadowStatePod shadow{};
          uint64_t shadow_seq = 0;
          if ((*state_shm_)->shadow_state.read(shadow, shadow_seq) &&
              shadow.valid != 0U &&
              shadow.stamp_ns >= activation_ns_ &&
              (steady_now_ns() - shadow.stamp_ns) <= 250000000LL) {
            controller_ready = true;
          }
        }
        if (!controller_ready) {
          continue;
        }
        snapshot.started = true;
        snapshot.origin_ns = applied.issued_ns;
        snapshot.boundary_ns = applied.issued_ns;
      }

      const int64_t start_ns = snapshot.boundary_ns;
      const int64_t end_ns = start_ns + kStepNs;
      // Absolute steady-clock deadlines avoid accumulating relative sleep drift.
      // A late wakeup is paid back with fixed-size substeps, never a larger dt,
      // dropped physics time, or a controller-triggered reset of the epoch.
      if (steady_now_ns() < end_ns) {
        std::this_thread::sleep_until(Clock::time_point(
          std::chrono::duration_cast<Clock::duration>(std::chrono::nanoseconds(end_ns))));
      }
      if (stop_requested_.load() || diagnostics_->fault.load() != Fault::NONE) {break;}
      if (!advance_physics(start_ns, applied)) {break;}
      ++snapshot.steps;
      snapshot.boundary_ns = end_ns;
      snapshot.command_ns = applied.issued_ns;
      snapshot.command_sequence = applied.sequence;
      // Publish visual SHM and diagnostics snapshots at 1 kHz instead of 10 kHz.
      const bool publish_telemetry = (snapshot.steps == 1) || (snapshot.steps % 10 == 0);
      publish_snapshot(snapshot, publish_telemetry);
      // Keep shutdown/other work responsive during prolonged overload. The
      // outstanding simulation time is retained and visible in diagnostics.
      const uint64_t yield_stride =
        rt_sched_active_.load(std::memory_order_acquire) ? 20ULL : 100ULL;
      if (snapshot.steps % yield_stride == 0 && steady_now_ns() >= end_ns + kStepNs) {
        // FIFO yield alone cannot run a lower-priority controller thread.
        std::this_thread::sleep_for(std::chrono::microseconds(1));
      }
    }
  } catch (const std::exception &) {
    diagnostics_->fault.store(Fault::PHYSICS_EXCEPTION);
  } catch (...) {
    diagnostics_->fault.store(Fault::PHYSICS_EXCEPTION);
  }
  gripper_.deactivate();
  diagnostics_->active.store(false);
  if (state_shm_ && state_shm_->valid()) {
    lite6_interfaces::shm::VisualStatePod pod{};
    uint64_t seq = 0;
    if (state_shm_->get()->visual_state.read(pod, seq)) {
      pod.active = 0;
      pod.published_steady_ns = steady_now_ns();
      state_shm_->get()->visual_state.write(pod);
    }
  }
}

void Lite6MujocoSystem::configure_diagnostics()
{
  if (diagnostics_timer_) {return;}
  const auto node = get_node();
  if (!node) {return;}
  auto publisher = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/simulation/diagnostics", rclcpp::QoS(1).best_effort());
  const auto clock = node->get_clock();
  const auto diagnostics = diagnostics_;
  const auto state_shm = state_shm_;
  const auto perturb_shm = perturb_shm_;
  const auto perturb_shm_ptr = perturb_shm_ptr_;
  const auto sysid_shm = sysid_shm_;
  const auto hardware_name = info_.name;
  auto perturb_incompat_logged = std::make_shared<bool>(false);
  // No captured hardware pointer: an in-flight callback remains valid even
  // during plugin destruction. All ROS formatting/publication is non-RT.
  diagnostics_timer_ = node->create_wall_timer(std::chrono::milliseconds(100),
    [diagnostics, publisher, clock, hardware_name, state_shm, perturb_shm, perturb_shm_ptr,
     sysid_shm, perturb_incompat_logged] {
      const int64_t now = steady_now_ns();
      if (state_shm && state_shm->valid()) {
        state_shm->get()->visual_alive_ns.store(now, std::memory_order_release);
      }
      if (perturb_shm && !perturb_shm->valid()) {
        const auto st = perturb_shm->open_status(false);
        if (st == lite6_interfaces::shm::ShmOpenStatus::OK) {
          perturb_shm_ptr->store(perturb_shm->get(), std::memory_order_release);
          *perturb_incompat_logged = false;
        } else if (st == lite6_interfaces::shm::ShmOpenStatus::INCOMPATIBLE &&
                   !*perturb_incompat_logged)
        {
          *perturb_incompat_logged = true;
          RCLCPP_ERROR(
            rclcpp::get_logger("Lite6MujocoSystem"),
            "Incompatible perturbation shared memory segment detected (magic/version/size mismatch)");
        }
      }
      bool sysid_obs = false;
      if (sysid_shm && sysid_shm->valid() &&
          sysid_shm->get()->consumer_active.load(std::memory_order_acquire) != 0)
      {
        const int64_t hb =
          sysid_shm->get()->consumer_heartbeat_ns.load(std::memory_order_acquire);
        if (lite6_interfaces::shm::is_stamp_fresh(hb, now, 1500000000LL)) {
          sysid_obs = true;
        } else {
          sysid_shm->get()->consumer_active.store(0U, std::memory_order_release);
        }
      }
      diagnostics->sysid_observed.store(sysid_obs);
      const auto snapshot = diagnostics->snapshots.read();
      const bool active = diagnostics->active.load();
      const auto fault = diagnostics->fault.load();
      const auto last_write = diagnostics->last_write_ns.load();
      const double elapsed = snapshot.started ? 1e-9 * (now - snapshot.origin_ns) : 0.0;
      const double lag = snapshot.started && active ?
        std::max(0.0, 1e-9 * (now - snapshot.boundary_ns)) : 0.0;
      const double write_age = last_write ? std::max(0.0, 1e-9 * (now - last_write)) : 0.0;
      diagnostic_msgs::msg::DiagnosticArray message;
      message.header.stamp = clock->now();
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "lite6_simulation/timing";
      status.hardware_id = hardware_name;
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = !active ? "Physics stopped" :
        (snapshot.started ? "Physics running independently" : "Waiting for first effective effort");
      if (fault != Fault::NONE) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        switch (fault) {
          case Fault::COMMAND_OVERFLOW: status.message = "Command history overflow; simulation stopped"; break;
          case Fault::INVALID_COMMAND: status.message = "Non-finite effort command; simulation stopped"; break;
          case Fault::STALE_STATE: status.message = "Stale physics feedback; simulation stopped"; break;
          case Fault::INVALID_STATE: status.message = "Invalid/reset MuJoCo state; simulation stopped"; break;
          default: status.message = "Physics thread exception; simulation stopped"; break;
        }
      } else if (active && snapshot.started && (lag > 0.002 || write_age > 0.005)) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = lag > 0.002 ? "Physics is behind steady clock" :
          "Control write is late; physics continues with the last effort";
      }
      auto add = [&status](const std::string & key, const std::string & value) {
          diagnostic_msgs::msg::KeyValue entry;
          entry.key = key; entry.value = value; status.values.push_back(std::move(entry));
        };
      add("physics_active", active ? "true" : "false");
      add("started", snapshot.started ? "true" : "false");
      add("step_s", "0.000100");
      add("simulation_time_s", std::to_string(snapshot.simulation_time));
      add("steps_since_start", std::to_string(snapshot.steps));
      add("wall_elapsed_s", std::to_string(elapsed));
      add("simulation_elapsed_s", std::to_string(snapshot.steps * 0.0001));
      add("lag_s", std::to_string(lag));
      add("max_lag_s", std::to_string(snapshot.max_lag));
      add("write_age_s", std::to_string(write_age));
      add("max_write_interval_s", std::to_string(
        1e-9 * diagnostics->max_write_interval_ns.load()));
      add("applied_command_age_s", std::to_string(snapshot.started ?
        std::max(0.0, 1e-9 * (now - snapshot.command_ns)) : 0.0));
      add("snapshot_age_s", std::to_string(snapshot.published_ns ?
        std::max(0.0, 1e-9 * (now - snapshot.published_ns)) : 0.0));
      add("applied_command_sequence", std::to_string(snapshot.command_sequence));
      message.status.push_back(std::move(status));
      publisher->publish(message);
    });
}

}  // namespace lite6_hardware

PLUGINLIB_EXPORT_CLASS(
  lite6_hardware::Lite6MujocoSystem,
  hardware_interface::SystemInterface)
