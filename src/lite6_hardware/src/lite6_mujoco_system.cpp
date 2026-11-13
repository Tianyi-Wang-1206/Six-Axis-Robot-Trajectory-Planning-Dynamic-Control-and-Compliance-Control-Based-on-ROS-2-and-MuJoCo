#include "lite6_hardware/lite6_mujoco_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <pthread.h>
#include <sched.h>
#include <stdexcept>

namespace lite6_hardware
{

Lite6MujocoSystem::~Lite6MujocoSystem()
{
  if (diagnostics_timer_) {diagnostics_timer_->cancel();}
  if (visualization_timer_) {visualization_timer_->cancel();}
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
  stop_requested_.store(true);
  if (physics_thread_.joinable()) {physics_thread_.join();}
  diagnostics_->active.store(false);
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

  if (info_.joints.size() != kJoints) {
    RCLCPP_ERROR(rclcpp::get_logger("Lite6MujocoSystem"), "Expected exactly six joints");
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
  size_t num_joints = info_.joints.size();
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

  // 3. Mapping joints and motors
  friction_models_.clear();
  friction_models_.reserve(num_joints);
  RCLCPP_INFO(rclcpp::get_logger("Lite6MujocoSystem"), "Mapping joints and motors...");
  for (const auto & joint : info_.joints) {
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
  if (const auto node = get_node(); node && !sysid_publisher_) {
    sysid_transport_ = node->create_publisher<lite6_interfaces::msg::SysIdSample>(
      "/simulation/sysid_samples", rclcpp::QoS(200).best_effort());
    sysid_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<
      lite6_interfaces::msg::SysIdSample>>(sysid_transport_);
  }
  configure_diagnostics();
  configure_visualization();
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
  diagnostics_->fault.store(Fault::NONE);
  diagnostics_->last_write_ns.store(0);
  diagnostics_->max_write_interval_ns.store(0);
  StateSnapshot initial;
  publish_snapshot(initial);
  stop_requested_.store(false);
  activation_ns_ = steady_now_ns();
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
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Lite6MujocoSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
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
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    hw_states_position_[i] = snapshot.position[i];
    hw_states_velocity_[i] = snapshot.velocity[i];
    hw_states_effort_[i] = snapshot.effort[i];
  }
  if (snapshot.started && diagnostics_->sysid_observed.load() &&
      sysid_publisher_ && sysid_publisher_->trylock()) {
    // Pair ROS and steady-clock observations here. Never label an old physical
    // snapshot as a new state merely because another controller cycle ran.
    const auto observed = get_node()->now();
    const auto observed_ns = steady_now_ns();
    auto & sample = sysid_publisher_->msg_;
    sample.read_stamp = observed;
    sample.physics_lag = std::max(0.0, 1e-9 * (observed_ns - snapshot.boundary_ns));
    sample.stamp = observed - rclcpp::Duration::from_seconds(sample.physics_lag);
    sample.simulation_time = snapshot.simulation_time;
    sample.step = snapshot.steps;
    sample.command_sequence = snapshot.command_sequence;
    sample.control_period = period.seconds();
    sample.command_age = std::max(0.0, 1e-9 * (snapshot.boundary_ns - snapshot.command_ns));
    sample.position = snapshot.position;
    sample.velocity = snapshot.velocity;
    sample.effort = snapshot.effort;
    sysid_publisher_->unlockAndPublish();
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

bool Lite6MujocoSystem::advance_physics(int64_t boundary_ns, EffortCommand & applied)
{
  // Only this thread touches live mjData. Recompute the native spring against
  // the current physics state, rather than holding a force computed at 60 Hz.
  mju_zero(m_data->xfrc_applied, 6 * m_model->nbody);
  const auto target = visualization_->targets.read();
  const auto now = steady_now_ns();
  if (target.issued_ns >= activation_ns_ && target.issued_ns <= boundary_ns &&
    now >= target.issued_ns && now - target.issued_ns < 200000000 && target.perturb.active)
  {
    mjv_applyPerturbForce(m_model, m_data, &target.perturb);
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
  const double expected_time = m_data->time + m_model->opt.timestep;
  mj_step(m_model, m_data);
  bool valid = std::isfinite(m_data->time) && std::abs(m_data->time - expected_time) < 1e-9;
  for (size_t i = 0; i < kJoints; ++i) {
    valid = valid && std::isfinite(m_data->qpos[qpos_indices_[i]]) &&
      std::isfinite(m_data->qvel[qvel_indices_[i]]) &&
      std::isfinite(m_data->qfrc_actuator[qvel_indices_[i]]);
  }
  if (!valid) {diagnostics_->fault.store(Fault::INVALID_STATE);}
  return valid;
}

void Lite6MujocoSystem::publish_snapshot(StateSnapshot & snapshot)
{
  for (size_t i = 0; i < kJoints; ++i) {
    snapshot.position[i] = m_data->qpos[qpos_indices_[i]];
    snapshot.velocity[i] = m_data->qvel[qvel_indices_[i]];
    snapshot.effort[i] = m_data->qfrc_actuator[qvel_indices_[i]];
  }
  snapshot.simulation_time = m_data->time;
  snapshot.published_ns = steady_now_ns();
  if (snapshot.started) {
    snapshot.max_lag = std::max(snapshot.max_lag,
      std::max(0.0, 1e-9 * (snapshot.published_ns - snapshot.boundary_ns)));
  }
  states_.publish(snapshot);
  diagnostics_->snapshots.publish(snapshot);
  visualization_->snapshots.publish(snapshot);
}

void Lite6MujocoSystem::configure_visualization()
{
  if (visualization_timer_) {return;}
  const auto node = get_node();
  if (!node) {return;}
  const auto exchange = visualization_;
  const auto diagnostics = diagnostics_;
  auto publisher = node->create_publisher<lite6_interfaces::msg::SimulationState>(
    "/simulation/visual_state", rclcpp::QoS(1).best_effort());
  // A dedicated consumer: neither read() nor the diagnostics timer shares its
  // triple buffer. Callbacks capture shared storage, never the hardware object.
  visualization_timer_ = node->create_wall_timer(std::chrono::milliseconds(8),
    [exchange, diagnostics, publisher] {
      if (!publisher->get_subscription_count()) {return;}
      const auto state = exchange->snapshots.read();
      lite6_interfaces::msg::SimulationState message;
      message.position = state.position;
      message.velocity = state.velocity;
      message.simulation_time = state.simulation_time;
      message.active = diagnostics->active.load() && state.started &&
        diagnostics->fault.load() == Fault::NONE;
      publisher->publish(message);
    });
  std::array<int, kJoints> bodies{};
  for (size_t i = 0; i < kJoints; ++i) {
    bodies[i] = mj_name2id(m_model, mjOBJ_BODY, ("link" + std::to_string(i + 1)).c_str());
  }
  mouse_subscription_ = node->create_subscription<lite6_interfaces::msg::MousePerturbation>(
    "/simulation/mouse_perturbation", rclcpp::QoS(1).best_effort(),
    [exchange, bodies](const lite6_interfaces::msg::MousePerturbation & message) {
      MouseTarget target;
      mjv_defaultPerturb(&target.perturb);
      const auto now = steady_now_ns();
      auto finite = [](const auto & values) {
          return std::all_of(values.begin(), values.end(),
            [](double v) {return std::isfinite(v) && std::abs(v) <= 100.0;});
        };
      const bool valid = message.issued_ns > 0 && message.issued_ns <= now &&
        now - message.issued_ns < 200000000 &&
        std::find(bodies.begin(), bodies.end(), message.body_id) != bodies.end() &&
        (message.active == mjPERT_TRANSLATE || message.active == mjPERT_ROTATE) &&
        finite(message.refpos) && finite(message.refselpos) &&
        finite(message.refquat) && finite(message.localpos) &&
        std::isfinite(message.localmass) && message.localmass > 0 && message.localmass < 100;
      if (valid) {
        target.issued_ns = message.issued_ns;
        target.perturb.select = message.body_id;
        target.perturb.active = message.active;
        mju_copy3(target.perturb.refpos, message.refpos.data());
        mju_copy3(target.perturb.refselpos, message.refselpos.data());
        mju_copy4(target.perturb.refquat, message.refquat.data());
        if (mju_normalize4(target.perturb.refquat) < 1e-6) {
          target.perturb.active = 0;
        }
        mju_copy3(target.perturb.localpos, message.localpos.data());
        target.perturb.localmass = message.localmass;
      }
      // Release, invalid messages and stale messages all clear the spring.
      exchange->targets.publish(target);
    });
}

void Lite6MujocoSystem::physics_loop()
{
  try {
    StateSnapshot snapshot;
    EffortCommand applied;
    while (!stop_requested_.load() && diagnostics_->fault.load() == Fault::NONE) {
      if (!snapshot.started) {
        // Preserve the original startup hold so gravity does not move the arm
        // while controller_manager is still loading the effort controller.
        if (!commands_.pop(applied)) {
          std::this_thread::sleep_for(std::chrono::nanoseconds(kStepNs));
          continue;
        }
        if (std::none_of(applied.effort.begin(), applied.effort.end(),
          [](double effort) {return std::abs(effort) > 0.01;}))
        {
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
      publish_snapshot(snapshot);
      // Keep shutdown/other work responsive during prolonged overload. The
      // outstanding simulation time is retained and visible in diagnostics.
      if (snapshot.steps % 100 == 0 && steady_now_ns() >= end_ns + kStepNs) {
        // FIFO yield alone cannot run a lower-priority controller thread.
        std::this_thread::sleep_for(std::chrono::microseconds(1));
      }
    }
  } catch (const std::exception &) {
    diagnostics_->fault.store(Fault::PHYSICS_EXCEPTION);
  } catch (...) {
    diagnostics_->fault.store(Fault::PHYSICS_EXCEPTION);
  }
  diagnostics_->active.store(false);
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
  const auto sysid_transport = sysid_transport_;
  const auto hardware_name = info_.name;
  // No captured hardware pointer: an in-flight callback remains valid even
  // during plugin destruction. All ROS formatting/publication is non-RT.
  diagnostics_timer_ = node->create_wall_timer(std::chrono::milliseconds(100),
    [diagnostics, publisher, clock, hardware_name, sysid_transport] {
      diagnostics->sysid_observed.store(
        sysid_transport && sysid_transport->get_subscription_count() > 0);
      const auto snapshot = diagnostics->snapshots.read();
      const int64_t now = steady_now_ns();
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
