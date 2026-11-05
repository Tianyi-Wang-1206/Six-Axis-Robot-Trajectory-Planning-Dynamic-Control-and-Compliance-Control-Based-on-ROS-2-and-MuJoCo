#include "lite6_controllers/ctc_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <cmath>
#include <algorithm>

namespace lite6_controllers
{

Lite6CTCController::Lite6CTCController() {}

controller_interface::CallbackReturn Lite6CTCController::on_init()
{
  joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  reference_interfaces_.resize(3 * joint_names_.size(), 0.0);
  
  auto_declare<std::vector<double>>("kp", {800.0, 800.0, 800.0, 800.0, 800.0, 800.0});
  auto_declare<std::vector<double>>("kv", {56.6, 56.6, 56.6, 56.6, 56.6, 56.6});
  auto_declare<std::vector<double>>("ki", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  auto_declare<std::vector<double>>("integral_torque_limit", {2.0, 2.0, 1.5, 0.7, 0.3, 0.15});
  auto_declare<double>("integral_position_window", 0.017453292519943295);
  auto_declare<double>("integral_velocity_window", 0.02);
  
  auto_declare<std::vector<double>>("friction_v_nominal", {1.0, 1.0, 0.5, 0.2, 0.1, 0.05});
  auto_declare<std::vector<double>>("friction_c_nominal", {1.5, 1.5, 1.0, 0.5, 0.2, 0.1});
  auto_declare<std::vector<double>>("armature", {0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
  
  auto_declare<std::vector<double>>("max_effort", {50.0, 50.0, 32.0, 32.0, 32.0, 20.0}); 
  auto_declare<double>("tanh_slope", 300.0);

  auto_declare<std::vector<double>>("estop_decel", {17.5, 12.5, 20.0, 40.0, 40.0, 50.0});
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration Lite6CTCController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) config.names.push_back(joint + "/effort");
  return config;
}

controller_interface::InterfaceConfiguration Lite6CTCController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // Keep positions first, followed by velocities, in the requested interface order.
  for (const auto & joint : joint_names_) config.names.push_back(joint + "/position");
  for (const auto & joint : joint_names_) config.names.push_back(joint + "/velocity");
  return config;
}

std::vector<hardware_interface::CommandInterface> Lite6CTCController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ref_interfaces;
  size_t n = joint_names_.size();
  for (size_t i = 0; i < n; ++i) {
    ref_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), joint_names_[i] + "/position", &reference_interfaces_[i]));
    ref_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), joint_names_[i] + "/velocity", &reference_interfaces_[n + i]));
    ref_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), joint_names_[i] + "/acceleration", &reference_interfaces_[2 * n + i]));
  }
  return ref_interfaces;
}

bool Lite6CTCController::on_set_chained_mode(bool) { return true; }
controller_interface::return_type Lite6CTCController::update_reference_from_subscribers(const rclcpp::Time &, const rclcpp::Duration &) { return controller_interface::return_type::OK; }

controller_interface::CallbackReturn Lite6CTCController::on_configure(const rclcpp_lifecycle::State &)
{
  size_t n = joint_names_.size();
  
  q_.setZero(n); dq_.setZero(n); 
  estop_q_.setZero(n); estop_dq_.setZero(n); 
  
  e_.setZero(n); de_.setZero(n); a_d_.setZero(n);
  tau_fric_.setZero(n); tau_cmd_.setZero(n);
  q_target_.setZero(n); dq_target_.setZero(n); ddq_target_.setZero(n);

  auto kp_v = get_node()->get_parameter("kp").as_double_array();
  auto kv_v = get_node()->get_parameter("kv").as_double_array();
  auto ki_v = get_node()->get_parameter("ki").as_double_array();
  auto integral_limit_v = get_node()->get_parameter("integral_torque_limit").as_double_array();
  integral_position_window_ = get_node()->get_parameter("integral_position_window").as_double();
  integral_velocity_window_ = get_node()->get_parameter("integral_velocity_window").as_double();
  const auto finite_nonnegative = [](double value) {
    return std::isfinite(value) && value >= 0.0;
  };
  if (ki_v.size() != n || integral_limit_v.size() != n ||
      !std::all_of(ki_v.begin(), ki_v.end(), finite_nonnegative) ||
      !std::all_of(integral_limit_v.begin(), integral_limit_v.end(), finite_nonnegative) ||
      !std::isfinite(integral_position_window_) || integral_position_window_ <= 0.0 ||
      !std::isfinite(integral_velocity_window_) || integral_velocity_window_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Integral gains/torque limits must contain six finite nonnegative values; "
      "integral position/velocity windows must be finite and positive");
    return controller_interface::CallbackReturn::ERROR;
  }
  integral_gain_ = Eigen::Map<const Eigen::VectorXd>(ki_v.data(), n);
  integral_torque_limit_ = Eigen::Map<const Eigen::VectorXd>(integral_limit_v.data(), n);
  integral_acceleration_.setZero(n);
  integral_torque_.setZero(n);
  integral_mass_.setZero(n, n);
  integral_enabled_ = (integral_gain_.array() > 0.0).any();
  auto fv_v = get_node()->get_parameter("friction_v_nominal").as_double_array();
  auto fc_v = get_node()->get_parameter("friction_c_nominal").as_double_array();
  auto arm_v = get_node()->get_parameter("armature").as_double_array();
  auto effort_limit_v = get_node()->get_parameter("max_effort").as_double_array();
  auto estop_decel_v = get_node()->get_parameter("estop_decel").as_double_array();

  Kp_ = Eigen::MatrixXd::Zero(n, n);
  Kv_ = Eigen::MatrixXd::Zero(n, n);
  friction_v_.resize(n); friction_c_.resize(n); armature_.resize(n); max_effort_.resize(n); estop_decel_.resize(n);

  for(size_t i=0; i<n; ++i) {
      Kp_(i, i) = kp_v[i]; Kv_(i, i) = kv_v[i]; 
      friction_v_[i] = fv_v[i]; friction_c_[i] = fc_v[i]; armature_[i] = arm_v[i];
      max_effort_[i] = effort_limit_v[i];
      estop_decel_[i] = estop_decel_v[i];
  }
  tanh_slope_ = get_node()->get_parameter("tanh_slope").as_double();

  cmd_buffer_.writeFromNonRT(0);
  cmd_sub_ = get_node()->create_subscription<std_msgs::msg::Int32>(
    "~/system_cmd", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Int32::SharedPtr msg) { cmd_buffer_.writeFromNonRT(msg->data); });

  try {
    std::string urdf_path = ament_index_cpp::get_package_share_directory("lite6_description") + "/urdf/lite6.urdf";
    pinocchio::urdf::buildModel(urdf_path, pin_model_);
    pin_data_ = pinocchio::Data(pin_model_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load URDF");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize the standard ROS publisher
  shadow_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>("/shadow/joint_states", rclcpp::SystemDefaultsQoS());
  
  // Wrap it in the RealtimePublisher
  rt_shadow_pub_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(shadow_pub_);
  
  // Pre-allocate the memory for the message to avoid dynamic memory allocation in the RT loop
  rt_shadow_pub_->msg_.name = joint_names_;
  rt_shadow_pub_->msg_.position.resize(n, 0.0);
  rt_shadow_pub_->msg_.velocity.resize(n, 0.0);
  rt_shadow_pub_->msg_.effort.resize(n, 0.0);

  // Observe the actual CTC mode without modifying any control behavior.
  // Reliable + volatile matches the planner's /ctc_controller/status subscriber.
  status_pub_ = get_node()->create_publisher<lite6_interfaces::msg::ControllerStatus>(
    "/ctc_controller/status", rclcpp::QoS(10).reliable());
  rt_status_pub_ = std::make_shared<
    realtime_tools::RealtimePublisher<lite6_interfaces::msg::ControllerStatus>>(status_pub_);
  rt_status_pub_->msg_.mode = lite6_interfaces::msg::ControllerStatus::MODE_UNKNOWN;
  status_publish_counter_ = 0;
  status_publish_pending_ = true;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Lite6CTCController::on_activate(const rclcpp_lifecycle::State &)
{
  size_t n = joint_names_.size();

  for (size_t i = 0; i < n; ++i) {
    double initial_pos = state_interfaces_[i].get_value();
    q_[i] = initial_pos;
    dq_[i] = state_interfaces_[n + i].get_value();
    reference_interfaces_[i] = initial_pos;
    reference_interfaces_[n + i] = 0.0;
    reference_interfaces_[2 * n + i] = 0.0;
  }
  
  system_state_ = SystemState::NORMAL;
  integral_acceleration_.setZero();
  cmd_buffer_.writeFromNonRT(0);
  // Force a fresh NORMAL observation on the first successful update.
  // Never infer a mode from the last received system_cmd value.
  status_publish_counter_ = 0;
  status_publish_pending_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Lite6CTCController::on_deactivate(const rclcpp_lifecycle::State &)
{
  integral_acceleration_.setZero();
  return controller_interface::CallbackReturn::SUCCESS;
}

void Lite6CTCController::apply_integral(double dt)
{
  // Preserve the original PD + feedforward command if integration is disabled.
  if (!integral_enabled_ || !std::isfinite(dt) || !q_.allFinite() ||
      !dq_.allFinite() || !e_.allFinite() || !dq_target_.allFinite() ||
      !tau_cmd_.allFinite()) return;

  pinocchio::crba(pin_model_, pin_data_, q_);
  integral_mass_ = pin_data_.M.selfadjointView<Eigen::Upper>();
  integral_mass_.diagonal() += armature_;
  if (!integral_mass_.allFinite()) return;

  // Limit actual integral torque, including coupling and changing inertia.
  // Uniform scaling preserves zero integral acceleration on axes with Ki == 0.
  integral_torque_.noalias() = integral_mass_ * integral_acceleration_;
  double retained_scale = 1.0;
  for (Eigen::Index j = 0; j < integral_torque_.size(); ++j) {
    const double magnitude = std::abs(integral_torque_[j]);
    if (magnitude > integral_torque_limit_[j]) {
      retained_scale = std::min(retained_scale, integral_torque_limit_[j] / magnitude);
    }
  }
  integral_acceleration_ *= retained_scale;
  integral_torque_ *= retained_scale;

  for (Eigen::Index i = 0; i < e_.size(); ++i) {
    // Freeze (do not clear) outside the integration window, including motion.
    // There is deliberately no small-error deadband or reset on reaching goal.
    if (integral_gain_[i] == 0.0 || std::abs(e_[i]) > integral_position_window_ ||
        std::abs(dq_[i]) > integral_velocity_window_ ||
        std::abs(dq_target_[i]) > integral_velocity_window_) continue;

    const double increment = integral_gain_[i] * e_[i] * dt;
    if (!std::isfinite(increment)) continue;
    double scale = 1.0;
    for (Eigen::Index j = 0; j < e_.size(); ++j) {
      const double delta_torque = integral_mass_(j, i) * increment;
      if (delta_torque == 0.0) continue;
      const double direction = delta_torque > 0.0 ? 1.0 : -1.0;
      const double integral_room = integral_torque_limit_[j] - direction * integral_torque_[j];
      const double actuator_room = max_effort_[j] - direction * (tau_cmd_[j] + integral_torque_[j]);
      // Conditional integration: block increments driving ANY coupled actuator
      // further into saturation, but allow increments that unwind saturation.
      const double room = std::max(0.0, std::min(integral_room, actuator_room));
      scale = std::min(scale, room / std::abs(delta_torque));
    }
    const double accepted = scale * increment;
    integral_acceleration_[i] += accepted;
    for (Eigen::Index j = 0; j < e_.size(); ++j) {
      integral_torque_[j] += integral_mass_(j, i) * accepted;
    }
  }

  // Equivalent to adding Ki * integral(e dt) to a_d before RNEA and armature.
  // Keeping the original command as the baseline enables torque anti-windup.
  a_d_ += integral_acceleration_;
  tau_cmd_ += integral_torque_;
}

controller_interface::return_type Lite6CTCController::update_and_write_commands(
  const rclcpp::Time & , const rclcpp::Duration & period)
{
  double dt = period.seconds();
  if (dt <= 0.0) return controller_interface::return_type::OK;
  size_t n = joint_names_.size();

  // 1. Read position and velocity from the same hardware feedback cycle.
  for (size_t i = 0; i < n; ++i) {
    q_[i] = state_interfaces_[i].get_value();
    dq_[i] = state_interfaces_[n + i].get_value();
  }

  // 2. State Machine & Kinematic Deceleration Trajectory Generation
  int32_t* cmd_ptr = cmd_buffer_.readFromRT();
  int32_t current_cmd = cmd_ptr ? *cmd_ptr : 0;
  const SystemState previous_state = system_state_;

  if (current_cmd == 1 && system_state_ != SystemState::ESTOP) {
      system_state_ = SystemState::ESTOP;
      estop_q_ = q_; 
      estop_dq_ = dq_;
  } else if (current_cmd == 2 && system_state_ != SystemState::SYS_ID) {
      system_state_ = SystemState::SYS_ID;
  } else if (current_cmd == 0 && system_state_ != SystemState::NORMAL) {
      Eigen::Map<Eigen::VectorXd> map_q_d(reference_interfaces_.data(), n);
      if ((map_q_d - q_).cwiseAbs().maxCoeff() < 0.05) {
          system_state_ = SystemState::NORMAL;
          for(size_t i=0; i<n; ++i) {
              reference_interfaces_[i] = q_[i];
              reference_interfaces_[n+i] = 0.0;
              reference_interfaces_[2*n+i] = 0.0;
          }
      }
  }

  if (system_state_ != previous_state || system_state_ != SystemState::NORMAL) {
      integral_acceleration_.setZero();
  }

  Eigen::Map<Eigen::VectorXd> map_q_d(reference_interfaces_.data(), n);
  Eigen::Map<Eigen::VectorXd> map_dq_d(reference_interfaces_.data() + n, n);
  Eigen::Map<Eigen::VectorXd> map_ddq_d(reference_interfaces_.data() + 2 * n, n);

  if (system_state_ == SystemState::NORMAL || system_state_ == SystemState::SYS_ID) {
      q_target_ = map_q_d; dq_target_ = map_dq_d; ddq_target_ = map_ddq_d;
  } else {
      for (size_t i = 0; i < n; ++i) {
          if (std::abs(estop_dq_[i]) > 1e-4) {
              double sign = (estop_dq_[i] > 0.0) ? 1.0 : -1.0;
              ddq_target_[i] = -sign * estop_decel_[i];
              
              double next_dq = estop_dq_[i] + ddq_target_[i] * dt;
              
              if ((sign > 0.0 && next_dq <= 0.0) || (sign < 0.0 && next_dq >= 0.0)) {
                  double t_zero = std::abs(estop_dq_[i] / estop_decel_[i]);
                  estop_q_[i] += estop_dq_[i] * t_zero + 0.5 * ddq_target_[i] * t_zero * t_zero;
                  estop_dq_[i] = 0.0;
                  ddq_target_[i] = 0.0;
              } else {
                  estop_q_[i] += estop_dq_[i] * dt + 0.5 * ddq_target_[i] * dt * dt;
                  estop_dq_[i] = next_dq;
              }
          } else {
              estop_dq_[i] = 0.0;
              ddq_target_[i] = 0.0;
          }
      }
      q_target_ = estop_q_;
      dq_target_ = estop_dq_;
  }

  // 3. Error Calculation
  e_ = q_target_ - q_;
  de_ = dq_target_ - dq_;

  // 4. Branching Control Law
  if (system_state_ == SystemState::SYS_ID) {
      a_d_.noalias() = ddq_target_ + (Kp_ * e_) + (Kv_ * de_);
      pinocchio::rnea(pin_model_, pin_data_, q_, dq_, a_d_);
      for(size_t i = 0; i < n; ++i) tau_cmd_[i] = pin_data_.tau[i]; 
  } 
  else {
      a_d_.noalias() = ddq_target_ + (Kp_ * e_) + (Kv_ * de_);
      pinocchio::rnea(pin_model_, pin_data_, q_, dq_, a_d_);
      for(size_t i = 0; i < n; ++i) {
          tau_fric_[i] = friction_v_[i] * dq_[i] + friction_c_[i] * std::tanh(tanh_slope_ * (dq_target_[i] + 5.0 * e_[i]));
          tau_cmd_[i] = pin_data_.tau[i] + (armature_[i] * a_d_[i]) + tau_fric_[i];
      }
  }

  if (system_state_ == SystemState::NORMAL) apply_integral(dt);

  // 5. Hardware Limits
  for (size_t i = 0; i < n; ++i) {
      if (std::isnan(tau_cmd_[i])) tau_cmd_[i] = 0.0;
      tau_cmd_[i] = std::clamp(tau_cmd_[i], -max_effort_[i], max_effort_[i]);
      [[maybe_unused]] bool set_ok = command_interfaces_[i].set_value(tau_cmd_[i]);
  }

  // 6. Publish at 200Hz (1000Hz / 5) to match the Joint State Broadcaster
  publish_counter_++;
  if (publish_counter_ >= publish_rate_divider_) {
      publish_counter_ = 0;
      
      // trylock() ensures we never block the 1000Hz loop.
      if (rt_shadow_pub_->trylock()) {
          rt_shadow_pub_->msg_.header.stamp = get_node()->now();
          
          for (size_t i = 0; i < n; ++i) {
              rt_shadow_pub_->msg_.position[i] = q_target_[i];
              // rt_shadow_pub_->msg_.velocity[i] = dq_target_[i];    // Optional: Uncomment if you want to publish target velocities
              // rt_shadow_pub_->msg_.effort[i] = tau_cmd_[i];    // Optional: Uncomment if you want to publish commanded efforts
          }
          rt_shadow_pub_->unlockAndPublish();
      }
  }

  // 7. Publish the actual internal CTC mode, observed by the Supervisor.
  // The mode is mapped explicitly: SystemState is NOT the wire enum.
  // Publish periodically even if unchanged (heartbeat), and retry every
  // update cycle after a mode change until the nonblocking lock succeeds.
  ++status_publish_counter_;
  if (status_publish_pending_ || system_state_ != last_reported_state_ ||
      status_publish_counter_ >= status_publish_rate_divider_) {
    if (rt_status_pub_->trylock()) {
      auto & status = rt_status_pub_->msg_;
      status.stamp = get_node()->now();
      switch (system_state_) {
        case SystemState::NORMAL:
          status.mode = lite6_interfaces::msg::ControllerStatus::MODE_NORMAL;
          break;
        case SystemState::ESTOP:
          status.mode = lite6_interfaces::msg::ControllerStatus::MODE_ESTOP;
          break;
        case SystemState::SYS_ID:
          status.mode = lite6_interfaces::msg::ControllerStatus::MODE_SYSID;
          break;
      }
      rt_status_pub_->unlockAndPublish();
      last_reported_state_ = system_state_;
      status_publish_counter_ = 0;
      status_publish_pending_ = false;
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace lite6_controllers

PLUGINLIB_EXPORT_CLASS(lite6_controllers::Lite6CTCController, controller_interface::ChainableControllerInterface)
