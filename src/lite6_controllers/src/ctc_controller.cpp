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
  auto_declare<std::vector<double>>("ki", {1000.0, 1000.0, 700.0, 350.0, 150.0, 80.0});
  auto_declare<std::vector<double>>("antiwindup_gain", {10.0, 10.0, 10.0, 10.0, 10.0, 10.0});
  auto_declare<std::vector<double>>("integral_max_effort", {2.5, 2.5, 1.7, 0.85, 0.35, 0.18});
  auto_declare<std::vector<double>>("friction_v_nominal", {1.0, 1.0, 0.5, 0.2, 0.1, 0.05});
  auto_declare<std::vector<double>>("friction_c_nominal", {1.5, 1.5, 1.0, 0.5, 0.2, 0.1});
  auto_declare<std::vector<double>>("armature", {0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
  
  auto_declare<std::vector<double>>("max_effort", {50.0, 50.0, 32.0, 32.0, 32.0, 20.0}); 
  auto_declare<double>("tanh_slope", 300.0);
  auto_declare<std::vector<double>>("friction_error_gain", {5.0, 5.0, 5.0, 5.0, 5.0, 5.0});

  auto_declare<std::vector<double>>("estop_decel", {17.5, 12.5, 20.0, 40.0, 40.0, 50.0});
  auto_declare<std::vector<double>>("execution.path_position_tolerance", std::vector<double>(6, 0.1));
  auto_declare<std::vector<double>>("execution.path_velocity_tolerance", std::vector<double>(6, 0.5));
  auto_declare<std::vector<double>>("execution.goal_position_tolerance", std::vector<double>(6, 0.01));
  auto_declare<std::vector<double>>("execution.sysid_goal_position_tolerance", std::vector<double>(6, 0.01));
  auto_declare<std::vector<double>>("execution.sysid_goal_velocity_tolerance", std::vector<double>(6, 0.005));
  auto_declare<std::vector<double>>("execution.goal_velocity_tolerance", std::vector<double>(6, 0.01));
  auto_declare<double>("execution.goal_settle_duration", 0.3);
  auto_declare<double>("execution.goal_time_tolerance", 5.0);
  auto_declare<double>("execution.saturation_timeout", 1.0);
  
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
  for (const auto * name : {"gripper_left_finger", "gripper_right_finger"}) {
    for (const auto * field : {"position", "velocity", "acceleration"}) {
      config.names.push_back(std::string(name) + "/" + field);
    }
  }
  return config;
}

controller_interface::CallbackReturn Lite6CTCController::on_configure(const rclcpp_lifecycle::State &)
{
  size_t n = joint_names_.size();
  
  q_.setZero(n); dq_.setZero(n); 
  estop_q_.setZero(n); estop_dq_.setZero(n); 
  
  e_.setZero(n); de_.setZero(n); a_d_.setZero(n);
  tau_fric_.setZero(n); tau_cmd_.setZero(n);
  integral_effort_.setZero(n);
  q_target_.setZero(n); dq_target_.setZero(n); ddq_target_.setZero(n);
  current_execution_ = nullptr;

  path_position_tolerance_ = get_node()->get_parameter("execution.path_position_tolerance").as_double_array();
  path_velocity_tolerance_ = get_node()->get_parameter("execution.path_velocity_tolerance").as_double_array();
  goal_position_tolerance_ = get_node()->get_parameter("execution.goal_position_tolerance").as_double_array();
  sysid_goal_position_tolerance_ = get_node()->get_parameter("execution.sysid_goal_position_tolerance").as_double_array();
  sysid_goal_velocity_tolerance_ = get_node()->get_parameter("execution.sysid_goal_velocity_tolerance").as_double_array();
  goal_velocity_tolerance_ = get_node()->get_parameter("execution.goal_velocity_tolerance").as_double_array();
  goal_settle_duration_ = get_node()->get_parameter("execution.goal_settle_duration").as_double();
  goal_time_tolerance_ = get_node()->get_parameter("execution.goal_time_tolerance").as_double();
  saturation_timeout_ = get_node()->get_parameter("execution.saturation_timeout").as_double();
  for (const auto * values : {&path_position_tolerance_, &path_velocity_tolerance_,
      &goal_position_tolerance_, &sysid_goal_position_tolerance_, &goal_velocity_tolerance_,
      &sysid_goal_velocity_tolerance_}) {
    if (values->size() != n || !std::all_of(values->begin(), values->end(),
        [](double value) {return std::isfinite(value) && value > 0.0;})) {
      RCLCPP_ERROR(get_node()->get_logger(), "Execution tolerances require six finite positive values");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  // The upstream planner's action deadline is trajectory duration + 12 seconds.
  if (!std::isfinite(goal_settle_duration_) || goal_settle_duration_ <= 0.0 ||
      !std::isfinite(goal_time_tolerance_) || goal_time_tolerance_ < goal_settle_duration_ ||
      goal_time_tolerance_ > 10.0 || !std::isfinite(saturation_timeout_) ||
      saturation_timeout_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "Require 0 < goal_settle_duration <= goal_time_tolerance <= 10 s and positive saturation_timeout");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < n; ++i) {
    // Leave margin below the existing NORMAL restoration guards (0.05 / 0.01).
    if (sysid_goal_position_tolerance_[i] >= 0.05 ||
        sysid_goal_velocity_tolerance_[i] >= 0.01 ||
        sysid_goal_velocity_tolerance_[i] > path_velocity_tolerance_[i]) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "SYS_ID goal tolerances must be below NORMAL restoration guards (0.05 rad, 0.01 rad/s) and within path tolerances");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (goal_position_tolerance_[i] > path_position_tolerance_[i] ||
        sysid_goal_position_tolerance_[i] > path_position_tolerance_[i] ||
        goal_velocity_tolerance_[i] > path_velocity_tolerance_[i]) {
      RCLCPP_ERROR(get_node()->get_logger(), "Goal tolerances must not exceed path tolerances");
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  auto kp_v = get_node()->get_parameter("kp").as_double_array();
  auto kv_v = get_node()->get_parameter("kv").as_double_array();
  auto ki_v = get_node()->get_parameter("ki").as_double_array();
  auto aw_v = get_node()->get_parameter("antiwindup_gain").as_double_array();
  auto integral_limit_v = get_node()->get_parameter("integral_max_effort").as_double_array();
  auto error_gain_v = get_node()->get_parameter("friction_error_gain").as_double_array();
  auto fv_v = get_node()->get_parameter("friction_v_nominal").as_double_array();
  auto fc_v = get_node()->get_parameter("friction_c_nominal").as_double_array();
  if(fv_v.size()!=n||fc_v.size()!=n) {
    RCLCPP_ERROR(get_node()->get_logger(),"Friction parameter arrays must contain six values");
    return controller_interface::CallbackReturn::ERROR;
  }
  for(size_t i=0;i<n;++i) {
    if(!std::isfinite(fv_v[i])||!std::isfinite(fc_v[i])||
       fv_v[i]<0.0||fc_v[i]<0.0) {
      RCLCPP_ERROR(get_node()->get_logger(),
          "Invalid joint%zu friction parameters: require finite Fv >= 0, Fc >= 0",i+1);
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  auto arm_v = get_node()->get_parameter("armature").as_double_array();
  auto effort_limit_v = get_node()->get_parameter("max_effort").as_double_array();
  auto estop_decel_v = get_node()->get_parameter("estop_decel").as_double_array();
  tanh_slope_ = get_node()->get_parameter("tanh_slope").as_double();
  if (kp_v.size() != n || kv_v.size() != n || ki_v.size() != n || aw_v.size() != n ||
      integral_limit_v.size() != n || error_gain_v.size() != n || arm_v.size() != n ||
      effort_limit_v.size() != n || estop_decel_v.size() != n) {
    RCLCPP_ERROR(get_node()->get_logger(), "Control parameter arrays must contain six values");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!std::isfinite(tanh_slope_) || tanh_slope_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "tanh_slope must be finite and positive");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < n; ++i) {
    if (!std::isfinite(kp_v[i]) || kp_v[i] < 0.0 ||
        !std::isfinite(kv_v[i]) || kv_v[i] < 0.0 ||
        !std::isfinite(ki_v[i]) || ki_v[i] < 0.0 ||
        !std::isfinite(aw_v[i]) || aw_v[i] < 0.0 ||
        !std::isfinite(integral_limit_v[i]) || integral_limit_v[i] < 0.0 ||
        !std::isfinite(error_gain_v[i]) || error_gain_v[i] < 0.0 ||
        !std::isfinite(arm_v[i]) || arm_v[i] < 0.0 ||
        !std::isfinite(effort_limit_v[i]) || effort_limit_v[i] <= 0.0 ||
        !std::isfinite(estop_decel_v[i]) || estop_decel_v[i] <= 0.0 ||
        integral_limit_v[i] > effort_limit_v[i] ||
        (ki_v[i] > 0.0 && (aw_v[i] <= 0.0 || integral_limit_v[i] <= 0.0))) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Invalid joint%zu control parameters: require finite nonnegative gains/inertia, "
        "positive effort/deceleration limits, and for ki > 0 a positive antiwindup_gain "
        "and 0 < integral_max_effort <= max_effort", i + 1);
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  Kp_ = Eigen::MatrixXd::Zero(n, n);
  Kv_ = Eigen::MatrixXd::Zero(n, n);
  friction_v_.resize(n); friction_c_.resize(n);
  armature_.resize(n); max_effort_.resize(n); estop_decel_.resize(n);
  ki_.resize(n); antiwindup_gain_.resize(n); integral_max_effort_.resize(n);
  friction_error_gain_.resize(n);

  for(size_t i=0; i<n; ++i) {
      Kp_(i, i) = kp_v[i]; Kv_(i, i) = kv_v[i]; 
      friction_v_[i] = fv_v[i]; friction_c_[i] = fc_v[i]; armature_[i] = arm_v[i];
      max_effort_[i] = effort_limit_v[i];
      estop_decel_[i] = estop_decel_v[i];
      ki_[i] = ki_v[i];
      antiwindup_gain_[i] = aw_v[i];
      integral_max_effort_[i] = integral_limit_v[i];
      friction_error_gain_[i] = error_gain_v[i];
  }

  cmd_buffer_.writeFromNonRT(ModeCommand{0, ++command_sequence_});
  cmd_sub_ = get_node()->create_subscription<std_msgs::msg::Int32>(
    "~/system_cmd", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Int32::SharedPtr msg) {
      cmd_buffer_.writeFromNonRT(ModeCommand{msg->data, ++command_sequence_});
    });

  try {
    std::string urdf_path = ament_index_cpp::get_package_share_directory("lite6_description") + "/urdf/lite6.urdf";
    pinocchio::urdf::buildModel(urdf_path, pin_model_);
    if (pin_model_.nq != 8 || pin_model_.nv != 8) {
      throw std::runtime_error("Expected six arm coordinates and two gripper coordinates");
    }
    for (size_t i = 0; i < 8; ++i) {
      const auto name = i < 6 ? joint_names_[i] :
        (i == 6 ? std::string("gripper_left_finger") : std::string("gripper_right_finger"));
      const auto id = pin_model_.getJointId(name);
      if (id >= pin_model_.njoints || pin_model_.joints[id].idx_q() != static_cast<int>(i)) {
        throw std::runtime_error("Unexpected dynamics joint ordering");
      }
    }
    pin_data_ = pinocchio::Data(pin_model_);
    dynamics_q_ = Eigen::VectorXd::Zero(8);
    dynamics_dq_ = Eigen::VectorXd::Zero(8);
    dynamics_ddq_ = Eigen::VectorXd::Zero(8);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load URDF");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Visualization needs the latest target, not a queue of obsolete samples.
  shadow_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
    "/shadow/joint_states", rclcpp::QoS(1).reliable());
  
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

  configure_trajectory_action();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Lite6CTCController::on_activate(const rclcpp_lifecycle::State &)
{
  size_t n = joint_names_.size();
  // Never reuse a bias learned before deactivation or a hardware restart.
  integral_effort_.setZero();
  execution_stop_latched_ = false;

  for (size_t i = 0; i < n; ++i) {
    double initial_pos = state_interfaces_[i].get_value();
    q_[i] = initial_pos;
    dq_[i] = state_interfaces_[n + i].get_value();
    if (!std::isfinite(q_[i]) || !std::isfinite(dq_[i])) {
      trajectory_enabled_.store(false);
      return controller_interface::CallbackReturn::ERROR;
    }
    reference_interfaces_[i] = initial_pos;
    reference_interfaces_[n + i] = 0.0;
    reference_interfaces_[2 * n + i] = 0.0;
  }
  
  system_state_ = SystemState::NORMAL;
  cmd_buffer_.writeFromNonRT(ModeCommand{0, ++command_sequence_});
  // Force a fresh NORMAL observation on the first successful update.
  // Never infer a mode from the last received system_cmd value.
  status_publish_counter_ = 0;
  status_publish_pending_ = true;
  trajectory_enabled_.store(true);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Lite6CTCController::on_deactivate(const rclcpp_lifecycle::State &)
{
  trajectory_enabled_.store(false);
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  if (active_trajectory_) {
    auto expected = TrajectoryResult::RUNNING;
    active_trajectory_->result.compare_exchange_strong(expected, TrajectoryResult::DEACTIVATED);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type Lite6CTCController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  current_execution_ = trajectory_buffer_.readFromRT()->get();
  try {
    return update_and_write_commands(time, period);
  } catch (const std::exception &) {
    return fail_controller(TrajectoryResult::CONTROL_EXCEPTION);
  }
}

controller_interface::return_type Lite6CTCController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  double dt = period.seconds();
  if (dt <= 0.0) return controller_interface::return_type::OK;
  size_t n = joint_names_.size();

  // 1. Read position and velocity from the same hardware feedback cycle.
  for (size_t i = 0; i < n; ++i) {
    q_[i] = state_interfaces_[i].get_value();
    dq_[i] = state_interfaces_[n + i].get_value();
    if (!std::isfinite(q_[i]) || !std::isfinite(dq_[i]))
      return fail_controller(TrajectoryResult::INVALID_STATE, static_cast<int>(i));
  }

  dynamics_q_.head(6) = q_;
  dynamics_dq_.head(6) = dq_;
  for (size_t i = 0; i < 2; ++i) {
    dynamics_q_[6+i] = state_interfaces_[2*n+3*i].get_value();
    dynamics_dq_[6+i] = state_interfaces_[2*n+3*i+1].get_value();
    dynamics_ddq_[6+i] = state_interfaces_[2*n+3*i+2].get_value();
  }
  if (!dynamics_q_.allFinite() || !dynamics_dq_.allFinite() || !dynamics_ddq_.tail(2).allFinite())
    return fail_controller(TrajectoryResult::INVALID_STATE);

  update_trajectory_reference(time, period);

  // 2. State Machine & Kinematic Deceleration Trajectory Generation
  const auto * cmd_ptr = cmd_buffer_.readFromRT();
  const int32_t current_cmd = cmd_ptr->value;
  current_command_sequence_ = cmd_ptr->sequence;

  if (current_cmd == 1 && system_state_ != SystemState::ESTOP) {
      system_state_ = SystemState::ESTOP;
      estop_q_ = q_; 
      estop_dq_ = dq_;
  } else if (current_cmd == 2 && system_state_ == SystemState::NORMAL) {
      system_state_ = SystemState::SYS_ID;
  } else if (current_cmd == 0 && system_state_ != SystemState::NORMAL &&
      (!execution_stop_latched_ || current_command_sequence_ > execution_stop_sequence_)) {
      Eigen::Map<Eigen::VectorXd> map_q_d(reference_interfaces_.data(), n);
      if ((map_q_d - q_).cwiseAbs().maxCoeff() < 0.05 &&
          dq_.cwiseAbs().maxCoeff() < 0.01) {
          system_state_ = SystemState::NORMAL;
          execution_stop_latched_ = false;
          for(size_t i=0; i<n; ++i) {
              reference_interfaces_[i] = q_[i];
              reference_interfaces_[n+i] = 0.0;
              reference_interfaces_[2*n+i] = 0.0;
          }
      }
  }

  check_trajectory_path();

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
  if (!e_.allFinite() || !de_.allFinite() || !ddq_target_.allFinite())
    return fail_controller(TrajectoryResult::INVALID_COMMAND);

  // 4. Branching Control Law
  if (system_state_ == SystemState::SYS_ID) {
      // Identification must contain neither friction nor learned bias compensation.
      integral_effort_.setZero();
      // Rotor inertia is known in every mode; only friction is unidentified.
      a_d_.noalias() = ddq_target_ + (Kp_ * e_) + (Kv_ * de_);
      dynamics_ddq_.head(6) = a_d_;
      pinocchio::rnea(pin_model_, pin_data_, dynamics_q_, dynamics_dq_, dynamics_ddq_);
      for(size_t i = 0; i < n; ++i) {
          tau_cmd_[i] = pin_data_.tau[i] + (armature_[i] * a_d_[i]);
      }
  } 
  else {
      a_d_.noalias() = ddq_target_ + (Kp_ * e_) + (Kv_ * de_);
      dynamics_ddq_.head(6) = a_d_;
      pinocchio::rnea(pin_model_, pin_data_, dynamics_q_, dynamics_dq_, dynamics_ddq_);
      for(size_t i = 0; i < n; ++i) {
          // Viscous friction and Coulomb friction with smooth reference/error direction.
          tau_fric_[i] = friction_v_[i] * dq_[i] + friction_c_[i] *
              std::tanh(tanh_slope_ * (dq_target_[i] + friction_error_gain_[i] * e_[i]));
          // ki == 0 disables both integration and back-calculation for this joint.
          if (ki_[i] == 0.0) integral_effort_[i] = 0.0;
          tau_cmd_[i] = pin_data_.tau[i] + (armature_[i] * a_d_[i]) + tau_fric_[i] +
              integral_effort_[i];
      }
  }

  if (!a_d_.allFinite() || !tau_cmd_.allFinite())
    return fail_controller(TrajectoryResult::INVALID_COMMAND);

  // 5. Limit the TOTAL effort, then update the bias for the NEXT control cycle.
  for (size_t i = 0; i < n; ++i) {
      const double raw_effort = tau_cmd_[i];
      effort_saturated_[i] = std::abs(raw_effort) > max_effort_[i];
      tau_cmd_[i] = std::clamp(raw_effort, -max_effort_[i], max_effort_[i]);
      if (system_state_ != SystemState::SYS_ID && ki_[i] > 0.0) {
          // During ESTOP retain the holding bias without integrating position
          // error. Back-calculation still unwinds bias if braking saturates.
          // NORMAL resumes from this state; trajectory completion never resets it.
          const double error_rate = system_state_ == SystemState::NORMAL ?
              ki_[i] * e_[i] : 0.0;
          const double rate = error_rate +
              antiwindup_gain_[i] * (tau_cmd_[i] - raw_effort);
          integral_effort_[i] = std::clamp(
              integral_effort_[i] + dt * rate,
              -integral_max_effort_[i], integral_max_effort_[i]);
      }
  }
  if (!integral_effort_.allFinite())
    return fail_controller(TrajectoryResult::INVALID_COMMAND);
  for (size_t i = 0; i < n; ++i) {
    if (!command_interfaces_[i].set_value(tau_cmd_[i]))
      return fail_controller(TrajectoryResult::COMMAND_WRITE, static_cast<int>(i));
  }
  // No success is published until this cycle's feedback and command writes pass.
  finish_trajectory_cycle(dt);

  // 6. Offer this cycle's actual reference to the display.
  if (rt_shadow_pub_->trylock()) {
    rt_shadow_pub_->msg_.header.stamp = time;
    for (size_t i = 0; i < n; ++i) {
      rt_shadow_pub_->msg_.position[i] = q_target_[i];
      // rt_shadow_pub_->msg_.velocity[i] = dq_target_[i];
    }
    rt_shadow_pub_->unlockAndPublish();
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
      const auto * execution = current_execution_;
      status.trajectory_start = builtin_interfaces::msg::Time{};
      status.trajectory_id.fill(0);
      if (execution && execution->started) {
        status.trajectory_start = rclcpp::Time(execution->start_time_ns);
        status.trajectory_id = execution->goal->get_goal_id();
      }
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

PLUGINLIB_EXPORT_CLASS(lite6_controllers::Lite6CTCController, controller_interface::ControllerInterface)
