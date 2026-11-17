#include "lite6_controllers/ctc_controller.hpp"
#include "lite6_interfaces/compliance_validation.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <thread>

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

  auto_declare<bool>("gmo.enabled", true);
  auto_declare<std::vector<double>>("gmo.observer_gains", {25.0, 25.0, 25.0, 30.0, 30.0, 30.0});
  auto_declare<double>("gmo.jacobian_damping", 0.04);
  auto_declare<double>("gmo.friction_transition_speed", 0.03);
  auto_declare<double>("gmo.contact_force_threshold", 2.5);

  auto_declare<double>("compliance.max_joint_velocity", 1.5);
  auto_declare<double>("compliance.hard_limit_tolerance", 0.01);
  auto_declare<double>("compliance.saturation_timeout", 1.0);
  auto_declare<double>("compliance.entry_max_speed", 0.01);
  auto_declare<std::vector<double>>(
    "compliance.max_external_joint_torque", {35.0, 35.0, 24.0, 20.0, 20.0, 12.0});
  auto_declare<double>("compliance.external_torque_timeout", 0.25);
  auto_declare<double>("compliance.friction_viscous_scale", 0.85);
  auto_declare<double>("compliance.friction_coulomb_scale", 0.0);
  auto_declare<double>("compliance.friction_transition_speed", 0.06);
  auto_declare<double>("compliance.friction_quiet_speed", 0.08);
  auto_declare<double>("compliance.filter_time_constant", 0.15);
  auto_declare<double>("compliance.cartesian_null_damping", 0.15);
  auto_declare<double>("compliance.soft_limit_margin", 0.15);
  auto_declare<double>("compliance.soft_limit_stiffness", 80.0);
  auto_declare<double>("compliance.soft_limit_damping", 5.0);
  auto_declare<double>("compliance.torque_slew_rate", 100.0);
  auto_declare<std::vector<double>>("compliance.teach.damping_levels", {4.0, 3.25, 2.5, 2.25, 2.0});
  auto_declare<std::vector<double>>("compliance.teach.assistance_levels", {0.35, 0.475, 0.60, 0.625, 0.65});
  auto_declare<std::vector<double>>("compliance.teach.friction_transition_levels", {0.06, 0.06, 0.06, 0.06, 0.06});
  auto_declare<std::vector<double>>("compliance.teach.quiet_speed_levels", {0.06, 0.06, 0.06, 0.06, 0.06});

  auto_declare<double>("impedance_execution.max_translation_deviation", 0.12);
  auto_declare<double>("impedance_execution.max_rotation_deviation", 0.60);
  auto_declare<double>("impedance_execution.max_joint_deviation", 0.80);
  auto_declare<double>("impedance_execution.max_joint_velocity", 3.0);
  auto_declare<double>("impedance_execution.hard_limit_tolerance", 0.01);
  auto_declare<double>("impedance_execution.saturation_timeout", 1.0);
  auto_declare<std::vector<double>>(
    "impedance_execution.max_external_joint_torque", {35.0, 35.0, 24.0, 20.0, 20.0, 12.0});
  auto_declare<double>("impedance_execution.external_torque_timeout", 0.25);
  auto_declare<double>("impedance_execution.friction_viscous_scale", 0.90);
  auto_declare<double>("impedance_execution.friction_coulomb_scale", 0.80);
  auto_declare<double>("impedance_execution.friction_transition_speed", 0.04);
  auto_declare<double>("impedance_execution.friction_quiet_speed", 0.01);
  auto_declare<double>("impedance_execution.goal_translation_tolerance", 0.05);
  auto_declare<double>("impedance_execution.goal_rotation_tolerance", 0.25);
  auto_declare<double>("impedance_execution.goal_velocity_tolerance", 0.01);
  auto_declare<double>("impedance_execution.goal_settle_duration", 0.3);
  auto_declare<double>("impedance_execution.goal_time_tolerance", 5.0);
  auto_declare<double>("impedance_execution.entry_max_speed", 0.02);
  auto_declare<double>("impedance_execution.entry_position_tolerance", 0.15);
  
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
  const size_t n = joint_names_.size();
  if (n != 6) {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected 6 arm joints, got %zu", n);
    return controller_interface::CallbackReturn::ERROR;
  }
  node_clock_ = get_node()->get_clock();

  q_.setZero(); dq_.setZero();
  estop_q_.setZero(); estop_dq_.setZero();

  e_.setZero(); de_.setZero(); a_d_.setZero();
  tau_fric_.setZero(); tau_cmd_.setZero();
  integral_effort_.setZero();
  q_target_.setZero(); dq_target_.setZero(); ddq_target_.setZero();
  current_execution_ = nullptr;
  current_impedance_execution_ = nullptr;
  rt_seen_execution_.store(nullptr, std::memory_order_release);
  rt_seen_impedance_execution_.store(nullptr, std::memory_order_release);

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

  kp_.setZero();
  kv_.setZero();
  friction_v_.setZero(); friction_c_.setZero();
  armature_.setZero(); max_effort_.setZero(); estop_decel_.setZero();
  ki_.setZero(); antiwindup_gain_.setZero(); integral_max_effort_.setZero();
  friction_error_gain_.setZero();

  for (size_t i = 0; i < n; ++i) {
      const Eigen::Index idx = static_cast<Eigen::Index>(i);
      kp_[idx] = kp_v[i]; kv_[idx] = kv_v[i];
      friction_v_[idx] = fv_v[i]; friction_c_[idx] = fc_v[i]; armature_[idx] = arm_v[i];
      max_effort_[idx] = effort_limit_v[i];
      estop_decel_[idx] = estop_decel_v[i];
      ki_[idx] = ki_v[i];
      antiwindup_gain_[idx] = aw_v[i];
      integral_max_effort_[idx] = integral_limit_v[i];
      friction_error_gain_[idx] = error_gain_v[i];
  }

  configure_command_service();

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
    pin_data_ref_ = pinocchio::Data(pin_model_);
    pin_data_gmo_ = pinocchio::Data(pin_model_);
    gmo_jacobian_.setZero();
    dynamics_q_.setZero();
    dynamics_dq_.setZero();
    dynamics_ddq_.setZero();
    dynamics_q_ref_.setZero();
    dynamics_dq_ref_.setZero();
    imp_ref_kinematics_cached_ = false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load URDF");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!state_shm_.valid() && !state_shm_.open(true)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to open state shared memory for shadow output");
    return controller_interface::CallbackReturn::ERROR;
  }
  shadow_epoch_ = static_cast<uint64_t>(lite6_interfaces::shm::steady_now_ns());

  // Observe the actual CTC mode without modifying any control behavior.
  // Reliable + volatile matches the planner's /ctc_controller/status subscriber.
  status_pub_ = get_node()->create_publisher<lite6_interfaces::msg::ControllerStatus>(
    "/ctc_controller/status", rclcpp::QoS(10).reliable());
  rt_status_pub_ = std::make_shared<
    realtime_tools::RealtimePublisher<lite6_interfaces::msg::ControllerStatus>>(status_pub_);
  rt_status_pub_->msg_.mode = lite6_interfaces::msg::ControllerStatus::MODE_UNKNOWN;
  rt_status_pub_->msg_.estop_reason = lite6_interfaces::msg::ControllerStatus::ESTOP_NONE;
  rt_status_pub_->msg_.estop_joint = -1;
  rt_status_pub_->msg_.estop_value = 0.0;
  rt_status_pub_->msg_.estop_limit = 0.0;
  rt_status_pub_->msg_.external_joint_torque.fill(0.0);
  rt_status_pub_->msg_.external_cartesian_wrench.fill(0.0);
  rt_status_pub_->msg_.external_contact_detected = false;
  rt_status_pub_->msg_.status_seq = 0;
  rt_status_pub_->msg_.controller_epoch = 0;
  rt_status_pub_->msg_.mode_epoch = 0;
  rt_status_pub_->msg_.last_cmd_seq = 0;
  rt_status_pub_->msg_.last_cmd_code = lite6_interfaces::srv::ControllerCommand::Request::CMD_NONE;
  rt_status_pub_->msg_.last_cmd_result = lite6_interfaces::msg::ControllerStatus::ACK_NONE;
  rt_status_pub_->msg_.rigid_trajectory_active = false;
  rt_status_pub_->msg_.payload_mass = 0.0;
  rt_status_pub_->msg_.payload_com_tcp.fill(0.0);
  status_publish_counter_ = 0;
  status_publish_pending_ = true;

  if (!configure_momentum_observer() || !configure_compliance() || !configure_impedance_trajectory()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  configure_trajectory_action();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Lite6CTCController::on_activate(const rclcpp_lifecycle::State &)
{
  size_t n = joint_names_.size();
  // Never reuse a bias learned before deactivation or a hardware restart.
  integral_effort_.setZero();
  compliance_blocks_trajectory_.store(false);
  emergency_pending_.store(false);
  compliance_stopping_elapsed_ = 0.0;
  rigid_cancel_braking_ = false;
  rigid_cancel_elapsed_ = 0.0;
  clear_estop_reason();

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
    q_imp_ref_[i] = initial_pos;
    dq_imp_ref_[i] = 0.0;
    ddq_imp_ref_[i] = 0.0;
  }

  controller_epoch_ = std::max<uint64_t>(
    controller_epoch_ + 1ULL,
    static_cast<uint64_t>(lite6_interfaces::shm::steady_now_ns()));
  shadow_epoch_ = controller_epoch_;
  mode_epoch_ = 1ULL;
  status_seq_ = 0ULL;
  last_cmd_seq_ = 0ULL;
  last_cmd_code_ = lite6_interfaces::srv::ControllerCommand::Request::CMD_NONE;
  last_cmd_result_ = lite6_interfaces::msg::ControllerStatus::ACK_NONE;

  system_state_ = SystemState::NORMAL;
  last_reported_state_ = SystemState::NORMAL;
  tau_applied_prev_.setZero();
  imp_ref_kinematics_cached_ = false;
  reset_momentum_observer();
  cmd_ring_.discard_all();
  cmd_gate_.store(0ULL, std::memory_order_release);
  controller_epoch_atomic_.store(controller_epoch_, std::memory_order_release);
  mode_epoch_atomic_.store(mode_epoch_, std::memory_order_release);
  current_mode_atomic_.store(
    lite6_interfaces::msg::ControllerStatus::MODE_NORMAL, std::memory_order_release);
  last_cmd_ack_code_.store(last_cmd_code_, std::memory_order_relaxed);
  last_cmd_ack_result_.store(last_cmd_result_, std::memory_order_relaxed);
  last_cmd_ack_mode_.store(
    lite6_interfaces::msg::ControllerStatus::MODE_NORMAL, std::memory_order_relaxed);
  last_cmd_ack_mode_epoch_.store(mode_epoch_, std::memory_order_relaxed);
  last_cmd_ack_seq_.store(0ULL, std::memory_order_release);

  // Force a fresh NORMAL observation on the first successful update.
  // Never infer a mode from the last command sent before reactivation.
  status_publish_counter_ = 0;
  status_publish_pending_ = true;
  publish_shadow_shm(&q_, true);
  trajectory_enabled_.store(true, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Lite6CTCController::on_deactivate(const rclcpp_lifecycle::State &)
{
  trajectory_enabled_.store(false, std::memory_order_release);
  current_mode_atomic_.store(
    lite6_interfaces::msg::ControllerStatus::MODE_UNKNOWN, std::memory_order_release);
  publish_shadow_shm(nullptr, false);
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  if (active_trajectory_) {
    auto expected = TrajectoryResult::RUNNING;
    active_trajectory_->result.compare_exchange_strong(expected, TrajectoryResult::DEACTIVATED);
  }
  if (active_impedance_trajectory_) {
    auto expected = ImpedanceTrajectoryResult::RUNNING;
    active_impedance_trajectory_->result.compare_exchange_strong(expected, ImpedanceTrajectoryResult::DEACTIVATED);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type Lite6CTCController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  current_execution_ = trajectory_buffer_.readFromRT()->get();
  rt_seen_execution_.store(current_execution_, std::memory_order_release);
  current_impedance_execution_ = impedance_trajectory_buffer_.readFromRT()->get();
  rt_seen_impedance_execution_.store(current_impedance_execution_, std::memory_order_release);
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

  // Update the sensorless Generalized Momentum Observer using the torque applied in the
  // previous cycle before generating this cycle's new reference or control command.
  gmo_gravity_cached_this_cycle_ = false;
  gmo_tcp_cached_this_cycle_ = false;
  update_momentum_observer(dt);

  // Process ordered edge-triggered ControllerCommand requests and emergency triggers.
  process_controller_commands(dt);

  update_trajectory_reference(time, period);
  update_impedance_trajectory_reference(time, period);

  check_compliance_safety_limits(dt);
  step_compliance_stopping(dt);
  compliance_blocks_trajectory_.store(
    compliance_active() ||
    system_state_ == SystemState::COMPLIANCE_STOPPING ||
    impedance_trajectory_active());
  check_trajectory_path();
  check_impedance_trajectory_path();

  Eigen::Map<Vec6d> map_q_d(reference_interfaces_.data());
  Eigen::Map<Vec6d> map_dq_d(reference_interfaces_.data() + n);
  Eigen::Map<Vec6d> map_ddq_d(reference_interfaces_.data() + 2 * n);

  if (system_state_ == SystemState::NORMAL || system_state_ == SystemState::SYS_ID) {
      q_target_ = map_q_d; dq_target_ = map_dq_d; ddq_target_ = map_ddq_d;
  } else if (impedance_trajectory_active()) {
      q_target_ = q_imp_ref_; dq_target_ = dq_imp_ref_; ddq_target_ = ddq_imp_ref_;
  } else if (compliance_active()) {
      q_target_=compliance_q_; dq_target_.setZero(); ddq_target_.setZero();
      if(system_state_==SystemState::TEACH) q_target_=q_;
  } else {
      step_deceleration_tracker(dt);
      q_target_ = estop_q_;
      dq_target_ = estop_dq_;
  }

  // 3. Error Calculation
  e_ = q_target_ - q_;
  de_ = dq_target_ - dq_;
  if (!e_.allFinite() || !de_.allFinite() || !ddq_target_.allFinite())
    return fail_controller(TrajectoryResult::INVALID_COMMAND);

  // 4. Branching Control Law
  if (impedance_trajectory_active()) {
      compute_impedance_trajectory(dt);
  } else if (compliance_active()) {
      compute_compliance(dt);
  } else if (system_state_ == SystemState::SYS_ID) {
      // Identification must contain neither friction nor learned bias compensation.
      integral_effort_.setZero();
      // Rotor inertia is known in every mode; only friction is unidentified.
      a_d_ = ddq_target_ + kp_.cwiseProduct(e_) + kv_.cwiseProduct(de_);
      dynamics_ddq_.head(6) = a_d_;
      pinocchio::rnea(pin_model_, pin_data_, dynamics_q_, dynamics_dq_, dynamics_ddq_);
      for(size_t i = 0; i < n; ++i) {
          tau_cmd_[i] = pin_data_.tau[i] + (armature_[i] * a_d_[i]);
      }
  } 
  else {
      a_d_ = ddq_target_ + kp_.cwiseProduct(e_) + kv_.cwiseProduct(de_);
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
      if (system_state_ != SystemState::SYS_ID && !compliance_active() &&
          !impedance_trajectory_active() && ki_[i] > 0.0) {
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
    tau_applied_prev_[static_cast<Eigen::Index>(i)] = tau_cmd_[static_cast<Eigen::Index>(i)];
  }
  // No success is published until this cycle's feedback and command writes pass.
  finish_trajectory_cycle(dt);
  finish_impedance_trajectory_cycle(dt);

  // 6. Write this cycle's actual reference directly to the shared-memory register.
  publish_shadow_shm(&q_target_, true);

  // 7. Publish the actual internal CTC mode, observed by the Supervisor.
  // The mode is mapped explicitly: SystemState is NOT the wire enum.
  // Publish periodically even if unchanged (heartbeat), and retry every
  // update cycle after a mode change until the nonblocking lock succeeds.
  ++status_publish_counter_;
  if (status_publish_pending_ || system_state_ != last_reported_state_ ||
      status_publish_counter_ >= status_publish_rate_divider_) {
    if (rt_status_pub_->trylock()) {
      auto & status = rt_status_pub_->msg_;
      status.stamp = time;
      const auto * execution = current_execution_;
      const auto * imp_execution = current_impedance_execution_;
      status.trajectory_start = builtin_interfaces::msg::Time{};
      status.trajectory_id.fill(0);
      if (execution && execution->started) {
        status.trajectory_start = rclcpp::Time(execution->start_time_ns);
        status.trajectory_id = execution->goal->get_goal_id();
      } else if (imp_execution && imp_execution->started) {
        status.trajectory_start = rclcpp::Time(imp_execution->start_time_ns);
        status.trajectory_id = imp_execution->goal->get_goal_id();
      }
      status.mode = controller_mode_for_state(system_state_);
      status.estop_reason = estop_reason_;
      status.estop_joint = estop_joint_;
      status.estop_value = estop_value_;
      status.estop_limit = estop_limit_;
      for (size_t i = 0; i < 6; ++i) {
        status.external_joint_torque[i] = gmo_residual_tau_[static_cast<Eigen::Index>(i)];
        status.external_cartesian_wrench[i] = gmo_external_wrench_[static_cast<Eigen::Index>(i)];
      }
      status.external_contact_detected = gmo_contact_detected_;
      ++status_seq_;
      status.status_seq = status_seq_;
      status.controller_epoch = controller_epoch_;
      status.mode_epoch = mode_epoch_;
      status.last_cmd_seq = last_cmd_seq_;
      status.last_cmd_code = last_cmd_code_;
      status.last_cmd_result = last_cmd_result_;
      status.rigid_trajectory_active = rigid_trajectory_active() || rigid_cancel_braking_;
      status.payload_mass = payload_mass_;
      status.payload_com_tcp = payload_com_tcp_;
      rt_status_pub_->unlockAndPublish();
      last_reported_state_ = system_state_;
      status_publish_counter_ = 0;
      status_publish_pending_ = false;
    }
  }

  return controller_interface::return_type::OK;
}

void Lite6CTCController::publish_shadow_shm(const Vec6d * target, bool valid)
{
  if (!state_shm_.valid()) {return;}
  lite6_interfaces::shm::ShadowStatePod pod{};
  if (target) {
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      pod.position[i] = (*target)[static_cast<Eigen::Index>(i)];
    }
  }
  pod.stamp_ns = lite6_interfaces::shm::steady_now_ns();
  pod.controller_epoch = shadow_epoch_;
  pod.valid = valid ? 1 : 0;
  state_shm_->shadow_state.write(pod);
}

bool Lite6CTCController::configure_momentum_observer()
{
  const auto node = get_node();
  gmo_enabled_ = node->get_parameter("gmo.enabled").as_bool();
  const auto gains = node->get_parameter("gmo.observer_gains").as_double_array();
  gmo_jacobian_damping_ = node->get_parameter("gmo.jacobian_damping").as_double();
  gmo_fric_trans_speed_ = node->get_parameter("gmo.friction_transition_speed").as_double();
  gmo_contact_force_threshold_ = node->get_parameter("gmo.contact_force_threshold").as_double();

  if (gains.size() != 6 ||
      !std::all_of(gains.begin(), gains.end(), [](double v) { return std::isfinite(v) && v > 0.0; }) ||
      !std::isfinite(gmo_jacobian_damping_) || gmo_jacobian_damping_ <= 0.0 ||
      !std::isfinite(gmo_fric_trans_speed_) || gmo_fric_trans_speed_ <= 0.0 ||
      !std::isfinite(gmo_contact_force_threshold_) || gmo_contact_force_threshold_ < 0.0) {
    RCLCPP_ERROR(node->get_logger(), "Invalid gmo observer parameter configuration");
    return false;
  }
  for (size_t i = 0; i < 6; ++i) {
    gmo_gains_[i] = gains[i];
  }
  compliance_frame_ = pin_model_.getFrameId("link_eef");
  if (compliance_frame_ >= static_cast<pinocchio::FrameIndex>(pin_model_.nframes)) {
    RCLCPP_ERROR(node->get_logger(), "Missing link_eef frame for momentum observer / compliance");
    return false;
  }
  tau_applied_prev_.setZero();
  reset_momentum_observer();
  return true;
}

void Lite6CTCController::reset_momentum_observer()
{
  gmo_initialized_ = false;
  gmo_gravity_cached_this_cycle_ = false;
  gmo_tcp_cached_this_cycle_ = false;
  gmo_p_integral_.setZero();
  gmo_residual_tau_.setZero();
  gmo_external_wrench_.setZero();
  gmo_contact_detected_ = false;
}

void Lite6CTCController::update_momentum_observer(double dt)
{
  if (!gmo_enabled_) {
    gmo_residual_tau_.setZero();
    gmo_external_wrench_.setZero();
    gmo_contact_detected_ = false;
    return;
  }
  if (!(dt > 0.0) || dt > 0.1) {
    return;
  }

  pinocchio::crba(pin_model_, pin_data_gmo_, dynamics_q_);
  pin_data_gmo_.M.triangularView<Eigen::StrictlyLower>() =
    pin_data_gmo_.M.transpose().triangularView<Eigen::StrictlyLower>();
  Eigen::Matrix<double, 6, 1> p_a = pin_data_gmo_.M.topRows<6>() * dynamics_dq_;
  for (int i = 0; i < 6; ++i) {
    p_a[i] += armature_[i] * dynamics_dq_[i];
  }

  if (!gmo_initialized_) {
    gmo_p_integral_ = p_a;
    gmo_residual_tau_.setZero();
    gmo_external_wrench_.setZero();
    gmo_contact_detected_ = false;
    gmo_initialized_ = true;
    return;
  }

  pinocchio::computeCoriolisMatrix(pin_model_, pin_data_gmo_, dynamics_q_, dynamics_dq_);
  const Eigen::Matrix<double, 6, 1> ct_dq =
    (pin_data_gmo_.C.transpose() * dynamics_dq_).head<6>();

  pinocchio::computeGeneralizedGravity(pin_model_, pin_data_gmo_, dynamics_q_);
  const Eigen::Matrix<double, 6, 1> g_a = pin_data_gmo_.g.head<6>();

  Eigen::Matrix<double, 6, 1> tau_f_obs;
  for (int i = 0; i < 6; ++i) {
    tau_f_obs[i] = friction_v_[i] * dq_[i] +
      friction_c_[i] * std::tanh(dq_[i] / gmo_fric_trans_speed_);
  }

  const Eigen::Matrix<double, 6, 1> beta = tau_applied_prev_ - tau_f_obs - g_a + ct_dq;
  for (int i = 0; i < 6; ++i) {
    const double ko = gmo_gains_[static_cast<size_t>(i)];
    const double r_i = (ko / (1.0 + ko * dt)) *
      (p_a[i] - gmo_p_integral_[i] - dt * beta[i]);
    gmo_residual_tau_[i] = r_i;
    gmo_p_integral_[i] += dt * (beta[i] + r_i);
  }

  if (!gmo_residual_tau_.allFinite()) {
    reset_momentum_observer();
    return;
  }
  gmo_gravity_cached_this_cycle_ = true;

  // Only compute the 6x8 TCP Jacobian, 6x6 LDLT wrench solve, and contact flag
  // when Cartesian compliance/trajectory needs them or this cycle will publish status.
  const bool need_tcp_wrench =
    system_state_ == SystemState::CARTESIAN_IMPEDANCE ||
    system_state_ == SystemState::CARTESIAN_IMPEDANCE_TRAJECTORY ||
    status_publish_pending_ ||
    system_state_ != last_reported_state_ ||
    (status_publish_counter_ + 1 >= status_publish_rate_divider_);
  if (!need_tcp_wrench) {
    return;
  }

  pinocchio::computeJointJacobians(pin_model_, pin_data_gmo_, dynamics_q_);
  pinocchio::updateFramePlacement(pin_model_, pin_data_gmo_, compliance_frame_);
  gmo_jacobian_.setZero();
  pinocchio::getFrameJacobian(
    pin_model_, pin_data_gmo_, compliance_frame_,
    pinocchio::LOCAL_WORLD_ALIGNED, gmo_jacobian_);
  const Eigen::Matrix<double, 6, 6> j_arm = gmo_jacobian_.leftCols<6>();
  Eigen::Matrix<double, 6, 6> jj_t = j_arm * j_arm.transpose();
  jj_t.diagonal().array() += gmo_jacobian_damping_ * gmo_jacobian_damping_;
  gmo_external_wrench_ = jj_t.ldlt().solve(j_arm * gmo_residual_tau_);

  if (!gmo_external_wrench_.allFinite()) {
    reset_momentum_observer();
    return;
  }
  gmo_contact_detected_ =
    gmo_external_wrench_.head<3>().norm() >= gmo_contact_force_threshold_;
  gmo_tcp_cached_this_cycle_ = true;
}

void Lite6CTCController::read_cached_tcp_kinematics(
  const pinocchio::SE3 * & out_pose, const Mat6x8d * & out_jacobian)
{
  if (gmo_tcp_cached_this_cycle_) {
    out_pose = &pin_data_gmo_.oMf[compliance_frame_];
    out_jacobian = &gmo_jacobian_;
    return;
  }
  pinocchio::computeJointJacobians(pin_model_, pin_data_, dynamics_q_);
  pinocchio::updateFramePlacement(pin_model_, pin_data_, compliance_frame_);
  compliance_jacobian_.setZero();
  pinocchio::getFrameJacobian(
    pin_model_, pin_data_, compliance_frame_,
    pinocchio::LOCAL_WORLD_ALIGNED, compliance_jacobian_);
  out_pose = &pin_data_.oMf[compliance_frame_];
  out_jacobian = &compliance_jacobian_;
}

void Lite6CTCController::add_passive_friction_ff(
  double fv_scale, double fc_scale, double v_trans, double quiet_speed,
  const Vec6d & dq_coulomb,
  Vec6d & tau_out) const
{
  const size_t n = joint_names_.size();
  for (size_t i = 0; i < n; ++i) {
    const Eigen::Index idx = static_cast<Eigen::Index>(i);
    const double viscous = fv_scale * friction_v_[idx] * dq_[idx];
    const double dq_c = dq_coulomb[idx];
    double coulomb_gate = 1.0;
    if (quiet_speed > 0.0) {
      const double ramp = std::clamp((std::abs(dq_c) - quiet_speed) / quiet_speed, 0.0, 1.0);
      coulomb_gate = ramp * ramp * (3.0 - 2.0 * ramp);
    }
    const double coulomb = fc_scale * coulomb_gate *
      friction_c_[idx] * std::tanh(dq_c / v_trans);
    tau_out[idx] += viscous + coulomb;
  }
}

bool Lite6CTCController::check_external_torque_limit(
  double dt, const std::array<double, 6> & limits, double timeout,
  double & accum_duration, int & out_joint, double & out_val, double & out_limit) const
{
  if (!gmo_enabled_) {
    accum_duration = 0.0;
    return false;
  }
  int worst_joint = -1;
  double worst_ratio = 0.0;
  for (int i = 0; i < 6; ++i) {
    const double lim = limits[static_cast<size_t>(i)];
    const double mag = std::abs(gmo_residual_tau_[i]);
    if (mag > lim) {
      const double ratio = mag / lim;
      if (ratio > worst_ratio) {
        worst_ratio = ratio;
        worst_joint = i;
      }
    }
  }
  if (worst_joint >= 0) {
    accum_duration += dt;
    if (accum_duration >= timeout) {
      out_joint = worst_joint;
      out_val = std::abs(gmo_residual_tau_[worst_joint]);
      out_limit = limits[static_cast<size_t>(worst_joint)];
      return true;
    }
  } else {
    accum_duration = 0.0;
  }
  return false;
}

uint8_t Lite6CTCController::controller_mode_for_state(SystemState state)
{
  using CS = lite6_interfaces::msg::ControllerStatus;
  switch (state) {
    case SystemState::NORMAL: return CS::MODE_NORMAL;
    case SystemState::ESTOP: return CS::MODE_ESTOP;
    case SystemState::SYS_ID: return CS::MODE_SYSID;
    case SystemState::JOINT_IMPEDANCE: return CS::MODE_JOINT_IMPEDANCE;
    case SystemState::CARTESIAN_IMPEDANCE: return CS::MODE_CARTESIAN_IMPEDANCE;
    case SystemState::TEACH: return CS::MODE_TEACH;
    case SystemState::COMPLIANCE_STOPPING: return CS::MODE_COMPLIANCE_STOPPING;
    case SystemState::CARTESIAN_IMPEDANCE_TRAJECTORY: return CS::MODE_CARTESIAN_IMPEDANCE_TRAJECTORY;
    default: return CS::MODE_UNKNOWN;
  }
}

uint8_t Lite6CTCController::current_mode_code() const
{
  return controller_mode_for_state(system_state_);
}

bool Lite6CTCController::rigid_trajectory_active() const
{
  return current_execution_ != nullptr &&
    current_execution_->result.load(std::memory_order_acquire) == TrajectoryResult::RUNNING;
}

void Lite6CTCController::transition_system_state(SystemState next_state)
{
  if (system_state_ != next_state) {
    system_state_ = next_state;
    ++mode_epoch_;
    mode_epoch_atomic_.store(mode_epoch_, std::memory_order_release);
    current_mode_atomic_.store(controller_mode_for_state(next_state), std::memory_order_release);
    status_publish_pending_ = true;
    if (next_state != SystemState::COMPLIANCE_STOPPING) {
      compliance_stopping_elapsed_ = 0.0;
    }
  }
}

void Lite6CTCController::begin_deceleration_tracker()
{
  if (q_.allFinite() && dq_.allFinite()) {
    estop_q_ = q_;
    estop_dq_ = dq_;
  }
  ddq_target_.setZero();
}

bool Lite6CTCController::step_deceleration_tracker(double dt)
{
  const size_t n = joint_names_.size();
  for (size_t i = 0; i < n; ++i) {
    if (std::abs(estop_dq_[i]) > 1e-4) {
      const double sign = (estop_dq_[i] > 0.0) ? 1.0 : -1.0;
      ddq_target_[i] = -sign * estop_decel_[i];
      const double next_dq = estop_dq_[i] + ddq_target_[i] * dt;
      if ((sign > 0.0 && next_dq <= 0.0) || (sign < 0.0 && next_dq >= 0.0)) {
        const double t_zero = std::abs(estop_dq_[i] / estop_decel_[i]);
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
  return estop_dq_.cwiseAbs().maxCoeff() < 1e-4;
}

void Lite6CTCController::configure_command_service()
{
  command_callback_group_ = get_node()->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);
  command_service_ = get_node()->create_service<lite6_interfaces::srv::ControllerCommand>(
    "~/command",
    [this](
      const std::shared_ptr<lite6_interfaces::srv::ControllerCommand::Request> req,
      std::shared_ptr<lite6_interfaces::srv::ControllerCommand::Response> resp)
    {
      using Cmd = lite6_interfaces::srv::ControllerCommand::Request;
      using Resp = lite6_interfaces::srv::ControllerCommand::Response;
      using CS = lite6_interfaces::msg::ControllerStatus;
      // Captured before any lock: a later CMD_ESTOP supersedes this request.
      const uint64_t request_estop_generation =
        estop_generation_.load(std::memory_order_acquire);

      auto fill_snapshot = [this, &resp]() {
        resp->controller_epoch = controller_epoch_atomic_.load(std::memory_order_acquire);
        resp->mode_epoch = mode_epoch_atomic_.load(std::memory_order_acquire);
        resp->current_mode = current_mode_atomic_.load(std::memory_order_acquire);
      };

      if (!req->reason.empty()) {
        RCLCPP_DEBUG(
          get_node()->get_logger(), "ControllerCommand(%u): %s",
          static_cast<unsigned>(req->command), req->reason.c_str());
      }

      if (!lite6_interfaces::valid_controller_command(*req)) {
        resp->accepted = false;
        resp->ack_result = Resp::ACK_REJECTED_INVALID_ARG;
        resp->cmd_seq = 0;
        fill_snapshot();
        resp->message = "Invalid ControllerCommand code or out-of-bounds parameter";
        return;
      }

      if (!trajectory_enabled_.load(std::memory_order_acquire)) {
        resp->accepted = false;
        resp->ack_result = Resp::ACK_REJECTED_STATE;
        resp->cmd_seq = 0;
        fill_snapshot();
        resp->message = "CTC controller is not active";
        return;
      }

      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(lite6_interfaces::contract::kControllerCmdWaitTimeoutMs);

      if (req->command == Cmd::CMD_ESTOP) {
        // E-STOP fast path: no epoch check, no command mutex, no ring slot. The RT
        // loop latches it after every command received before this request.
        // The request is never withdrawn; the wait below only reports the outcome.
        emergency_pending_.store(true, std::memory_order_release);
        estop_generation_.fetch_add(1ULL, std::memory_order_acq_rel);
        resp->cmd_seq = command_sequence_.fetch_add(1ULL, std::memory_order_acq_rel) + 1ULL;
        while (std::chrono::steady_clock::now() < deadline &&
               trajectory_enabled_.load(std::memory_order_acquire))
        {
          if (current_mode_atomic_.load(std::memory_order_acquire) == CS::MODE_ESTOP) {
            resp->accepted = true;
            resp->ack_result = Resp::ACK_APPLIED;
            fill_snapshot();
            resp->message = "E-STOP latched";
            return;
          }
          std::this_thread::sleep_for(std::chrono::microseconds(250));
        }
        resp->accepted = false;
        resp->ack_result = Resp::ACK_RT_TIMEOUT;
        fill_snapshot();
        resp->message =
          "E-STOP not yet observed from the RT loop; the latch request stays pending";
        return;
      }

      std::lock_guard<std::mutex> lock(cmd_service_mutex_);

      const uint64_t active_epoch = controller_epoch_atomic_.load(std::memory_order_acquire);
      if (req->expected_epoch > 0 && req->expected_epoch != active_epoch) {
        resp->accepted = false;
        resp->ack_result = Resp::ACK_REJECTED_EPOCH;
        resp->cmd_seq = 0;
        fill_snapshot();
        resp->message = "Controller epoch mismatch";
        return;
      }

      PendingControllerCommand pending{};
      pending.cmd_seq = command_sequence_.fetch_add(1ULL, std::memory_order_acq_rel) + 1ULL;
      pending.estop_generation = request_estop_generation;
      pending.command = req->command;
      pending.compliance_recapture = req->compliance.recapture;
      pending.joint_stiffness = req->compliance.joint_stiffness;
      pending.joint_damping = req->compliance.joint_damping;
      pending.translation_stiffness = req->compliance.translation_stiffness;
      pending.translation_damping = req->compliance.translation_damping;
      pending.rotation_stiffness = req->compliance.rotation_stiffness;
      pending.rotation_damping = req->compliance.rotation_damping;
      pending.sensitivity = req->compliance.sensitivity;
      pending.nullspace_stiffness = req->compliance.nullspace_stiffness;
      pending.nullspace_damping = req->compliance.nullspace_damping;
      pending.payload_mass = req->payload_mass;
      for (size_t i = 0; i < 3; ++i) {
        pending.payload_com_tcp[i] = req->payload_com_tcp[i];
      }

      // Open the claim gate before the RT loop can observe the command.
      cmd_gate_.store(pending.cmd_seq, std::memory_order_release);
      if (!cmd_ring_.push(pending)) {
        cmd_gate_.store(0ULL, std::memory_order_release);
        resp->accepted = false;
        resp->ack_result = Resp::ACK_RT_TIMEOUT;
        resp->cmd_seq = 0;
        fill_snapshot();
        resp->message = "RT command ring buffer full; command not queued";
        return;
      }

      auto fill_ack = [&]() {
        const uint8_t ack = last_cmd_ack_result_.load(std::memory_order_relaxed);
        resp->accepted = (ack == Resp::ACK_APPLIED);
        resp->ack_result = ack;
        resp->cmd_seq = pending.cmd_seq;
        resp->controller_epoch = active_epoch;
        resp->mode_epoch = last_cmd_ack_mode_epoch_.load(std::memory_order_relaxed);
        resp->current_mode = last_cmd_ack_mode_.load(std::memory_order_relaxed);
        resp->message = resp->accepted ? "Applied" : "Rejected by RT controller";
      };
      auto acked = [&]() {
        return last_cmd_ack_seq_.load(std::memory_order_acquire) >= pending.cmd_seq;
      };

      bool deactivated = false;
      while (std::chrono::steady_clock::now() < deadline) {
        if (acked()) {
          fill_ack();
          return;
        }
        if (!trajectory_enabled_.load(std::memory_order_acquire)) {
          deactivated = true;
          break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(250));
      }

      // Try to withdraw the command. Success guarantees the RT loop will drop it.
      uint64_t expected_gate = pending.cmd_seq;
      if (cmd_gate_.compare_exchange_strong(
          expected_gate, 0ULL, std::memory_order_acq_rel, std::memory_order_acquire))
      {
        resp->accepted = false;
        resp->ack_result = deactivated ? Resp::ACK_REJECTED_STATE : Resp::ACK_RT_TIMEOUT;
        resp->cmd_seq = pending.cmd_seq;
        fill_snapshot();
        resp->message = deactivated ?
          "CTC controller deactivated before applying the command; command withdrawn" :
          "Timed out waiting for 1 kHz RT controller handshake; command withdrawn (not applied)";
        return;
      }

      // The RT loop already claimed the command and finishes it within one cycle.
      const auto claim_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
      while (std::chrono::steady_clock::now() < claim_deadline) {
        if (acked()) {
          fill_ack();
          return;
        }
        if (!trajectory_enabled_.load(std::memory_order_acquire)) {
          break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }
      resp->accepted = false;
      resp->ack_result = Resp::ACK_REJECTED_STATE;
      resp->cmd_seq = pending.cmd_seq;
      fill_snapshot();
      resp->message = "RT loop claimed the command but stopped before acknowledging it";
    },
    rclcpp::ServicesQoS(), command_callback_group_);
}

void Lite6CTCController::process_controller_commands(double /*dt*/)
{
  using CS = lite6_interfaces::msg::ControllerStatus;
  using Resp = lite6_interfaces::srv::ControllerCommand::Response;

  PendingControllerCommand batch[32];
  const size_t count = cmd_ring_.pop_batch(batch, 32);
  const uint64_t current_estop_generation = estop_generation_.load(std::memory_order_acquire);

  for (size_t i = 0; i < count; ++i) {
    const auto & cmd = batch[i];
    uint8_t res = Resp::ACK_RT_TIMEOUT;
    uint64_t expected_gate = cmd.cmd_seq;
    if (!cmd_gate_.compare_exchange_strong(
        expected_gate, 0ULL, std::memory_order_acq_rel, std::memory_order_acquire))
    {
      // Withdrawn by its timed-out service call: never applied.
      res = Resp::ACK_RT_TIMEOUT;
    } else if (cmd.estop_generation != current_estop_generation) {
      // Received before a later CMD_ESTOP request: superseded, never applied after it.
      res = Resp::ACK_REJECTED_STATE;
    } else {
      res = apply_controller_command(cmd);
    }
    last_cmd_seq_ = cmd.cmd_seq;
    last_cmd_code_ = cmd.command;
    last_cmd_result_ = res;
    last_cmd_ack_code_.store(cmd.command, std::memory_order_relaxed);
    last_cmd_ack_result_.store(res, std::memory_order_relaxed);
    last_cmd_ack_mode_.store(current_mode_code(), std::memory_order_relaxed);
    last_cmd_ack_mode_epoch_.store(mode_epoch_, std::memory_order_relaxed);
    last_cmd_ack_seq_.store(cmd.cmd_seq, std::memory_order_release);
    status_publish_pending_ = true;
  }

  // Emergency latching runs after the batch, so an E-STOP always wins over any
  // command processed in the same cycle. The CMD_ESTOP service sets this flag
  // before bumping estop_generation_, so commands received earlier are either
  // processed in this batch (and overridden here) or rejected as superseded.
  const bool emerg = emergency_pending_.exchange(false, std::memory_order_acq_rel);
  if (!emerg) {
    return;
  }
  if (compliance_saturation_estop_pending_) {
    latch_estop_reason(
      CS::ESTOP_COMPLIANCE_SATURATION,
      pending_estop_joint_, pending_estop_value_, pending_estop_limit_);
    compliance_saturation_estop_pending_ = false;
  } else {
    latch_estop_reason(CS::ESTOP_EXTERNAL);
  }
  if (system_state_ != SystemState::ESTOP) {
    begin_deceleration_tracker();
    transition_system_state(SystemState::ESTOP);
  }
  status_publish_pending_ = true;
}

uint8_t Lite6CTCController::apply_controller_command(const PendingControllerCommand & cmd)
{
  using Cmd = lite6_interfaces::srv::ControllerCommand::Request;
  using Resp = lite6_interfaces::srv::ControllerCommand::Response;
  using CS = lite6_interfaces::msg::ControllerStatus;

  const bool rigid_busy = rigid_trajectory_active() || rigid_cancel_braking_;
  const bool imp_traj_busy =
    (current_impedance_execution_ != nullptr &&
     current_impedance_execution_->result.load(std::memory_order_acquire) == ImpedanceTrajectoryResult::RUNNING) ||
    impedance_trajectory_active();
  const bool stopping_busy = (system_state_ == SystemState::COMPLIANCE_STOPPING);
  const bool busy = rigid_busy || imp_traj_busy || stopping_busy;
  const double max_speed = dq_.cwiseAbs().maxCoeff();

  auto populate_compliance_settings = [&cmd](lite6_interfaces::msg::ComplianceSettings & out) {
    out.joint_stiffness = cmd.joint_stiffness;
    out.joint_damping = cmd.joint_damping;
    out.translation_stiffness = cmd.translation_stiffness;
    out.translation_damping = cmd.translation_damping;
    out.rotation_stiffness = cmd.rotation_stiffness;
    out.rotation_damping = cmd.rotation_damping;
    out.sensitivity = cmd.sensitivity;
    out.recapture = cmd.compliance_recapture;
    out.nullspace_stiffness = cmd.nullspace_stiffness;
    out.nullspace_damping = cmd.nullspace_damping;
  };

  auto within_hard_limits = [this]() -> bool {
    for (int i = 0; i < 6; ++i) {
      if (q_[i] < pin_model_.lowerPositionLimit[i] - compliance_hard_limit_tolerance_ ||
          q_[i] > pin_model_.upperPositionLimit[i] + compliance_hard_limit_tolerance_) {
        return false;
      }
    }
    return true;
  };

  switch (cmd.command) {
    case Cmd::CMD_ESTOP:
      latch_estop_reason(CS::ESTOP_EXTERNAL);
      if (system_state_ != SystemState::ESTOP) {
        begin_deceleration_tracker();
        transition_system_state(SystemState::ESTOP);
      }
      return Resp::ACK_APPLIED;

    case Cmd::CMD_CLEAR_ESTOP:
      if (busy) return Resp::ACK_REJECTED_BUSY;
      if (system_state_ != SystemState::ESTOP) return Resp::ACK_REJECTED_STATE;
      if (max_speed > lite6_interfaces::contract::kClearEstopMaxSpeedRadPerSec) {
        return Resp::ACK_REJECTED_SPEED;
      }
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        reference_interfaces_[i] = q_[i];
        reference_interfaces_[joint_names_.size() + i] = 0.0;
        reference_interfaces_[2 * joint_names_.size() + i] = 0.0;
      }
      q_target_ = q_;
      dq_target_.setZero();
      ddq_target_.setZero();
      estop_q_ = q_;
      estop_dq_.setZero();
      clear_estop_reason();
      transition_system_state(SystemState::NORMAL);
      return Resp::ACK_APPLIED;

    case Cmd::CMD_ENTER_NORMAL:
      if (busy) return Resp::ACK_REJECTED_BUSY;
      if (system_state_ == SystemState::ESTOP) return Resp::ACK_REJECTED_STATE;
      if (system_state_ == SystemState::NORMAL) return Resp::ACK_APPLIED;
      if (system_state_ == SystemState::SYS_ID) {
        if (max_speed > lite6_interfaces::contract::kControllerEntryMaxSpeedRadPerSec) {
          return Resp::ACK_REJECTED_SPEED;
        }
        for (size_t i = 0; i < joint_names_.size(); ++i) {
          reference_interfaces_[i] = q_[i];
          reference_interfaces_[joint_names_.size() + i] = 0.0;
          reference_interfaces_[2 * joint_names_.size() + i] = 0.0;
        }
        transition_system_state(SystemState::NORMAL);
        return Resp::ACK_APPLIED;
      }
      if (compliance_active()) {
        begin_deceleration_tracker();
        integral_effort_.setZero();
        compliance_stopping_elapsed_ = 0.0;
        transition_system_state(SystemState::COMPLIANCE_STOPPING);
        return Resp::ACK_APPLIED;
      }
      return Resp::ACK_REJECTED_STATE;

    case Cmd::CMD_ENTER_SYSID:
      if (busy) return Resp::ACK_REJECTED_BUSY;
      if (system_state_ != SystemState::NORMAL) return Resp::ACK_REJECTED_STATE;
      if (max_speed > lite6_interfaces::contract::kControllerEntryMaxSpeedRadPerSec) {
        return Resp::ACK_REJECTED_SPEED;
      }
      if (!within_hard_limits()) return Resp::ACK_REJECTED_LIMIT;
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        reference_interfaces_[i] = q_[i];
        reference_interfaces_[joint_names_.size() + i] = 0.0;
        reference_interfaces_[2 * joint_names_.size() + i] = 0.0;
      }
      transition_system_state(SystemState::SYS_ID);
      return Resp::ACK_APPLIED;

    case Cmd::CMD_ENTER_JOINT_IMPEDANCE:
    case Cmd::CMD_ENTER_CARTESIAN_IMPEDANCE:
    case Cmd::CMD_ENTER_TEACH: {
      if (busy) return Resp::ACK_REJECTED_BUSY;
      if (system_state_ != SystemState::NORMAL && !compliance_active()) {
        return Resp::ACK_REJECTED_STATE;
      }
      const SystemState desired =
        cmd.command == Cmd::CMD_ENTER_JOINT_IMPEDANCE ? SystemState::JOINT_IMPEDANCE :
        (cmd.command == Cmd::CMD_ENTER_CARTESIAN_IMPEDANCE ? SystemState::CARTESIAN_IMPEDANCE :
         SystemState::TEACH);
      const bool entering = (desired != system_state_);
      if (entering && max_speed > lite6_interfaces::contract::kControllerEntryMaxSpeedRadPerSec) {
        return Resp::ACK_REJECTED_SPEED;
      }
      if (!within_hard_limits()) return Resp::ACK_REJECTED_LIMIT;
      populate_compliance_settings(compliance_settings_);
      if (entering || cmd.compliance_recapture) {
        compliance_q_ = q_;
        if (gmo_tcp_cached_this_cycle_) {
          compliance_pose_ = pin_data_gmo_.oMf[compliance_frame_];
        } else {
          pinocchio::forwardKinematics(pin_model_, pin_data_, dynamics_q_);
          compliance_pose_ = pinocchio::updateFramePlacement(pin_model_, pin_data_, compliance_frame_);
        }
      }
      if (entering) {
        compliance_filtered_ = compliance_settings_;
        compliance_last_tau_ = tau_cmd_;
        compliance_saturation_time_ = 0.0;
        compliance_external_torque_time_ = 0.0;
        compliance_saturation_estop_pending_ = false;
        integral_effort_.setZero();
      }
      transition_system_state(desired);
      return Resp::ACK_APPLIED;
    }

    case Cmd::CMD_UPDATE_COMPLIANCE_SETTINGS:
      if (busy) return Resp::ACK_REJECTED_BUSY;
      if (!compliance_active()) return Resp::ACK_REJECTED_STATE;
      populate_compliance_settings(compliance_settings_);
      if (cmd.compliance_recapture) {
        compliance_q_ = q_;
        if (gmo_tcp_cached_this_cycle_) {
          compliance_pose_ = pin_data_gmo_.oMf[compliance_frame_];
        } else {
          pinocchio::forwardKinematics(pin_model_, pin_data_, dynamics_q_);
          compliance_pose_ = pinocchio::updateFramePlacement(pin_model_, pin_data_, compliance_frame_);
        }
      }
      return Resp::ACK_APPLIED;

    case Cmd::CMD_SET_PAYLOAD:
      if (busy) return Resp::ACK_REJECTED_BUSY;
      payload_mass_ = cmd.payload_mass;
      payload_com_tcp_ = {cmd.payload_com_tcp[0], cmd.payload_com_tcp[1], cmd.payload_com_tcp[2]};
      return Resp::ACK_APPLIED;

    case Cmd::CMD_TARE_GMO:
      if (busy) return Resp::ACK_REJECTED_BUSY;
      if (system_state_ == SystemState::ESTOP) return Resp::ACK_REJECTED_STATE;
      if (max_speed > lite6_interfaces::contract::kControllerEntryMaxSpeedRadPerSec) {
        return Resp::ACK_REJECTED_SPEED;
      }
      reset_momentum_observer();
      return Resp::ACK_APPLIED;

    default:
      return Resp::ACK_REJECTED_INVALID_ARG;
  }
}

}  // namespace lite6_controllers

PLUGINLIB_EXPORT_CLASS(lite6_controllers::Lite6CTCController, controller_interface::ControllerInterface)
