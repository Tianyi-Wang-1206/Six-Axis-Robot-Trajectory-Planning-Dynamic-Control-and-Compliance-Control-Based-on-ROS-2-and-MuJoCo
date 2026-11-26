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

  auto_declare<std::vector<double>>("kp", {3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0});
  auto_declare<std::vector<double>>("kv", {100.0, 100.0, 100.0, 100.0, 100.0, 100.0});
  auto_declare<std::vector<double>>("ki", {5000.0, 5000.0, 3500.0, 1750.0, 750.0, 400.0});
  auto_declare<std::vector<double>>("antiwindup_gain", {10.0, 10.0, 10.0, 10.0, 10.0, 10.0});
  auto_declare<std::vector<double>>("integral_max_effort", {12.5, 12.5, 8.5, 4.25, 1.75, 0.9});
  auto_declare<std::vector<double>>("friction_v_nominal", {1.002, 1.009, 0.506, 0.201, 0.101, 0.051});
  auto_declare<std::vector<double>>("friction_c_nominal", {1.499, 1.498, 0.999, 0.499, 0.200, 0.099});
  auto_declare<std::vector<double>>("armature", {0.1, 0.1, 0.1, 0.1, 0.1, 0.1});

  auto_declare<std::vector<double>>("max_effort", {50.0, 50.0, 32.0, 32.0, 32.0, 20.0});
  auto_declare<double>("tanh_slope", 300.0);

  // Keep acceleration defaults in sync with both motion-limit YAML files.
  auto_declare<std::vector<double>>("estop_decel", {16.0, 12.0, 20.0, 32.0, 36.0, 40.0});
  auto_declare<std::vector<double>>("execution.max_joint_velocity", {3.14, 3.14, 3.14, 3.14, 3.14, 3.14});
  auto_declare<std::vector<double>>("execution.max_joint_acceleration", {8.0, 6.0, 10.0, 16.0, 18.0, 20.0});
  auto_declare<double>("execution.position_limit_tolerance", 1e-4);
  auto_declare<std::vector<double>>("execution.path_position_tolerance", std::vector<double>(6, 0.1));
  auto_declare<std::vector<double>>("execution.path_velocity_tolerance", std::vector<double>(6, 0.5));
  auto_declare<std::vector<double>>("execution.goal_position_tolerance", std::vector<double>(6, 0.00015));
  auto_declare<std::vector<double>>("execution.sysid_goal_position_tolerance", std::vector<double>(6, 0.04));
  auto_declare<std::vector<double>>("execution.sysid_goal_velocity_tolerance", std::vector<double>(6, 0.005));
  auto_declare<std::vector<double>>("execution.goal_velocity_tolerance", std::vector<double>(6, 0.02));
  auto_declare<double>("execution.entry_position_tolerance", 0.05);
  auto_declare<double>("execution.entry_velocity_tolerance", 0.05);
  auto_declare<double>("execution.goal_settle_duration", 1.0);
  auto_declare<double>("execution.goal_time_tolerance", 5.0);
  auto_declare<double>("execution.saturation_timeout", 1.0);

  auto_declare<double>("payload.filter_time_constant", 0.03);
  auto_declare<double>("payload.nominal_gyration_radius", 0.025);

  auto_declare<double>("compliance.characteristic_length", 0.25);

  auto declare_mode_common = [this](const std::string & prefix) {
    auto_declare<double>(prefix + ".max_joint_velocity", 3.0);
    auto_declare<double>(prefix + ".hard_limit_tolerance", 0.01);
    auto_declare<double>(prefix + ".saturation_timeout", 1.0);
    auto_declare<double>(prefix + ".entry_max_speed", 0.03);
    auto_declare<double>(prefix + ".filter_time_constant", 0.15);
    auto_declare<double>(prefix + ".soft_limit_margin", 0.15);
    auto_declare<double>(prefix + ".soft_limit_stiffness", 80.0);
    auto_declare<double>(prefix + ".soft_limit_damping", 5.0);
    auto_declare<double>(prefix + ".torque_slew_rate", 120.0);
    auto_declare<double>(prefix + ".transition_duration", 0.25);
  };
  declare_mode_common("compliance.joint_impedance");

  declare_mode_common("compliance.cartesian_impedance");
  auto_declare<double>("compliance.cartesian_impedance.min_translation_damping", 2.0);
  auto_declare<double>("compliance.cartesian_impedance.min_rotation_damping", 0.15);
  auto_declare<double>("compliance.cartesian_impedance.min_effective_mass", 0.30);
  auto_declare<double>("compliance.cartesian_impedance.max_effective_mass", 15.0);
  auto_declare<double>("compliance.cartesian_impedance.min_effective_inertia", 0.005);
  auto_declare<double>("compliance.cartesian_impedance.max_effective_inertia", 0.60);
  auto_declare<double>("compliance.cartesian_impedance.operational_damping_lambda", 0.04);

  declare_mode_common("compliance.teach");
  auto_declare<double>("compliance.teach.armature_weight", 0.04);
  auto_declare<std::vector<double>>(
    "compliance.teach.min_joint_damping", {0.15, 0.15, 0.10, 0.04, 0.03, 0.015});
  auto_declare<std::vector<double>>(
    "compliance.teach.max_joint_damping", {15.0, 15.0, 10.0, 4.0, 3.0, 1.5});
  auto_declare<std::vector<double>>("compliance.teach.damping_levels", {22.0, 17.5, 13.5, 11.5, 9.5});
  auto_declare<double>("impedance_execution.max_translation_deviation", 0.12);
  auto_declare<double>("impedance_execution.max_rotation_deviation", 0.60);
  auto_declare<double>("impedance_execution.max_joint_deviation", 0.80);
  auto_declare<double>("impedance_execution.max_joint_velocity", 3.0);
  auto_declare<double>("impedance_execution.hard_limit_tolerance", 0.01);
  auto_declare<double>("impedance_execution.saturation_timeout", 1.0);
  auto_declare<double>("impedance_execution.dynamics_ff_scale", 1.0);
  auto_declare<double>("impedance_execution.friction_viscous_scale", 0.90);
  auto_declare<double>("impedance_execution.friction_coulomb_scale", 0.80);
  auto_declare<double>("impedance_execution.friction_transition_speed", 0.025);
  auto_declare<double>("impedance_execution.friction_quiet_speed", 0.008);
  auto_declare<double>("impedance_execution.filter_time_constant", 0.10);
  auto_declare<double>("impedance_execution.min_translation_damping", 2.0);
  auto_declare<double>("impedance_execution.min_rotation_damping", 0.15);
  auto_declare<double>("impedance_execution.min_effective_mass", 0.30);
  auto_declare<double>("impedance_execution.max_effective_mass", 15.0);
  auto_declare<double>("impedance_execution.min_effective_inertia", 0.005);
  auto_declare<double>("impedance_execution.max_effective_inertia", 0.60);
  auto_declare<double>("impedance_execution.operational_damping_lambda", 0.04);
  auto_declare<double>("impedance_execution.soft_limit_margin", 0.15);
  auto_declare<double>("impedance_execution.soft_limit_stiffness", 80.0);
  auto_declare<double>("impedance_execution.soft_limit_damping", 5.0);
  auto_declare<double>("impedance_execution.torque_slew_rate", 150.0);
  auto_declare<double>("impedance_execution.goal_translation_tolerance", 0.05);
  auto_declare<double>("impedance_execution.goal_rotation_tolerance", 0.25);
  auto_declare<double>("impedance_execution.goal_velocity_tolerance", 0.015);
  auto_declare<double>("impedance_execution.goal_settle_duration", 0.3);
  auto_declare<double>("impedance_execution.goal_time_tolerance", 5.0);
  auto_declare<double>("impedance_execution.entry_max_speed", 0.03);
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
    for (const auto * field : {"position", "velocity"}) {
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
  tau_applied_prev_.setZero();
  has_applied_tau_ = false;
  recovery_bias_active_ = false;
  recovery_bias_elapsed_ = 0.0;
  recovery_bias_init_.setZero();
  tau_point_ff_.setZero();
  integral_effort_.setZero();
  q_target_.setZero(); dq_target_.setZero(); ddq_target_.setZero();
  current_execution_ = nullptr;
  current_impedance_execution_ = nullptr;
  rt_seen_execution_.store(nullptr, std::memory_order_release);
  rt_seen_impedance_execution_.store(nullptr, std::memory_order_release);
  publish_measured_state_atomic(false);

  execution_max_joint_velocity_ =
    get_node()->get_parameter("execution.max_joint_velocity").as_double_array();
  execution_max_joint_acceleration_ =
    get_node()->get_parameter("execution.max_joint_acceleration").as_double_array();
  execution_position_limit_tolerance_ =
    get_node()->get_parameter("execution.position_limit_tolerance").as_double();
  path_position_tolerance_ = get_node()->get_parameter("execution.path_position_tolerance").as_double_array();
  path_velocity_tolerance_ = get_node()->get_parameter("execution.path_velocity_tolerance").as_double_array();
  goal_position_tolerance_ = get_node()->get_parameter("execution.goal_position_tolerance").as_double_array();
  sysid_goal_position_tolerance_ = get_node()->get_parameter("execution.sysid_goal_position_tolerance").as_double_array();
  sysid_goal_velocity_tolerance_ = get_node()->get_parameter("execution.sysid_goal_velocity_tolerance").as_double_array();
  goal_velocity_tolerance_ = get_node()->get_parameter("execution.goal_velocity_tolerance").as_double_array();
  entry_position_tolerance_ = get_node()->get_parameter("execution.entry_position_tolerance").as_double();
  entry_velocity_tolerance_ = get_node()->get_parameter("execution.entry_velocity_tolerance").as_double();
  goal_settle_duration_ = get_node()->get_parameter("execution.goal_settle_duration").as_double();
  goal_time_tolerance_ = get_node()->get_parameter("execution.goal_time_tolerance").as_double();
  saturation_timeout_ = get_node()->get_parameter("execution.saturation_timeout").as_double();
  for (const auto * values : {&execution_max_joint_velocity_, &execution_max_joint_acceleration_,
      &path_position_tolerance_, &path_velocity_tolerance_,
      &goal_position_tolerance_, &sysid_goal_position_tolerance_, &goal_velocity_tolerance_,
      &sysid_goal_velocity_tolerance_}) {
    if (values->size() != n || !std::all_of(values->begin(), values->end(),
        [](double value) {return std::isfinite(value) && value > 0.0;})) {
      RCLCPP_ERROR(get_node()->get_logger(), "Execution limits and tolerances require six finite positive values");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  if (!std::isfinite(execution_position_limit_tolerance_) ||
      execution_position_limit_tolerance_ < 0.0 ||
      execution_position_limit_tolerance_ > 0.01)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "execution.position_limit_tolerance must be finite and lie in [0.0, 0.01] rad");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!std::isfinite(entry_position_tolerance_) || entry_position_tolerance_ <= 0.0 ||
      !std::isfinite(entry_velocity_tolerance_) || entry_velocity_tolerance_ <= 0.0)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "execution.entry_position_tolerance and execution.entry_velocity_tolerance must be finite and positive");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < n; ++i) {
    if (entry_position_tolerance_ > path_position_tolerance_[i] ||
        entry_velocity_tolerance_ > path_velocity_tolerance_[i])
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Entry tolerances must not exceed execution path tolerances");
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
  auto fv_v = get_node()->get_parameter("friction_v_nominal").as_double_array();
  auto fc_v = get_node()->get_parameter("friction_c_nominal").as_double_array();
  if (fv_v.size() != n || fc_v.size() != n) {
    RCLCPP_ERROR(get_node()->get_logger(), "Friction parameter arrays must contain six values");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < n; ++i) {
    if (!std::isfinite(fv_v[i]) || !std::isfinite(fc_v[i]) ||
        fv_v[i] < 0.0 || fc_v[i] < 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(),
          "Invalid joint%zu friction parameters: require finite Fv >= 0, Fc >= 0", i + 1);
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  auto arm_v = get_node()->get_parameter("armature").as_double_array();
  auto effort_limit_v = get_node()->get_parameter("max_effort").as_double_array();
  auto estop_decel_v = get_node()->get_parameter("estop_decel").as_double_array();
  tanh_slope_ = get_node()->get_parameter("tanh_slope").as_double();
  if (kp_v.size() != n || kv_v.size() != n || ki_v.size() != n || aw_v.size() != n ||
      integral_limit_v.size() != n || arm_v.size() != n ||
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

  for (size_t i = 0; i < n; ++i) {
    const Eigen::Index idx = static_cast<Eigen::Index>(i);
    kp_[idx] = kp_v[i]; kv_[idx] = kv_v[i];
    friction_v_[idx] = fv_v[i]; friction_c_[idx] = fc_v[i]; armature_[idx] = arm_v[i];
    max_effort_[idx] = effort_limit_v[i];
    estop_decel_[idx] = estop_decel_v[i];
    ki_[idx] = ki_v[i];
    antiwindup_gain_[idx] = aw_v[i];
    integral_max_effort_[idx] = integral_limit_v[i];
  }

  payload_filter_time_constant_ = get_node()->get_parameter("payload.filter_time_constant").as_double();
  payload_nominal_gyration_radius_ = get_node()->get_parameter("payload.nominal_gyration_radius").as_double();
  if (!std::isfinite(payload_filter_time_constant_) || payload_filter_time_constant_ <= 0.0 ||
      !std::isfinite(payload_nominal_gyration_radius_) || payload_nominal_gyration_radius_ < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid payload filter configuration");
    return controller_interface::CallbackReturn::ERROR;
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
      if (id >= static_cast<pinocchio::JointIndex>(pin_model_.njoints) ||
          pin_model_.joints[id].idx_q() != static_cast<int>(i)) {
        throw std::runtime_error("Unexpected dynamics joint ordering");
      }
    }
    compliance_frame_ = pin_model_.getFrameId("link_eef");
    if (compliance_frame_ >= static_cast<pinocchio::FrameIndex>(pin_model_.nframes)) {
      throw std::runtime_error("Missing link_eef frame");
    }
    joint6_id_ = pin_model_.getJointId("joint6");
    if (joint6_id_ >= static_cast<pinocchio::JointIndex>(pin_model_.njoints)) {
      throw std::runtime_error("Missing joint6 in Pinocchio model");
    }
    base_joint6_inertia_ = pin_model_.inertias[joint6_id_];
    eef_to_joint6_placement_ = pin_model_.frames[compliance_frame_].placement;

    pin_data_ = pinocchio::Data(pin_model_);
    pin_data_ref_ = pinocchio::Data(pin_model_);
    pin_data_dynamics_ = pinocchio::Data(pin_model_);
    tcp_jacobian_.setZero();
    dynamics_q_.setZero();
    dynamics_dq_.setZero();
    dynamics_ddq_.setZero();
    dynamics_q_ref_.setZero();
    dynamics_dq_ref_.setZero();
    imp_ref_kinematics_cached_ = false;
  } catch (const std::exception &) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load URDF");
    return controller_interface::CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < n; ++i) {
    const int j = static_cast<int>(i);
    const double lo = pin_model_.lowerPositionLimit[j];
    const double hi = pin_model_.upperPositionLimit[j];
    const double urdf_vel = pin_model_.velocityLimit[j];
    if (!std::isfinite(lo) || !std::isfinite(hi) || !(lo < hi) ||
        !std::isfinite(urdf_vel) || !(urdf_vel > 0.0))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "URDF joint%zu must define finite lower < upper position limits and positive velocity limit",
        i + 1);
      return controller_interface::CallbackReturn::ERROR;
    }
    if (execution_max_joint_velocity_[i] > urdf_vel + 1e-6) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "execution.max_joint_velocity[%zu]=%.4f exceeds URDF velocity limit %.4f rad/s",
        i, execution_max_joint_velocity_[i], urdf_vel);
      return controller_interface::CallbackReturn::ERROR;
    }
    if (execution_max_joint_acceleration_[i] > estop_decel_[static_cast<Eigen::Index>(i)] + 1e-6) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "execution.max_joint_acceleration[%zu]=%.4f exceeds estop_decel %.4f rad/s^2",
        i, execution_max_joint_acceleration_[i], estop_decel_[static_cast<Eigen::Index>(i)]);
      return controller_interface::CallbackReturn::ERROR;
    }
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

  if (!configure_compliance() || !configure_impedance_trajectory()) {
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
  tau_point_ff_.setZero();
  compliance_blocks_trajectory_.store(false);
  emergency_pending_.store(false, std::memory_order_release);
  estop_rt_ack_generation_.store(
    estop_generation_.load(std::memory_order_acquire), std::memory_order_release);
  compliance_stopping_elapsed_ = 0.0;
  stopping_source_state_ = SystemState::JOINT_IMPEDANCE;
  rigid_cancel_braking_ = false;
  rigid_cancel_elapsed_ = 0.0;
  transition_slew_active_ = false;
  transition_slew_elapsed_ = 0.0;
  transition_slew_converged_cycles_ = 0;
  transition_last_tau_.setZero();
  recovery_bias_active_ = false;
  recovery_bias_elapsed_ = 0.0;
  recovery_bias_init_.setZero();
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
  publish_measured_state_atomic(true);

  payload_mass_filtered_ = payload_mass_;
  const Eigen::Vector3d c_tcp(payload_com_tcp_[0], payload_com_tcp_[1], payload_com_tcp_[2]);
  const Eigen::Vector3d c_link6 = eef_to_joint6_placement_.act(c_tcp);
  payload_first_moment_link6_filtered_ = payload_mass_filtered_ * c_link6;
  payload_smoothing_active_ = false;
  if (payload_mass_filtered_ > 1e-7) {
    const double i_diag = payload_mass_filtered_ *
      payload_nominal_gyration_radius_ * payload_nominal_gyration_radius_;
    const Eigen::Matrix3d I_com = Eigen::Vector3d::Constant(i_diag).asDiagonal();
    pin_model_.inertias[joint6_id_] =
      base_joint6_inertia_ + pinocchio::Inertia(payload_mass_filtered_, c_link6, I_com);
  } else {
    pin_model_.inertias[joint6_id_] = base_joint6_inertia_;
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

  // Only first activation starts in NORMAL; later activations brake in ESTOP.
  // Recovery requires CMD_CLEAR_ESTOP.
  const bool reactivation = activated_once_;
  activated_once_ = true;
  system_state_ = reactivation ? SystemState::ESTOP : SystemState::NORMAL;
  last_reported_state_ = system_state_;
  if (reactivation) {
    begin_deceleration_tracker();
    latch_estop_reason(lite6_interfaces::msg::ControllerStatus::ESTOP_EXTERNAL);
  }
  tau_applied_prev_.setZero();
  has_applied_tau_ = false;
  imp_ref_kinematics_cached_ = false;
  {
    // Coordinate queue/epoch reset with non-RT service producers so no command
    // checked against an older epoch can be enqueued after activation begins.
    std::lock_guard<std::mutex> qlock(cmd_queue_mutex_);
    const uint64_t discarded = cmd_gate_.exchange(0ULL, std::memory_order_acq_rel);
    if (discarded != 0ULL) {
      last_cmd_discarded_seq_.store(discarded, std::memory_order_release);
    }
    cmd_ring_.discard_all();
    estop_rt_ack_generation_.store(
      estop_generation_.load(std::memory_order_acquire), std::memory_order_release);
    estop_latest_cmd_seq_.store(0ULL, std::memory_order_release);
    controller_epoch_atomic_.store(controller_epoch_, std::memory_order_release);
    mode_epoch_atomic_.store(mode_epoch_, std::memory_order_release);
    current_mode_atomic_.store(
      controller_mode_for_state(system_state_), std::memory_order_release);
    trajectory_enabled_.store(true, std::memory_order_release);
  }

  // Force a fresh mode observation (NORMAL, or ESTOP on re-activation) on the first
  // successful update. Never infer a mode from the last command sent before reactivation.
  status_publish_counter_ = 0;
  status_publish_pending_ = true;
  publish_shadow_shm(&q_, true);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Lite6CTCController::on_deactivate(const rclcpp_lifecycle::State &)
{
  {
    std::lock_guard<std::mutex> qlock(cmd_queue_mutex_);
    trajectory_enabled_.store(false, std::memory_order_release);
    current_mode_atomic_.store(
      lite6_interfaces::msg::ControllerStatus::MODE_UNKNOWN, std::memory_order_release);
  }
  publish_measured_state_atomic(false);
  has_applied_tau_ = false;
  recovery_bias_active_ = false;
  recovery_bias_elapsed_ = 0.0;
  recovery_bias_init_.setZero();
  publish_shadow_shm(nullptr, false);
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  if (active_trajectory_) {
    auto expected = TrajectoryResult::RUNNING;
    active_trajectory_->result.compare_exchange_strong(expected, TrajectoryResult::DEACTIVATED);
    if (active_trajectory_->splice_owner) {
      uint8_t pending_state = static_cast<uint8_t>(SpliceState::PENDING);
      active_trajectory_->splice_owner->state.compare_exchange_strong(
        pending_state, static_cast<uint8_t>(SpliceState::REJECTED_STATE),
        std::memory_order_acq_rel, std::memory_order_acquire);
    }
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

void Lite6CTCController::update_payload_model(double dt)
{
  const Eigen::Vector3d c_tcp(payload_com_tcp_[0], payload_com_tcp_[1], payload_com_tcp_[2]);
  const Eigen::Vector3d c_link6 = eef_to_joint6_placement_.act(c_tcp);
  const Eigen::Vector3d s_target = payload_mass_ * c_link6;

  const double m_err = std::abs(payload_mass_ - payload_mass_filtered_);
  const double s_err = (s_target - payload_first_moment_link6_filtered_).norm();
  if (m_err <= 1e-7 && s_err <= 1e-8) {
    payload_smoothing_active_ = false;
    return;
  }

  payload_smoothing_active_ = true;
  Vec6d g_old = Vec6d::Zero();
  if (system_state_ == SystemState::NORMAL) {
    g_old = pinocchio::computeGeneralizedGravity(pin_model_, pin_data_, dynamics_q_).head<6>();
  }

  const double alpha = (payload_filter_time_constant_ > 1e-4) ?
    (1.0 - std::exp(-dt / payload_filter_time_constant_)) : 1.0;
  payload_mass_filtered_ += alpha * (payload_mass_ - payload_mass_filtered_);
  payload_first_moment_link6_filtered_ += alpha * (s_target - payload_first_moment_link6_filtered_);

  if (std::abs(payload_mass_ - payload_mass_filtered_) <= 1e-3 &&
      (s_target - payload_first_moment_link6_filtered_).norm() <= 1e-4)
  {
    payload_mass_filtered_ = payload_mass_;
    payload_first_moment_link6_filtered_ = s_target;
    payload_smoothing_active_ = false;
  }

  if (payload_mass_filtered_ > 1e-7) {
    const Eigen::Vector3d c_filt = payload_first_moment_link6_filtered_ / payload_mass_filtered_;
    const double i_diag = payload_mass_filtered_ *
      payload_nominal_gyration_radius_ * payload_nominal_gyration_radius_;
    const Eigen::Matrix3d I_com = Eigen::Vector3d::Constant(i_diag).asDiagonal();
    pin_model_.inertias[joint6_id_] =
      base_joint6_inertia_ + pinocchio::Inertia(payload_mass_filtered_, c_filt, I_com);
  } else {
    pin_model_.inertias[joint6_id_] = base_joint6_inertia_;
  }

  // Bumpless handover between feedback torque integrator and updated Pinocchio gravity model.
  if (system_state_ == SystemState::NORMAL) {
    const Vec6d g_new =
      pinocchio::computeGeneralizedGravity(pin_model_, pin_data_, dynamics_q_).head<6>();
    for (int i = 0; i < 6; ++i) {
      if (ki_[i] <= 0.0) continue;
      const double dg = g_new[i] - g_old[i];
      if (dg > 0.0 && integral_effort_[i] > 0.0) {
        integral_effort_[i] -= std::min(integral_effort_[i], dg);
      } else if (dg < 0.0 && integral_effort_[i] < 0.0) {
        integral_effort_[i] -= std::max(integral_effort_[i], dg);
      }
      integral_effort_[i] = std::clamp(
        integral_effort_[i], -integral_max_effort_[i], integral_max_effort_[i]);
    }
  }
}

void Lite6CTCController::begin_torque_transition(double duration_sec)
{
  transition_slew_active_ = true;
  transition_slew_elapsed_ = 0.0;
  transition_slew_duration_ = std::max(0.01, duration_sec);
  transition_slew_converged_cycles_ = 0;
  transition_last_tau_ = tau_applied_prev_;
}

void Lite6CTCController::bumpless_handover_to_normal(double transition_duration_sec)
{
  const size_t n = joint_names_.size();
  for (size_t i = 0; i < n; ++i) {
    reference_interfaces_[i] = q_[i];
    reference_interfaces_[n + i] = 0.0;
    reference_interfaces_[2 * n + i] = 0.0;
  }
  q_target_ = q_;
  dq_target_.setZero();
  ddq_target_.setZero();
  tau_point_ff_.setZero();
  estop_q_ = q_;
  estop_dq_.setZero();
  transition_slew_active_ = false;
  recovery_bias_active_ = false;
  recovery_bias_elapsed_ = 0.0;
  recovery_bias_init_.setZero();

  if (has_applied_tau_ && dynamics_q_.allFinite() && dynamics_dq_.allFinite() &&
      tau_applied_prev_.allFinite())
  {
    // Match the previous torque when recapturing the NORMAL reference.
    // Carry excess integral bias in a decaying cosine term while PD damping stays active.
    const Vec6d a_hold = -kv_.cwiseProduct(dq_);
    dynamics_ddq_.head(6) = a_hold;
    dynamics_ddq_.tail(2).setZero();
    pinocchio::rnea(pin_model_, pin_data_, dynamics_q_, dynamics_dq_, dynamics_ddq_);
    double max_excess = 0.0;
    for (size_t i = 0; i < n; ++i) {
      const Eigen::Index idx = static_cast<Eigen::Index>(i);
      const double tau_base = pin_data_.tau[idx] + armature_[idx] * a_hold[idx];
      const double bias = tau_applied_prev_[idx] - tau_base;
      if (!std::isfinite(bias)) {
        integral_effort_[idx] = 0.0;
        recovery_bias_init_[idx] = 0.0;
        continue;
      }
      if (ki_[idx] > 0.0) {
        integral_effort_[idx] =
          std::clamp(bias, -integral_max_effort_[idx], integral_max_effort_[idx]);
      } else {
        integral_effort_[idx] = 0.0;
      }
      const double excess = bias - integral_effort_[idx];
      recovery_bias_init_[idx] = excess;
      max_excess = std::max(max_excess, std::abs(excess));
    }
    if (max_excess > 1e-6) {
      const double nominal_dur = std::max(0.20, transition_duration_sec);
      recovery_bias_duration_ = std::clamp(std::max(nominal_dur, max_excess / 50.0), 0.20, 0.50);
      recovery_bias_elapsed_ = 0.0;
      recovery_bias_active_ = true;
    }
  } else {
    integral_effort_.setZero();
  }
}

void Lite6CTCController::apply_transition_torque_slew(
  double dt, double slew_rate, Vec6d & tau)
{
  if (!transition_slew_active_ || !(dt > 0.0) || !(slew_rate > 0.0)) {
    transition_last_tau_ = tau;
    return;
  }
  transition_slew_elapsed_ += dt;
  double effective_slew_rate = slew_rate;
  if (transition_slew_elapsed_ > transition_slew_duration_) {
    // Rather than dropping slew limiting abruptly mid-ramp when the nominal window
    // elapses, smoothly accelerate catch-up (up to 4x) until all joints converge.
    const double overtime = transition_slew_elapsed_ - transition_slew_duration_;
    const double boost =
      1.0 + std::min(3.0, overtime / std::max(0.05, transition_slew_duration_));
    effective_slew_rate = slew_rate * boost;
  }
  const double max_step = effective_slew_rate * dt;
  bool all_within_step = true;
  for (int i = 0; i < 6; ++i) {
    const double reachable_tau = std::clamp(tau[i], -max_effort_[i], max_effort_[i]);
    if (std::abs(reachable_tau - transition_last_tau_[i]) > max_step) {
      all_within_step = false;
    }
    tau[i] = std::clamp(
      tau[i], transition_last_tau_[i] - max_step, transition_last_tau_[i] + max_step);
    transition_last_tau_[i] = tau[i];
  }
  if (all_within_step) {
    ++transition_slew_converged_cycles_;
  } else {
    transition_slew_converged_cycles_ = 0;
  }
  const double max_slew_window =
    std::max(transition_slew_duration_ + 1.0, 2.0 * transition_slew_duration_);
  if (transition_slew_converged_cycles_ >= 5 ||
      transition_slew_elapsed_ >= max_slew_window)
  {
    transition_slew_active_ = false;
  }
}

void Lite6CTCController::compute_cartesian_effective_inertia(
  const Mat6d & j_arm, double lambda,
  double min_mass, double max_mass,
  double min_inertia, double max_inertia,
  Vec6d & out_lambda_diag) const
{
  Mat6d m_tot = pin_data_dynamics_.M.topLeftCorner<6, 6>();
  for (int i = 0; i < 6; ++i) {
    m_tot(i, i) += armature_[i];
  }
  const Eigen::LLT<Mat6d> llt(m_tot);
  if (llt.info() != Eigen::Success) {
    out_lambda_diag.head<3>().setConstant(std::clamp(1.5, min_mass, max_mass));
    out_lambda_diag.tail<3>().setConstant(std::clamp(0.05, min_inertia, max_inertia));
    return;
  }
  Mat6d lambda_inv = j_arm * llt.solve(j_arm.transpose());
  const double lam_rot_sq = lambda * lambda;
  const double lam_trans_sq =
    (cartesian_characteristic_length_ * lambda) * (cartesian_characteristic_length_ * lambda);
  for (int k = 0; k < 3; ++k) {
    lambda_inv(k, k) += lam_trans_sq;
    lambda_inv(k + 3, k + 3) += lam_rot_sq;
  }
  const Eigen::LDLT<Mat6d> ldlt(lambda_inv);
  if (ldlt.info() == Eigen::Success) {
    const Mat6d lambda_mat = ldlt.solve(Mat6d::Identity());
    for (int k = 0; k < 3; ++k) {
      const double m_k = std::isfinite(lambda_mat(k, k)) ? lambda_mat(k, k) : 1.5;
      const double i_k = std::isfinite(lambda_mat(k + 3, k + 3)) ? lambda_mat(k + 3, k + 3) : 0.05;
      out_lambda_diag[k] = std::clamp(m_k, min_mass, max_mass);
      out_lambda_diag[k + 3] = std::clamp(i_k, min_inertia, max_inertia);
    }
  } else {
    for (int k = 0; k < 3; ++k) {
      const double m_k = 1.0 / std::max(1e-6, lambda_inv(k, k));
      const double i_k = 1.0 / std::max(1e-6, lambda_inv(k + 3, k + 3));
      out_lambda_diag[k] = std::clamp(m_k, min_mass, max_mass);
      out_lambda_diag[k + 3] = std::clamp(i_k, min_inertia, max_inertia);
    }
  }
}

Lite6CTCController::Vec6d Lite6CTCController::compute_nullspace_damping_torque(
  const Mat6d & j_arm, double lambda, double nullspace_damping,
  const Vec6d & dq_err) const
{
  if (!(nullspace_damping > 0.0) || !(lambda > 0.0)) {
    return Vec6d::Zero();
  }
  // Use the Woodbury form M*(M + J^T Sigma^-1 J)^-1*M for passive damping,
  // avoiding cancellation in M - J^T Lambda_reg J.
  Mat6d m_tot = pin_data_dynamics_.M.topLeftCorner<6, 6>();
  for (int i = 0; i < 6; ++i) {
    m_tot(i, i) += armature_[i];
  }
  const double inv_lam_rot_sq = 1.0 / (lambda * lambda);
  const double lam_trans = cartesian_characteristic_length_ * lambda;
  const double inv_lam_trans_sq = 1.0 / (lam_trans * lam_trans);
  Mat6d j_weighted = j_arm;
  j_weighted.topRows<3>() *= inv_lam_trans_sq;
  j_weighted.bottomRows<3>() *= inv_lam_rot_sq;
  Mat6d h_mat = m_tot;
  h_mat.noalias() += j_arm.transpose() * j_weighted;
  const Eigen::LLT<Mat6d> llt(h_mat);
  if (llt.info() != Eigen::Success) {
    return Vec6d::Zero();
  }
  const Vec6d m_dq = m_tot * dq_err;
  const Vec6d damped = -nullspace_damping * (m_tot * llt.solve(m_dq));
  return damped.allFinite() ? damped : Vec6d::Zero();
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
  publish_measured_state_atomic(true);

  dynamics_q_.head(6) = q_;
  dynamics_dq_.head(6) = dq_;
  for (size_t i = 0; i < 2; ++i) {
    dynamics_q_[6 + i] = state_interfaces_[2 * n + 2 * i].get_value();
    dynamics_dq_[6 + i] = state_interfaces_[2 * n + 2 * i + 1].get_value();
  }
  // Do not read simulator jaw acceleration into the arm controller RNEA.
  dynamics_ddq_.tail(2).setZero();
  if (!dynamics_q_.allFinite() || !dynamics_dq_.allFinite())
    return fail_controller(TrajectoryResult::INVALID_STATE);

  // Smoothly update the payload model before evaluating the control law.
  update_payload_model(dt);

  dynamics_cached_this_cycle_ = false;
  coriolis_cached_this_cycle_ = false;
  tcp_cached_this_cycle_ = false;

  // Process ordered edge-triggered ControllerCommand requests and emergency triggers.
  process_controller_commands(dt);

  update_trajectory_reference(time, period);
  update_impedance_trajectory_reference(time, period);

  check_compliance_safety_limits();
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
    q_target_ = compliance_q_; dq_target_.setZero(); ddq_target_.setZero();
    if (system_state_ == SystemState::TEACH) q_target_ = q_;
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
  Vec6d tau_unslewed = Vec6d::Zero();
  bool transition_slew_applied = false;
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
    for (size_t i = 0; i < n; ++i) {
      tau_cmd_[i] = pin_data_.tau[i] + (armature_[i] * a_d_[i]);
    }
  } else {
    a_d_ = ddq_target_ + kp_.cwiseProduct(e_) + kv_.cwiseProduct(de_);
    dynamics_ddq_.head(6) = a_d_;
    pinocchio::rnea(pin_model_, pin_data_, dynamics_q_, dynamics_dq_, dynamics_ddq_);
    const bool use_friction_ff = (system_state_ == SystemState::NORMAL);
    const bool use_point_ff =
      (system_state_ == SystemState::NORMAL) && rigid_trajectory_active() && !rigid_cancel_braking_;
    Vec6d rec_bias = Vec6d::Zero();
    if (system_state_ == SystemState::NORMAL && recovery_bias_active_) {
      if (recovery_bias_elapsed_ < recovery_bias_duration_) {
        constexpr double kPi = 3.14159265358979323846;
        const double phase = recovery_bias_elapsed_ / recovery_bias_duration_;
        const double recovery_bias_scale = 0.5 * (1.0 + std::cos(kPi * phase));
        rec_bias = recovery_bias_scale * recovery_bias_init_;
        recovery_bias_elapsed_ += dt;
        if (recovery_bias_elapsed_ >= recovery_bias_duration_) {
          recovery_bias_active_ = false;
          recovery_bias_init_.setZero();
        }
      } else {
        recovery_bias_active_ = false;
        recovery_bias_init_.setZero();
      }
    }
    for (size_t i = 0; i < n; ++i) {
      // Feedforward viscous and Coulomb friction along reference velocity in NORMAL only.
      // Disabled during ESTOP and COMPLIANCE_STOPPING so physical friction aids passive braking.
      tau_fric_[i] = use_friction_ff
        ? (friction_v_[i] * dq_target_[i] + friction_c_[i] * std::tanh(tanh_slope_ * dq_target_[i]))
        : 0.0;
      // ki == 0 disables both integration and back-calculation for this joint.
      if (ki_[i] == 0.0) integral_effort_[i] = 0.0;
      const double ff_i = use_point_ff ? tau_point_ff_[static_cast<Eigen::Index>(i)] : 0.0;
      tau_cmd_[i] = pin_data_.tau[i] + (armature_[i] * a_d_[i]) + tau_fric_[i] +
          integral_effort_[i] + ff_i + rec_bias[static_cast<Eigen::Index>(i)];
    }
    tau_unslewed = tau_cmd_;
    if (system_state_ == SystemState::NORMAL && transition_slew_active_) {
      apply_transition_torque_slew(dt, imp_torque_slew_rate_, tau_cmd_);
      transition_slew_applied = true;
    } else {
      transition_last_tau_ = tau_cmd_;
    }
  }

  if (!a_d_.allFinite() || !tau_cmd_.allFinite())
    return fail_controller(TrajectoryResult::INVALID_COMMAND);

  // 5. Limit the TOTAL effort, then update the bias for the NEXT control cycle.
  for (size_t i = 0; i < n; ++i) {
    const double pre_clamp_effort = tau_cmd_[i];
    effort_saturated_[i] = std::abs(pre_clamp_effort) > max_effort_[i];
    tau_cmd_[i] = std::clamp(pre_clamp_effort, -max_effort_[i], max_effort_[i]);
    if (system_state_ != SystemState::SYS_ID && !compliance_active() &&
        !impedance_trajectory_active() && ki_[i] > 0.0) {
      const double slew_delta = transition_slew_applied ?
          (pre_clamp_effort - tau_unslewed[i]) : 0.0;
      const double sat_delta = tau_cmd_[i] - pre_clamp_effort;
      const bool slew_clipped = std::abs(slew_delta) > 1e-12;
      // During ESTOP/COMPLIANCE_STOPPING retain the holding bias without integrating
      // position error. During NORMAL transition slew ramps, freeze error integration
      // on slew-clipped joints so the integrator cannot wind up against the ramp.
      const double error_rate =
        (system_state_ == SystemState::NORMAL && !slew_clipped) ?
        ki_[i] * e_[i] : 0.0;
      // Back-calculate actuator magnitude saturation unconditionally. If a nonzero
      // integral bias actively opposes a slew ramp, unwind it toward zero without
      // flipping its sign or pumping a zero bias into the decaying prior torque.
      double slew_aw_rate = 0.0;
      if (slew_clipped && integral_effort_[i] * slew_delta < 0.0) {
        slew_aw_rate = antiwindup_gain_[i] * slew_delta;
        const double next_i = integral_effort_[i] + dt * slew_aw_rate;
        if ((integral_effort_[i] > 0.0 && next_i < 0.0) ||
            (integral_effort_[i] < 0.0 && next_i > 0.0)) {
          slew_aw_rate = -integral_effort_[i] / dt;
        }
      }
      const double rate = error_rate + antiwindup_gain_[i] * sat_delta + slew_aw_rate;
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
  has_applied_tau_ = true;
  transition_last_tau_ = tau_applied_prev_;
  // No success is published until this cycle's feedback and command writes pass.
  finish_trajectory_cycle(dt);
  finish_impedance_trajectory_cycle(dt);

  // 6. Write this cycle's actual reference directly to the shared-memory register.
  publish_shadow_shm(&q_target_, true);

  // Publish the explicit controller-mode mapping periodically; retry transitions
  // each cycle until the nonblocking publish succeeds.
  ++status_publish_counter_;
  if (status_publish_pending_ || system_state_ != last_reported_state_ ||
      status_publish_counter_ >= status_publish_rate_divider_) {
    publish_controller_status_rt(time);
  }

  return controller_interface::return_type::OK;
}

void Lite6CTCController::publish_controller_status_rt(const rclcpp::Time & stamp)
{
  if (!rt_status_pub_ || !rt_status_pub_->trylock()) {
    return;
  }
  auto & status = rt_status_pub_->msg_;
  status.stamp = stamp;
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

void Lite6CTCController::ensure_dynamics_cached(bool need_coriolis)
{
  if (!dynamics_cached_this_cycle_) {
    pinocchio::crba(pin_model_, pin_data_dynamics_, dynamics_q_);
    pin_data_dynamics_.M.triangularView<Eigen::StrictlyLower>() =
      pin_data_dynamics_.M.transpose().triangularView<Eigen::StrictlyLower>();
    pinocchio::computeGeneralizedGravity(pin_model_, pin_data_dynamics_, dynamics_q_);
    dynamics_cached_this_cycle_ = true;
  }
  if (need_coriolis && !coriolis_cached_this_cycle_) {
    pinocchio::computeCoriolisMatrix(pin_model_, pin_data_dynamics_, dynamics_q_, dynamics_dq_);
    coriolis_cached_this_cycle_ = true;
  }
}

void Lite6CTCController::read_cached_tcp_kinematics(
  const pinocchio::SE3 * & out_pose, const Mat6x8d * & out_jacobian)
{
  if (tcp_cached_this_cycle_) {
    out_pose = &pin_data_dynamics_.oMf[compliance_frame_];
    out_jacobian = &tcp_jacobian_;
    return;
  }
  pinocchio::computeJointJacobians(pin_model_, pin_data_dynamics_, dynamics_q_);
  pinocchio::updateFramePlacement(pin_model_, pin_data_dynamics_, compliance_frame_);
  tcp_jacobian_.setZero();
  pinocchio::getFrameJacobian(
    pin_model_, pin_data_dynamics_, compliance_frame_,
    pinocchio::LOCAL_WORLD_ALIGNED, tcp_jacobian_);
  tcp_cached_this_cycle_ = true;
  out_pose = &pin_data_dynamics_.oMf[compliance_frame_];
  out_jacobian = &tcp_jacobian_;
}

void Lite6CTCController::add_trajectory_friction_ff(
  double fv_scale, double fc_scale, double v_trans, double quiet_speed,
  const Vec6d & dq_ref,
  Vec6d & tau_out) const
{
  const size_t n = joint_names_.size();
  for (size_t i = 0; i < n; ++i) {
    const Eigen::Index idx = static_cast<Eigen::Index>(i);
    const double v_ref = dq_ref[idx];
    const double v_ref_abs = std::abs(v_ref);
    if (v_ref_abs <= 0.0) {
      continue;
    }
    // Compensate friction only when measured and reference motion align;
    // disable it during stalled contact or pushback.
    const double v_along = (v_ref >= 0.0) ? dq_[idx] : -dq_[idx];
    const double v_eff = std::clamp(v_along, 0.0, v_ref_abs);
    if (v_eff <= 0.0) {
      continue;
    }
    const double dq_eff = std::copysign(v_eff, v_ref);
    const double viscous = fv_scale * friction_v_[idx] * dq_eff;
    double coulomb_gate = 1.0;
    if (quiet_speed > 0.0) {
      const double ramp = std::clamp((v_eff - quiet_speed) / quiet_speed, 0.0, 1.0);
      coulomb_gate = ramp * ramp * (3.0 - 2.0 * ramp);
    }
    const double coulomb = fc_scale * coulomb_gate *
      friction_c_[idx] * std::tanh(dq_eff / v_trans);
    tau_out[idx] += viscous + coulomb;
  }
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
    if (next_state != SystemState::NORMAL) {
      recovery_bias_active_ = false;
      recovery_bias_elapsed_ = 0.0;
      recovery_bias_init_.setZero();
    }
    if (next_state == SystemState::ESTOP) {
      transition_slew_active_ = false;
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
  // Reserve at least half the executor threads for lock-free CMD_ESTOP preemption,
  // action callbacks, and timers under concurrent non-E-STOP bursts.
  const unsigned int hw_threads = std::thread::hardware_concurrency();
  max_ordinary_cmd_waiters_ = std::clamp<uint32_t>(
    hw_threads > 1u ? (hw_threads / 2u) : 1u, 1u, 8u);
  ordinary_cmd_waiters_.store(0u, std::memory_order_release);

  command_callback_group_ = get_node()->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);
  // Deep reliable service queue (256) prevents silent DDS KEEP_LAST(10) drops under
  // command bursts while remaining 100% RxO-compatible with default ServicesQoS clients.
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

      const bool uses_compliance =
        req->command == Cmd::CMD_ENTER_JOINT_IMPEDANCE ||
        req->command == Cmd::CMD_ENTER_CARTESIAN_IMPEDANCE ||
        req->command == Cmd::CMD_ENTER_TEACH ||
        req->command == Cmd::CMD_UPDATE_COMPLIANCE_SETTINGS;
      if (!lite6_interfaces::valid_controller_command(*req)) {
        resp->accepted = false;
        resp->ack_result = Resp::ACK_REJECTED_INVALID_ARG;
        resp->cmd_seq = 0;
        fill_snapshot();
        resp->message = "Invalid ControllerCommand code or parameter out of bounds";
        return;
      }
      if (uses_compliance && req->compliance.nullspace_stiffness != 0.0) {
        resp->accepted = false;
        resp->ack_result = Resp::ACK_REJECTED_INVALID_ARG;
        resp->cmd_seq = 0;
        fill_snapshot();
        resp->message = "Invalid ControllerCommand parameter: nullspace_stiffness must be zero";
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
        // E-STOP fast path: no epoch check, no waiter cap, no command mutex, no ring slot.
        // Publish cmd_seq before incrementing estop_generation_ so the RT loop
        // correlates this E-STOP sequence into ControllerStatus.last_cmd_*.
        const uint64_t estop_seq =
          command_sequence_.fetch_add(1ULL, std::memory_order_acq_rel) + 1ULL;
        uint64_t prev_seq = estop_latest_cmd_seq_.load(std::memory_order_relaxed);
        while (estop_seq > prev_seq &&
               !estop_latest_cmd_seq_.compare_exchange_weak(
                 prev_seq, estop_seq, std::memory_order_release, std::memory_order_relaxed))
        {}
        const uint64_t target_estop_gen =
          estop_generation_.fetch_add(1ULL, std::memory_order_acq_rel) + 1ULL;
        resp->cmd_seq = estop_seq;
        while (std::chrono::steady_clock::now() < deadline &&
               trajectory_enabled_.load(std::memory_order_acquire))
        {
          if (current_mode_atomic_.load(std::memory_order_acquire) == CS::MODE_ESTOP &&
              estop_rt_ack_generation_.load(std::memory_order_acquire) >= target_estop_gen)
          {
            resp->accepted = true;
            resp->ack_result = Resp::ACK_APPLIED;
            fill_snapshot();
            resp->message = "E-STOP latched";
            return;
          }
          std::this_thread::sleep_for(std::chrono::microseconds(250));
        }
        resp->accepted = false;
        resp->ack_result = Resp::ACK_OUTCOME_UNKNOWN;
        fill_snapshot();
        resp->message =
          "E-STOP not yet observed from the RT loop; latch request remains pending; reconcile via ControllerStatus";
        return;
      }

      // Explicit concurrency backpressure for non-E-STOP commands: cap concurrent waiters
      // on cmd_service_mutex_ so excess requests receive an immediate definitive rejection
      // instead of blocking the executor pool or starving CMD_ESTOP.
      uint32_t waiters = ordinary_cmd_waiters_.load(std::memory_order_acquire);
      while (true) {
        if (waiters >= max_ordinary_cmd_waiters_) {
          resp->accepted = false;
          resp->ack_result = Resp::ACK_REJECTED_BUSY;
          resp->cmd_seq = 0;
          fill_snapshot();
          resp->message =
            "Controller command gateway busy (concurrency limit reached); command not queued";
          return;
        }
        if (ordinary_cmd_waiters_.compare_exchange_weak(
              waiters, waiters + 1u, std::memory_order_acq_rel, std::memory_order_acquire))
        {
          break;
        }
      }
      struct WaiterGuard
      {
        std::atomic<uint32_t> & counter;
        ~WaiterGuard() { counter.fetch_sub(1u, std::memory_order_acq_rel); }
      } waiter_guard{ordinary_cmd_waiters_};

      std::lock_guard<std::mutex> lock(cmd_service_mutex_);

      uint64_t active_epoch = 0;
      PendingControllerCommand pending{};
      {
        std::lock_guard<std::mutex> qlock(cmd_queue_mutex_);
        if (!trajectory_enabled_.load(std::memory_order_acquire)) {
          resp->accepted = false;
          resp->ack_result = Resp::ACK_REJECTED_STATE;
          resp->cmd_seq = 0;
          fill_snapshot();
          resp->message = "CTC controller is not active";
          return;
        }
        if (estop_generation_.load(std::memory_order_acquire) != request_estop_generation) {
          resp->accepted = false;
          resp->ack_result = Resp::ACK_REJECTED_STATE;
          resp->cmd_seq = 0;
          fill_snapshot();
          resp->message = "Superseded by E-STOP before RT handoff; command not applied";
          return;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
          resp->accepted = false;
          resp->ack_result = Resp::ACK_RT_TIMEOUT;
          resp->cmd_seq = 0;
          fill_snapshot();
          resp->message = "Timed out waiting for controller command gateway; command not queued";
          return;
        }
        active_epoch = controller_epoch_atomic_.load(std::memory_order_acquire);
        if (req->expected_epoch > 0 && req->expected_epoch != active_epoch) {
          resp->accepted = false;
          resp->ack_result = Resp::ACK_REJECTED_EPOCH;
          resp->cmd_seq = 0;
          fill_snapshot();
          resp->message = "Controller epoch mismatch";
          return;
        }

        pending.cmd_seq = command_sequence_.fetch_add(1ULL, std::memory_order_acq_rel) + 1ULL;
        pending.controller_epoch = active_epoch;
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
      }

      auto fill_ack = [&]() {
        const uint8_t ack = last_cmd_ack_result_.load(std::memory_order_relaxed);
        resp->accepted = (ack == Resp::ACK_APPLIED);
        resp->ack_result = ack;
        resp->cmd_seq = pending.cmd_seq;
        resp->controller_epoch = last_cmd_ack_controller_epoch_.load(std::memory_order_relaxed);
        resp->mode_epoch = last_cmd_ack_mode_epoch_.load(std::memory_order_relaxed);
        resp->current_mode = last_cmd_ack_mode_.load(std::memory_order_relaxed);
        resp->message = resp->accepted ? "Applied" : "Rejected by RT controller";
      };
      auto acked = [&]() {
        return last_cmd_ack_seq_.load(std::memory_order_acquire) >= pending.cmd_seq;
      };

      bool deactivated = false;
      bool epoch_changed = false;
      while (std::chrono::steady_clock::now() < deadline) {
        if (acked()) {
          fill_ack();
          return;
        }
        if (!trajectory_enabled_.load(std::memory_order_acquire)) {
          deactivated = true;
          break;
        }
        if (controller_epoch_atomic_.load(std::memory_order_acquire) != active_epoch) {
          epoch_changed = true;
          break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(250));
      }

      // Try to withdraw the command. Success (or confirmed discard during reactivation)
      // guarantees the RT loop never applied it.
      uint64_t expected_gate = pending.cmd_seq;
      if (cmd_gate_.compare_exchange_strong(
            expected_gate, 0ULL, std::memory_order_acq_rel, std::memory_order_acquire) ||
          last_cmd_discarded_seq_.load(std::memory_order_acquire) == pending.cmd_seq)
      {
        resp->accepted = false;
        resp->ack_result = epoch_changed ? Resp::ACK_REJECTED_EPOCH :
          (deactivated ? Resp::ACK_REJECTED_STATE : Resp::ACK_RT_TIMEOUT);
        resp->cmd_seq = pending.cmd_seq;
        fill_snapshot();
        resp->message = epoch_changed ?
          "Controller reactivated before applying the command; command discarded (not applied)" :
          (deactivated ?
            "CTC controller deactivated before applying the command; command withdrawn (not applied)" :
            "Timed out waiting for 1 kHz RT controller handshake; command withdrawn (not applied)");
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
      resp->ack_result = Resp::ACK_OUTCOME_UNKNOWN;
      resp->cmd_seq = pending.cmd_seq;
      fill_snapshot();
      resp->message =
        "RT loop claimed the command without timely acknowledgement; outcome unknown, reconcile via ControllerStatus";
    },
    rclcpp::ServicesQoS().keep_last(256), command_callback_group_);
}

void Lite6CTCController::process_controller_commands(double /*dt*/)
{
  using Cmd = lite6_interfaces::srv::ControllerCommand::Request;
  using CS = lite6_interfaces::msg::ControllerStatus;
  using Resp = lite6_interfaces::srv::ControllerCommand::Response;

  // Latch pending internal (compliance saturation) or external (CMD_ESTOP fast-path)
  // emergencies BEFORE applying ring commands, so a racing CMD_CLEAR_ESTOP is rejected
  // cleanly instead of receiving ACK_APPLIED (MODE_NORMAL) and immediately re-latching ESTOP.
  const bool internal_emerg = emergency_pending_.exchange(false, std::memory_order_acq_rel);
  const uint64_t current_estop_generation = estop_generation_.load(std::memory_order_acquire);
  const uint64_t prev_ack_generation =
    estop_rt_ack_generation_.load(std::memory_order_relaxed);
  const bool external_emerg = (current_estop_generation > prev_ack_generation);
  const bool estop_pending = internal_emerg || external_emerg;

  if (estop_pending) {
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
    if (external_emerg) {
      const uint64_t estop_seq = estop_latest_cmd_seq_.load(std::memory_order_acquire);
      if (estop_seq > last_cmd_seq_) {
        last_cmd_seq_ = estop_seq;
        last_cmd_code_ = Cmd::CMD_ESTOP;
        last_cmd_result_ = Resp::ACK_APPLIED;
      }
    }
    estop_rt_ack_generation_.store(current_estop_generation, std::memory_order_release);
    status_publish_pending_ = true;
  }

  PendingControllerCommand batch[32];
  const size_t count = cmd_ring_.pop_batch(batch, 32);

  for (size_t i = 0; i < count; ++i) {
    const auto & cmd = batch[i];
    uint8_t res = Resp::ACK_RT_TIMEOUT;
    uint64_t expected_gate = cmd.cmd_seq;
    if (!cmd_gate_.compare_exchange_strong(
        expected_gate, 0ULL, std::memory_order_acq_rel, std::memory_order_acquire))
    {
      // Withdrawn by its timed-out service call: never applied.
      res = Resp::ACK_RT_TIMEOUT;
    } else if (cmd.controller_epoch != controller_epoch_) {
      // Command was queued under a previous controller activation epoch: never applied.
      res = Resp::ACK_REJECTED_EPOCH;
    } else if (estop_pending || cmd.estop_generation != current_estop_generation) {
      // Superseded by an E-STOP in this cycle or after request arrival: never applied.
      res = Resp::ACK_REJECTED_STATE;
    } else {
      res = apply_controller_command(cmd);
    }
    if (cmd.cmd_seq > last_cmd_seq_) {
      last_cmd_seq_ = cmd.cmd_seq;
      last_cmd_code_ = cmd.command;
      last_cmd_result_ = res;
    }
    last_cmd_ack_controller_epoch_.store(controller_epoch_, std::memory_order_relaxed);
    last_cmd_ack_code_.store(cmd.command, std::memory_order_relaxed);
    last_cmd_ack_result_.store(res, std::memory_order_relaxed);
    last_cmd_ack_mode_.store(current_mode_code(), std::memory_order_relaxed);
    last_cmd_ack_mode_epoch_.store(mode_epoch_, std::memory_order_relaxed);
    last_cmd_ack_seq_.store(cmd.cmd_seq, std::memory_order_release);
    status_publish_pending_ = true;
  }

  // Catch any CMD_ESTOP that arrived while the batch loop ran so E-STOP still wins
  // within the same 1 kHz cycle.
  const uint64_t post_estop_generation = estop_generation_.load(std::memory_order_acquire);
  if (post_estop_generation > estop_rt_ack_generation_.load(std::memory_order_relaxed)) {
    latch_estop_reason(CS::ESTOP_EXTERNAL);
    if (system_state_ != SystemState::ESTOP) {
      begin_deceleration_tracker();
      transition_system_state(SystemState::ESTOP);
    }
    const uint64_t estop_seq = estop_latest_cmd_seq_.load(std::memory_order_acquire);
    if (estop_seq > last_cmd_seq_) {
      last_cmd_seq_ = estop_seq;
      last_cmd_code_ = Cmd::CMD_ESTOP;
      last_cmd_result_ = Resp::ACK_APPLIED;
    }
    estop_rt_ack_generation_.store(post_estop_generation, std::memory_order_release);
    status_publish_pending_ = true;
  }
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
    out.nullspace_damping = cmd.nullspace_damping;
  };

  auto within_hard_limits = [this](double tol) -> bool {
    for (int i = 0; i < 6; ++i) {
      if (q_[i] < pin_model_.lowerPositionLimit[i] - tol ||
          q_[i] > pin_model_.upperPositionLimit[i] + tol) {
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
      bumpless_handover_to_normal(0.20);
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
        bumpless_handover_to_normal(0.20);
        transition_system_state(SystemState::NORMAL);
        return Resp::ACK_APPLIED;
      }
      if (compliance_active()) {
        if (max_speed <= lite6_interfaces::contract::kPlannerSettleSpeedRadPerSec) {
          bumpless_handover_to_normal(0.20);
          clear_estop_reason();
          compliance_stopping_elapsed_ = 0.0;
          transition_system_state(SystemState::NORMAL);
          return Resp::ACK_APPLIED;
        }
        stopping_source_state_ = system_state_;
        begin_deceleration_tracker();
        integral_effort_.setZero();
        compliance_stopping_elapsed_ = 0.0;
        transition_slew_active_ = false;
        transition_system_state(SystemState::COMPLIANCE_STOPPING);
        return Resp::ACK_APPLIED;
      }
      return Resp::ACK_REJECTED_STATE;

    case Cmd::CMD_ENTER_SYSID:
      if (busy) return Resp::ACK_REJECTED_BUSY;
      if (system_state_ != SystemState::NORMAL) return Resp::ACK_REJECTED_STATE;
      // SYS_ID mode is strictly prohibited when any payload is configured or smoothing.
      if (payload_mass_ > 1e-6 || payload_mass_filtered_ > 1e-6) {
        return Resp::ACK_REJECTED_STATE;
      }
      if (max_speed > lite6_interfaces::contract::kControllerEntryMaxSpeedRadPerSec) {
        return Resp::ACK_REJECTED_SPEED;
      }
      if (!within_hard_limits(joint_imp_cfg_.hard_limit_tolerance)) return Resp::ACK_REJECTED_LIMIT;
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        reference_interfaces_[i] = q_[i];
        reference_interfaces_[joint_names_.size() + i] = 0.0;
        reference_interfaces_[2 * joint_names_.size() + i] = 0.0;
      }
      tau_point_ff_.setZero();
      transition_system_state(SystemState::SYS_ID);
      return Resp::ACK_APPLIED;

    case Cmd::CMD_ENTER_JOINT_IMPEDANCE:
    case Cmd::CMD_ENTER_CARTESIAN_IMPEDANCE:
    case Cmd::CMD_ENTER_TEACH: {
      if (busy || payload_smoothing_active_) return Resp::ACK_REJECTED_BUSY;
      if (system_state_ != SystemState::NORMAL && !compliance_active()) {
        return Resp::ACK_REJECTED_STATE;
      }
      const SystemState desired =
        cmd.command == Cmd::CMD_ENTER_JOINT_IMPEDANCE ? SystemState::JOINT_IMPEDANCE :
        (cmd.command == Cmd::CMD_ENTER_CARTESIAN_IMPEDANCE ? SystemState::CARTESIAN_IMPEDANCE :
         SystemState::TEACH);
      const ComplianceModeConfig & target_cfg =
        desired == SystemState::JOINT_IMPEDANCE ? joint_imp_cfg_ :
        (desired == SystemState::CARTESIAN_IMPEDANCE ? cart_imp_cfg_ : teach_cfg_);
      const bool entering = (desired != system_state_);
      const double speed_lim = std::min(
        lite6_interfaces::contract::kControllerEntryMaxSpeedRadPerSec,
        target_cfg.entry_max_speed);
      if (entering && max_speed > speed_lim) {
        return Resp::ACK_REJECTED_SPEED;
      }
      if (!within_hard_limits(target_cfg.hard_limit_tolerance)) return Resp::ACK_REJECTED_LIMIT;
      populate_compliance_settings(compliance_settings_);
      if (entering || cmd.compliance_recapture) {
        compliance_q_ = q_;
        pinocchio::forwardKinematics(pin_model_, pin_data_, dynamics_q_);
        compliance_pose_ = pinocchio::updateFramePlacement(pin_model_, pin_data_, compliance_frame_);
        begin_torque_transition(target_cfg.transition_duration);
      }
      // Scalar settings replace any held per-axis gains / feedforward wrench; the filter
      // restarts only when the mode is newly entered.
      set_cartesian_targets_from_settings(entering);
      if (entering) {
        compliance_filtered_ = compliance_settings_;
        compliance_saturation_time_ = 0.0;
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
      set_cartesian_targets_from_settings(false);
      if (cmd.compliance_recapture) {
        compliance_q_ = q_;
        pinocchio::forwardKinematics(pin_model_, pin_data_, dynamics_q_);
        compliance_pose_ = pinocchio::updateFramePlacement(pin_model_, pin_data_, compliance_frame_);
        begin_torque_transition(active_compliance_config().transition_duration);
      }
      return Resp::ACK_APPLIED;

    case Cmd::CMD_SET_PAYLOAD:
      if (busy) return Resp::ACK_REJECTED_BUSY;
      if (system_state_ == SystemState::SYS_ID) return Resp::ACK_REJECTED_STATE;
      if (!q_.allFinite() || !dq_.allFinite()) return Resp::ACK_REJECTED_STATE;
      if (max_speed > lite6_interfaces::contract::kPlannerSettleSpeedRadPerSec) {
        return Resp::ACK_REJECTED_SPEED;
      }
      payload_mass_ = cmd.payload_mass;
      payload_com_tcp_ = {cmd.payload_com_tcp[0], cmd.payload_com_tcp[1], cmd.payload_com_tcp[2]};
      payload_smoothing_active_ = true;
      return Resp::ACK_APPLIED;

    default:
      return Resp::ACK_REJECTED_INVALID_ARG;
  }
}

}  // namespace lite6_controllers

PLUGINLIB_EXPORT_CLASS(lite6_controllers::Lite6CTCController, controller_interface::ControllerInterface)
