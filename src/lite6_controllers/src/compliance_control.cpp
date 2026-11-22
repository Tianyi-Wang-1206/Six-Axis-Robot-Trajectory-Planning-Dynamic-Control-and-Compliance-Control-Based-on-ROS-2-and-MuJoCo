#include "lite6_controllers/ctc_controller.hpp"
#include "lite6_interfaces/motion_contract.hpp"
#include <pinocchio/spatial/explog.hpp>
#include <algorithm>
#include <cmath>

namespace lite6_controllers
{
namespace
{
double interpolate_teach_level(const std::array<double, 5> & levels, double sensitivity)
{
  const double pos = std::clamp(sensitivity, 1.0, 5.0) - 1.0;
  const size_t idx = std::min<size_t>(3, static_cast<size_t>(std::floor(pos)));
  const double frac = pos - static_cast<double>(idx);
  return levels[idx] + frac * (levels[idx + 1] - levels[idx]);
}

bool load_teach_levels(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name, double min_val, bool strict_positive, double max_val,
  std::array<double, 5> & out)
{
  const auto values = node->get_parameter(name).as_double_array();
  if (values.size() != 5) {
    RCLCPP_ERROR(node->get_logger(), "%s must contain exactly five values (levels 1..5)", name.c_str());
    return false;
  }
  for (size_t i = 0; i < 5; ++i) {
    const bool ok = std::isfinite(values[i]) &&
      (strict_positive ? values[i] > min_val : values[i] >= min_val) && values[i] <= max_val;
    if (!ok) {
      RCLCPP_ERROR(node->get_logger(), "%s[%zu] out of valid range", name.c_str(), i);
      return false;
    }
    out[i] = values[i];
  }
  return true;
}
}  // namespace

bool Lite6CTCController::compliance_active() const
{
  return system_state_ == SystemState::JOINT_IMPEDANCE ||
    system_state_ == SystemState::CARTESIAN_IMPEDANCE ||
    system_state_ == SystemState::TEACH;
}

const Lite6CTCController::ComplianceModeConfig &
Lite6CTCController::active_compliance_config() const
{
  if (system_state_ == SystemState::CARTESIAN_IMPEDANCE) {
    return cart_imp_cfg_;
  }
  if (system_state_ == SystemState::TEACH) {
    return teach_cfg_;
  }
  return joint_imp_cfg_;
}

bool Lite6CTCController::configure_compliance()
{
  const auto node = get_node();
  auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
  auto nonneg = [](double v) { return std::isfinite(v) && v >= 0.0; };

  auto load_mode_common = [&](const std::string & prefix, ComplianceModeConfig & cfg) -> bool {
    cfg.max_joint_velocity = node->get_parameter(prefix + ".max_joint_velocity").as_double();
    cfg.hard_limit_tolerance = node->get_parameter(prefix + ".hard_limit_tolerance").as_double();
    cfg.saturation_timeout = node->get_parameter(prefix + ".saturation_timeout").as_double();
    cfg.entry_max_speed = node->get_parameter(prefix + ".entry_max_speed").as_double();
    cfg.filter_time_constant = node->get_parameter(prefix + ".filter_time_constant").as_double();
    cfg.soft_limit_margin = node->get_parameter(prefix + ".soft_limit_margin").as_double();
    cfg.soft_limit_stiffness = node->get_parameter(prefix + ".soft_limit_stiffness").as_double();
    cfg.soft_limit_damping = node->get_parameter(prefix + ".soft_limit_damping").as_double();
    cfg.torque_slew_rate = node->get_parameter(prefix + ".torque_slew_rate").as_double();
    cfg.transition_duration = node->get_parameter(prefix + ".transition_duration").as_double();

    if (!positive(cfg.max_joint_velocity) || !nonneg(cfg.hard_limit_tolerance) ||
        !positive(cfg.saturation_timeout) || !positive(cfg.entry_max_speed) ||
        cfg.entry_max_speed > lite6_interfaces::contract::kControllerEntryMaxSpeedRadPerSec ||
        !positive(cfg.filter_time_constant) || !nonneg(cfg.soft_limit_margin) ||
        !nonneg(cfg.soft_limit_stiffness) || !nonneg(cfg.soft_limit_damping) ||
        !positive(cfg.torque_slew_rate) || !positive(cfg.transition_duration))
    {
      RCLCPP_ERROR(node->get_logger(), "Invalid common compliance parameters under %s", prefix.c_str());
      return false;
    }
    return true;
  };

  if (!load_mode_common("compliance.joint_impedance", joint_imp_cfg_) ||
      !load_mode_common("compliance.cartesian_impedance", cart_imp_cfg_) ||
      !load_mode_common("compliance.teach", teach_cfg_))
  {
    return false;
  }

  cart_imp_cfg_.min_translation_damping =
    node->get_parameter("compliance.cartesian_impedance.min_translation_damping").as_double();
  cart_imp_cfg_.min_rotation_damping =
    node->get_parameter("compliance.cartesian_impedance.min_rotation_damping").as_double();
  cart_imp_cfg_.min_effective_mass =
    node->get_parameter("compliance.cartesian_impedance.min_effective_mass").as_double();
  cart_imp_cfg_.max_effective_mass =
    node->get_parameter("compliance.cartesian_impedance.max_effective_mass").as_double();
  cart_imp_cfg_.min_effective_inertia =
    node->get_parameter("compliance.cartesian_impedance.min_effective_inertia").as_double();
  cart_imp_cfg_.max_effective_inertia =
    node->get_parameter("compliance.cartesian_impedance.max_effective_inertia").as_double();
  cart_imp_cfg_.operational_damping_lambda =
    node->get_parameter("compliance.cartesian_impedance.operational_damping_lambda").as_double();

  if (!positive(cart_imp_cfg_.min_translation_damping) ||
      !positive(cart_imp_cfg_.min_rotation_damping) ||
      !positive(cart_imp_cfg_.min_effective_mass) ||
      !positive(cart_imp_cfg_.max_effective_mass) ||
      cart_imp_cfg_.min_effective_mass > cart_imp_cfg_.max_effective_mass ||
      !positive(cart_imp_cfg_.min_effective_inertia) ||
      !positive(cart_imp_cfg_.max_effective_inertia) ||
      cart_imp_cfg_.min_effective_inertia > cart_imp_cfg_.max_effective_inertia ||
      !positive(cart_imp_cfg_.operational_damping_lambda))
  {
    RCLCPP_ERROR(node->get_logger(), "Invalid compliance.cartesian_impedance operational damping parameters");
    return false;
  }

  teach_armature_weight_ = node->get_parameter("compliance.teach.armature_weight").as_double();
  const auto teach_min_d =
    node->get_parameter("compliance.teach.min_joint_damping").as_double_array();
  const auto teach_max_d =
    node->get_parameter("compliance.teach.max_joint_damping").as_double_array();
  if (!nonneg(teach_armature_weight_) || teach_armature_weight_ > 1.0 ||
      teach_min_d.size() != 6 || teach_max_d.size() != 6)
  {
    RCLCPP_ERROR(node->get_logger(), "Invalid compliance.teach diag(M) normalization parameters");
    return false;
  }
  for (size_t i = 0; i < 6; ++i) {
    if (!positive(teach_min_d[i]) || !positive(teach_max_d[i]) || teach_min_d[i] > teach_max_d[i]) {
      RCLCPP_ERROR(node->get_logger(), "Invalid compliance.teach min/max joint damping bounds");
      return false;
    }
    teach_min_joint_damping_[i] = teach_min_d[i];
    teach_max_joint_damping_[i] = teach_max_d[i];
  }

  if (!load_teach_levels(
        node, "compliance.teach.damping_levels", 0.0, true, 200.0, teach_damping_levels_)) {
    return false;
  }
  cartesian_characteristic_length_ = node->get_parameter("compliance.characteristic_length").as_double();
  if (!positive(cartesian_characteristic_length_)) {
    RCLCPP_ERROR(node->get_logger(), "Invalid compliance.characteristic_length");
    return false;
  }

  compliance_frame_ = pin_model_.getFrameId("link_eef");
  if (compliance_frame_ >= static_cast<pinocchio::FrameIndex>(pin_model_.nframes)) {
    throw std::runtime_error("Missing compliance TCP frame");
  }
  return true;
}

void Lite6CTCController::set_cartesian_targets_from_settings(bool reset_filtered)
{
  cart_kt_target_.setConstant(compliance_settings_.translation_stiffness);
  cart_dt_target_.setConstant(compliance_settings_.translation_damping);
  cart_kr_target_.setConstant(compliance_settings_.rotation_stiffness);
  cart_dr_target_.setConstant(compliance_settings_.rotation_damping);
  cart_wrench_ff_target_.setZero();
  if (reset_filtered) {
    cart_kt_filtered_ = cart_kt_target_;
    cart_dt_filtered_ = cart_dt_target_;
    cart_kr_filtered_ = cart_kr_target_;
    cart_dr_filtered_ = cart_dr_target_;
    cart_wrench_ff_filtered_.setZero();
  }
}

void Lite6CTCController::check_compliance_safety_limits()
{
  if (!compliance_active() && system_state_ != SystemState::COMPLIANCE_STOPPING) {
    return;
  }
  using CS = lite6_interfaces::msg::ControllerStatus;
  const auto & cfg = active_compliance_config();
  for (int i = 0; i < 6; ++i) {
    if (q_[i] < pin_model_.lowerPositionLimit[i] - cfg.hard_limit_tolerance) {
      latch_estop_reason(
        CS::ESTOP_COMPLIANCE_POSITION_LIMIT, i, q_[i], pin_model_.lowerPositionLimit[i]);
      request_execution_stop();
      return;
    }
    if (q_[i] > pin_model_.upperPositionLimit[i] + cfg.hard_limit_tolerance) {
      latch_estop_reason(
        CS::ESTOP_COMPLIANCE_POSITION_LIMIT, i, q_[i], pin_model_.upperPositionLimit[i]);
      request_execution_stop();
      return;
    }
    if (std::abs(dq_[i]) > cfg.max_joint_velocity) {
      latch_estop_reason(
        CS::ESTOP_COMPLIANCE_VELOCITY_LIMIT, i, dq_[i], cfg.max_joint_velocity);
      request_execution_stop();
      return;
    }
  }
}

void Lite6CTCController::step_compliance_stopping(double dt)
{
  if (system_state_ != SystemState::COMPLIANCE_STOPPING) {
    return;
  }
  compliance_stopping_elapsed_ += dt;
  if (estop_dq_.cwiseAbs().maxCoeff() < 1e-4 &&
      dq_.cwiseAbs().maxCoeff() <= lite6_interfaces::contract::kPlannerSettleSpeedRadPerSec)
  {
    // Resume NORMAL at the measured resting pose, never at the old equilibrium.
    bumpless_handover_to_normal(0.20);
    clear_estop_reason();
    compliance_stopping_elapsed_ = 0.0;
    transition_system_state(SystemState::NORMAL);
    return;
  }
  if (compliance_stopping_elapsed_ >= lite6_interfaces::contract::kComplianceStoppingTimeoutSec) {
    latch_estop_reason(
      lite6_interfaces::msg::ControllerStatus::ESTOP_COMPLIANCE_STOPPING_TIMEOUT,
      -1, compliance_stopping_elapsed_,
      lite6_interfaces::contract::kComplianceStoppingTimeoutSec);
    compliance_stopping_elapsed_ = 0.0;
    request_execution_stop();
  }
}

Eigen::Vector3d Lite6CTCController::rotational_spring_torque(
  const Eigen::Matrix3d & error_rotation, const Eigen::Vector3d & stiffness)
{
  // Negative world-frame gradient of V = 0.5 * log(Rd R^T)^T K log(Rd R^T).
  // Jlog3 differentiates a right perturbation, matching the actual world twist here.
  Eigen::Matrix3d log_jacobian;
  pinocchio::Jlog3(error_rotation, log_jacobian);
  return log_jacobian.transpose() * stiffness.cwiseProduct(pinocchio::log3(error_rotation));
}

void Lite6CTCController::compute_compliance(double dt)
{
  // Use encoder feedback and model dynamics only. Retain physical friction damping;
  // canceling it from measured velocity can sustain motion after hand release.
  ensure_dynamics_cached();
  const auto & cfg = active_compliance_config();
  const double alpha = 1.0 - std::exp(-dt / cfg.filter_time_constant);
  auto smooth = [alpha](double & value, double target) { value += alpha * (target - value); };
  auto & s = compliance_filtered_;
  smooth(s.joint_stiffness, compliance_settings_.joint_stiffness);
  smooth(s.joint_damping, compliance_settings_.joint_damping);
  smooth(s.sensitivity, compliance_settings_.sensitivity);
  smooth(s.nullspace_damping, compliance_settings_.nullspace_damping);
  tau_cmd_ = pin_data_dynamics_.g.head<6>();
  a_d_.setZero();
  integral_effort_.setZero();

  for (int i = 0; i < 6; ++i) {
    // Keep the captured equilibrium valid when it is inside the soft margin.
    const double lo = std::min(
      pin_model_.lowerPositionLimit[i] + cfg.soft_limit_margin, compliance_q_[i]);
    const double hi = std::max(
      pin_model_.upperPositionLimit[i] - cfg.soft_limit_margin, compliance_q_[i]);
    if (q_[i] < lo) {
      tau_cmd_[i] += cfg.soft_limit_stiffness * (lo - q_[i]) -
        cfg.soft_limit_damping * std::min(0.0, dq_[i]);
    }
    if (q_[i] > hi) {
      tau_cmd_[i] -= cfg.soft_limit_stiffness * (q_[i] - hi) +
        cfg.soft_limit_damping * std::max(0.0, dq_[i]);
    }
  }

  if (system_state_ == SystemState::JOINT_IMPEDANCE) {
    tau_cmd_ += s.joint_stiffness * (compliance_q_ - q_) - s.joint_damping * dq_;
  } else if (system_state_ == SystemState::CARTESIAN_IMPEDANCE) {
    const pinocchio::SE3 * pose_ptr = nullptr;
    const Mat6x8d * jac_ptr = nullptr;
    read_cached_tcp_kinematics(pose_ptr, jac_ptr);
    const auto & pose = *pose_ptr;
    const Mat6d j_arm = jac_ptr->leftCols<6>();
    const Vec6d velocity = *jac_ptr * dynamics_dq_;
    Vec6d lambda_diag;
    compute_cartesian_effective_inertia(
      j_arm, cfg.operational_damping_lambda,
      cfg.min_effective_mass, cfg.max_effective_mass,
      cfg.min_effective_inertia, cfg.max_effective_inertia, lambda_diag);
    cart_kt_filtered_ += alpha * (cart_kt_target_ - cart_kt_filtered_);
    cart_kr_filtered_ += alpha * (cart_kr_target_ - cart_kr_filtered_);
    cart_dt_filtered_ += alpha * (cart_dt_target_ - cart_dt_filtered_);
    cart_dr_filtered_ += alpha * (cart_dr_target_ - cart_dr_filtered_);
    cart_wrench_ff_filtered_ += alpha * (cart_wrench_ff_target_ - cart_wrench_ff_filtered_);
    Vec6d wrench;
    wrench.head<3>() = cart_kt_filtered_.cwiseProduct(compliance_pose_.translation() - pose.translation());
    wrench.tail<3>() = rotational_spring_torque(
      compliance_pose_.rotation() * pose.rotation().transpose(), cart_kr_filtered_);
    // Diagonal effective inertia is a damping heuristic, not modal decoupling.
    for (int k = 0; k < 3; ++k) {
      const double d_trans = std::max(cfg.min_translation_damping,
        2.0 * cart_dt_filtered_[k] * std::sqrt(cart_kt_filtered_[k] * lambda_diag[k]));
      const double d_rot = std::max(cfg.min_rotation_damping,
        2.0 * cart_dr_filtered_[k] * std::sqrt(cart_kr_filtered_[k] * lambda_diag[k + 3]));
      wrench[k] -= d_trans * velocity[k];
      wrench[k + 3] -= d_rot * velocity[k + 3];
    }
    tau_cmd_.noalias() += j_arm.transpose() * (wrench + cart_wrench_ff_filtered_);
    // Passive M-weighted singularity damping: -d0 * (M - J^T Lambda_reg J) * dq.
    // Symmetric positive definite everywhere and vanishing away from singularities.
    tau_cmd_ += compute_nullspace_damping_torque(
      j_arm, cfg.operational_damping_lambda, s.nullspace_damping, dq_);
  } else {
    const double decay_rate = interpolate_teach_level(teach_damping_levels_, s.sensitivity);
    for (int i = 0; i < 6; ++i) {
      const double m_eff = std::max(1e-4,
        pin_data_dynamics_.M(i, i) + teach_armature_weight_ * armature_[i]);
      const double damping = std::clamp(decay_rate * m_eff,
        teach_min_joint_damping_[i], teach_max_joint_damping_[i]);
      tau_cmd_[i] -= damping * dq_[i];
    }
  }

  // Apply torque slew-rate limiting ONLY during mode/recapture transition windows.
  apply_transition_torque_slew(dt, cfg.torque_slew_rate, tau_cmd_);

  bool saturated = false;
  int saturated_joint = -1;
  for (int i = 0; i < 6; ++i) {
    if (std::abs(tau_cmd_[i]) > max_effort_[i]) {
      saturated = true;
      if (saturated_joint < 0) saturated_joint = i;
    }
  }
  compliance_saturation_time_ = saturated ? compliance_saturation_time_ + dt : 0.0;
  // Apply braking on the next cycle, so this cycle retains force-mode semantics.
  if (compliance_saturation_time_ > cfg.saturation_timeout) {
    compliance_saturation_estop_pending_ = true;
    pending_estop_joint_ = static_cast<int8_t>(saturated_joint);
    pending_estop_value_ = compliance_saturation_time_;
    pending_estop_limit_ = cfg.saturation_timeout;
    emergency_pending_.store(true);
  }
}

}  // namespace lite6_controllers
