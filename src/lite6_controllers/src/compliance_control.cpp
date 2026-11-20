#include "lite6_controllers/ctc_controller.hpp"
#include "lite6_interfaces/compliance_validation.hpp"
#include "lite6_interfaces/motion_contract.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
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
  auto unit = [](double v) { return std::isfinite(v) && v >= 0.0 && v <= 1.0; };

  auto load_mode_common = [&](const std::string & prefix, ComplianceModeConfig & cfg) -> bool {
    cfg.max_joint_velocity = node->get_parameter(prefix + ".max_joint_velocity").as_double();
    cfg.hard_limit_tolerance = node->get_parameter(prefix + ".hard_limit_tolerance").as_double();
    cfg.saturation_timeout = node->get_parameter(prefix + ".saturation_timeout").as_double();
    cfg.entry_max_speed = node->get_parameter(prefix + ".entry_max_speed").as_double();
    const auto ext_lim =
      node->get_parameter(prefix + ".max_external_joint_torque").as_double_array();
    cfg.external_torque_timeout =
      node->get_parameter(prefix + ".external_torque_timeout").as_double();
    cfg.filter_time_constant = node->get_parameter(prefix + ".filter_time_constant").as_double();
    cfg.soft_limit_margin = node->get_parameter(prefix + ".soft_limit_margin").as_double();
    cfg.soft_limit_stiffness = node->get_parameter(prefix + ".soft_limit_stiffness").as_double();
    cfg.soft_limit_damping = node->get_parameter(prefix + ".soft_limit_damping").as_double();
    cfg.torque_slew_rate = node->get_parameter(prefix + ".torque_slew_rate").as_double();
    cfg.transition_duration = node->get_parameter(prefix + ".transition_duration").as_double();

    if (!positive(cfg.max_joint_velocity) || !nonneg(cfg.hard_limit_tolerance) ||
        !positive(cfg.saturation_timeout) || !positive(cfg.entry_max_speed) ||
        !positive(cfg.external_torque_timeout) || ext_lim.size() != 6 ||
        !std::all_of(ext_lim.begin(), ext_lim.end(), positive) ||
        !positive(cfg.filter_time_constant) || !nonneg(cfg.soft_limit_margin) ||
        !nonneg(cfg.soft_limit_stiffness) || !nonneg(cfg.soft_limit_damping) ||
        !positive(cfg.torque_slew_rate) || !positive(cfg.transition_duration))
    {
      RCLCPP_ERROR(node->get_logger(), "Invalid common compliance parameters under %s", prefix.c_str());
      return false;
    }
    for (size_t i = 0; i < 6; ++i) {
      cfg.max_external_torque[i] = ext_lim[i];
    }
    return true;
  };

  auto load_mode_friction = [&](const std::string & prefix, ComplianceModeConfig & cfg) -> bool {
    cfg.fv_scale = node->get_parameter(prefix + ".friction_viscous_scale").as_double();
    cfg.fc_scale = node->get_parameter(prefix + ".friction_coulomb_scale").as_double();
    cfg.fric_trans_speed = node->get_parameter(prefix + ".friction_transition_speed").as_double();
    cfg.fric_quiet_speed = node->get_parameter(prefix + ".friction_quiet_speed").as_double();
    if (!unit(cfg.fv_scale) || !unit(cfg.fc_scale) ||
        !positive(cfg.fric_trans_speed) || !nonneg(cfg.fric_quiet_speed) ||
        cfg.fric_quiet_speed >= cfg.fric_trans_speed)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Invalid friction parameters under %s (require 0 <= quiet_speed < transition_speed)",
        prefix.c_str());
      return false;
    }
    return true;
  };

  if (!load_mode_common("compliance.joint_impedance", joint_imp_cfg_) ||
      !load_mode_friction("compliance.joint_impedance", joint_imp_cfg_) ||
      !load_mode_common("compliance.cartesian_impedance", cart_imp_cfg_) ||
      !load_mode_friction("compliance.cartesian_impedance", cart_imp_cfg_) ||
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
        node, "compliance.teach.damping_levels", 0.0, true, 200.0, teach_damping_levels_) ||
      !load_teach_levels(
        node, "compliance.teach.assistance_levels", 0.0, false, 1.0, teach_assistance_levels_) ||
      !load_teach_levels(
        node, "compliance.teach.friction_transition_levels",
        0.0, true, 10.0, teach_friction_transition_levels_) ||
      !load_teach_levels(
        node, "compliance.teach.quiet_speed_levels",
        0.0, false, 10.0, teach_quiet_speed_levels_))
  {
    return false;
  }
  for (size_t i = 0; i < 5; ++i) {
    if (teach_quiet_speed_levels_[i] >= teach_friction_transition_levels_[i]) {
      RCLCPP_ERROR(
        node->get_logger(),
        "compliance.teach.quiet_speed_levels[%zu] must be strictly less than friction_transition_levels[%zu]",
        i, i);
      return false;
    }
  }

  compliance_frame_ = pin_model_.getFrameId("link_eef");
  if (compliance_frame_ >= static_cast<pinocchio::FrameIndex>(pin_model_.nframes)) {
    throw std::runtime_error("Missing compliance TCP frame");
  }
  compliance_jacobian_.setZero();
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

void Lite6CTCController::check_compliance_safety_limits(double dt)
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
  int ext_joint = -1;
  double ext_val = 0.0;
  double ext_limit = 0.0;
  if (check_external_torque_limit(
        dt, cfg.max_external_torque,
        cfg.external_torque_timeout, compliance_external_torque_time_,
        ext_joint, ext_val, ext_limit))
  {
    latch_estop_reason(CS::ESTOP_COMPLIANCE_EXTERNAL_TORQUE, ext_joint, ext_val, ext_limit);
    request_execution_stop();
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
    for (size_t i = 0; i < 6; ++i) {
      reference_interfaces_[i] = q_[i];
      reference_interfaces_[6 + i] = 0.0;
      reference_interfaces_[12 + i] = 0.0;
    }
    q_target_ = q_;
    dq_target_.setZero();
    ddq_target_.setZero();
    tau_point_ff_.setZero();
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

void Lite6CTCController::compute_compliance(double dt)
{
  // Only encoder position/velocity and model dynamics/gravity are used. No effort
  // state, MuJoCo contact force, applied force, or hidden friction state enters here.
  ensure_dynamics_cached();
  const auto & cfg = active_compliance_config();
  const double alpha = 1.0 - std::exp(-dt / cfg.filter_time_constant);
  auto smooth = [alpha](double & value, double target) { value += alpha * (target - value); };
  auto & s = compliance_filtered_;

  // Candidate stiffness increments: negative increments (softening) are strictly passive;
  // positive increments (stiffening under deflection) are regulated by the Energy Tank.
  // Cartesian translation/rotation gains are filtered per axis in their own branch.
  const double cand_dkq = alpha * (compliance_settings_.joint_stiffness - s.joint_stiffness);
  const double cand_dknull = alpha * (compliance_settings_.nullspace_stiffness - s.nullspace_stiffness);

  smooth(s.joint_damping, compliance_settings_.joint_damping);
  smooth(s.sensitivity, compliance_settings_.sensitivity);
  smooth(s.nullspace_damping, compliance_settings_.nullspace_damping);

  // Apply purely passive (decreasing) stiffness steps immediately.
  if (cand_dkq <= 0.0) s.joint_stiffness += cand_dkq;
  if (cand_dknull <= 0.0) s.nullspace_stiffness += cand_dknull;

  // Conservative gravity compensation (never attenuated by the energy tank).
  Vec6d tau_passive = pin_data_gmo_.g.head<6>();
  Vec6d tau_active = Vec6d::Zero();
  // Physical joint friction (nominal model) dissipates energy that friction
  // compensation in the active channel partially returns; credit it for harvesting.
  double dissipated_power = modeled_friction_dissipation();
  double stiffness_rate_power = 0.0;
  a_d_.setZero();
  integral_effort_.setZero();

  // Compute soft-limit wall contribution and its damping dissipation first.
  Vec6d tau_soft_limit = Vec6d::Zero();
  for (int i = 0; i < 6; ++i) {
    // Preserve an equilibrium captured inside the margin (home is on joint3's
    // lower boundary). Do not push the arm simply because a mode was entered.
    const double lo = std::min(
      pin_model_.lowerPositionLimit[i] + cfg.soft_limit_margin, compliance_q_[i]);
    const double hi = std::max(
      pin_model_.upperPositionLimit[i] - cfg.soft_limit_margin, compliance_q_[i]);
    if (q_[i] < lo) {
      const double v_in = std::min(0.0, dq_[i]);
      tau_soft_limit[i] += cfg.soft_limit_stiffness * (lo - q_[i]) - cfg.soft_limit_damping * v_in;
      dissipated_power += cfg.soft_limit_damping * v_in * v_in;
    }
    if (q_[i] > hi) {
      const double v_out = std::max(0.0, dq_[i]);
      tau_soft_limit[i] -= cfg.soft_limit_stiffness * (q_[i] - hi) + cfg.soft_limit_damping * v_out;
      dissipated_power += cfg.soft_limit_damping * v_out * v_out;
    }
  }
  tau_passive += tau_soft_limit;

  if (system_state_ == SystemState::JOINT_IMPEDANCE) {
    const Vec6d q_err = compliance_q_ - q_;
    dissipated_power += s.joint_damping * dq_.squaredNorm();
    if (cand_dkq > 0.0 && dt > 1e-6) {
      stiffness_rate_power += 0.5 * (cand_dkq / dt) * q_err.squaredNorm();
    }
    add_passive_friction_ff(
      cfg.fv_scale, cfg.fc_scale, cfg.fric_trans_speed, cfg.fric_quiet_speed, dq_, tau_active);

    const double active_power = tau_active.dot(dq_) + stiffness_rate_power;
    const double tank_valve = step_energy_tank(dt, dissipated_power, active_power);
    if (cand_dkq > 0.0) {
      s.joint_stiffness += tank_valve * cand_dkq;
    }
    tau_passive += s.joint_stiffness * q_err - s.joint_damping * dq_;
    tau_cmd_ = tau_passive + tank_valve * tau_active;
  } else if (system_state_ == SystemState::CARTESIAN_IMPEDANCE) {
    const pinocchio::SE3 * pose_ptr = nullptr;
    const Mat6x8d * jac_ptr = nullptr;
    read_cached_tcp_kinematics(pose_ptr, jac_ptr);
    const auto & pose = *pose_ptr;
    const auto & jac = *jac_ptr;
    const Mat6d j_arm = jac.leftCols<6>();

    Vec6d lambda_diag = Vec6d::Ones();
    compute_cartesian_effective_inertia(
      j_arm, cfg.operational_damping_lambda,
      cfg.min_effective_mass, cfg.max_effective_mass,
      cfg.min_effective_inertia, cfg.max_effective_inertia,
      lambda_diag);

    // Per-axis gain filtering: softening and damping/wrench changes apply directly;
    // stiffening is energy-tank regulated below.
    const Eigen::Vector3d cand_kt = alpha * (cart_kt_target_ - cart_kt_filtered_);
    const Eigen::Vector3d cand_kr = alpha * (cart_kr_target_ - cart_kr_filtered_);
    cart_dt_filtered_ += alpha * (cart_dt_target_ - cart_dt_filtered_);
    cart_dr_filtered_ += alpha * (cart_dr_target_ - cart_dr_filtered_);
    cart_wrench_ff_filtered_ += alpha * (cart_wrench_ff_target_ - cart_wrench_ff_filtered_);
    for (int k = 0; k < 3; ++k) {
      if (cand_kt[k] <= 0.0) cart_kt_filtered_[k] += cand_kt[k];
      if (cand_kr[k] <= 0.0) cart_kr_filtered_[k] += cand_kr[k];
    }

    Vec6d error;
    error.head<3>() = compliance_pose_.translation() - pose.translation();
    error.tail<3>() = pinocchio::log3(compliance_pose_.rotation() * pose.rotation().transpose());
    const Vec6d velocity = jac * dynamics_dq_;
    const Vec6d q_null_err = compliance_q_ - q_;

    // Constant damping ratio law: D_i(q) = max(D_min, 2 * zeta_i * sqrt(K_i * Lambda_i(q))).
    auto cart_damping = [&]() {
      Vec6d d;
      for (int a = 0; a < 3; ++a) {
        d[a] = std::max(
          cfg.min_translation_damping,
          2.0 * cart_dt_filtered_[a] * std::sqrt(std::max(0.0, cart_kt_filtered_[a]) * lambda_diag[a]));
        d[a + 3] = std::max(
          cfg.min_rotation_damping,
          2.0 * cart_dr_filtered_[a] * std::sqrt(std::max(0.0, cart_kr_filtered_[a]) * lambda_diag[a + 3]));
      }
      return d;
    };
    // Dissipation estimate with the pre-stiffening gains.
    dissipated_power += velocity.dot(cart_damping().cwiseProduct(velocity));

    if (dt > 1e-6) {
      for (int k = 0; k < 3; ++k) {
        if (cand_kt[k] > 0.0) {
          stiffness_rate_power += 0.5 * (cand_kt[k] / dt) * error[k] * error[k];
        }
        if (cand_kr[k] > 0.0) {
          stiffness_rate_power += 0.5 * (cand_kr[k] / dt) * error[k + 3] * error[k + 3];
        }
      }
      if (cand_dknull > 0.0) {
        stiffness_rate_power += 0.5 * (cand_dknull / dt) * q_null_err.squaredNorm();
      }
    }

    // Posture regulation acts only in the dynamically consistent null space of the TCP
    // task, so it never changes the commanded Cartesian stiffness. Its projected power is
    // not sign-definite, so it runs through the (bidirectional) tank channel.
    tau_active += project_nullspace_torque(
      j_arm, cfg.operational_damping_lambda,
      s.nullspace_stiffness * q_null_err - s.nullspace_damping * dq_);
    // Feedforward wrench (non-zero only when holding an impedance trajectory's endpoint).
    tau_active.noalias() += j_arm.transpose() * cart_wrench_ff_filtered_;
    add_passive_friction_ff(
      cfg.fv_scale, cfg.fc_scale, cfg.fric_trans_speed, cfg.fric_quiet_speed, dq_, tau_active);

    const double active_power = tau_active.dot(dq_) + stiffness_rate_power;
    const double tank_valve = step_energy_tank(dt, dissipated_power, active_power);
    for (int k = 0; k < 3; ++k) {
      if (cand_kt[k] > 0.0) cart_kt_filtered_[k] += tank_valve * cand_kt[k];
      if (cand_kr[k] > 0.0) cart_kr_filtered_[k] += tank_valve * cand_kr[k];
    }
    if (cand_dknull > 0.0) s.nullspace_stiffness += tank_valve * cand_dknull;

    const Vec6d d_cart = cart_damping();
    Vec6d wrench;
    for (int k = 0; k < 3; ++k) {
      wrench[k] = cart_kt_filtered_[k] * error[k] - d_cart[k] * velocity[k];
      wrench[k + 3] = cart_kr_filtered_[k] * error[k + 3] - d_cart[k + 3] * velocity[k + 3];
    }
    tau_passive.noalias() += j_arm.transpose() * wrench;
    tau_cmd_ = tau_passive + tank_valve * tau_active;
  } else {
    // diag(M)-normalized drag teaching:
    // d_i(q) = clamp(lambda_teach(sensitivity) * (M_link_ii(q) + beta_arm * I_arm_i), d_min_i, d_max_i).
    const double decay_rate = interpolate_teach_level(teach_damping_levels_, s.sensitivity);
    const double assistance = interpolate_teach_level(teach_assistance_levels_, s.sensitivity);
    const double friction_transition =
      interpolate_teach_level(teach_friction_transition_levels_, s.sensitivity);
    const double quiet_speed = interpolate_teach_level(teach_quiet_speed_levels_, s.sensitivity);
    for (int i = 0; i < 6; ++i) {
      const double m_eff_i = std::max(
        1e-4, pin_data_gmo_.M(i, i) + teach_armature_weight_ * armature_[i]);
      const double d_i = std::clamp(
        decay_rate * m_eff_i,
        teach_min_joint_damping_[static_cast<size_t>(i)],
        teach_max_joint_damping_[static_cast<size_t>(i)]);
      tau_passive[i] -= d_i * dq_[i];
      dissipated_power += d_i * dq_[i] * dq_[i];
    }
    add_passive_friction_ff(
      assistance, assistance, friction_transition, quiet_speed, dq_, tau_active);

    const double active_power = tau_active.dot(dq_);
    const double tank_valve = step_energy_tank(dt, dissipated_power, active_power);
    tau_cmd_ = tau_passive + tank_valve * tau_active;
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
