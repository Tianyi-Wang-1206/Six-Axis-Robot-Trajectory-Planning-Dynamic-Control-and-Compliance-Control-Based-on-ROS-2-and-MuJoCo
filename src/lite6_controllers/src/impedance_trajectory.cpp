#include "lite6_controllers/ctc_controller.hpp"
#include "lite6_interfaces/compliance_validation.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace lite6_controllers
{
namespace
{
double to_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return duration.sec + 1e-9 * duration.nanosec;
}

uint8_t map_impedance_estop_reason(
  int reason_code)
{
  using CS = lite6_interfaces::msg::ControllerStatus;
  switch (reason_code) {
    case 1: return CS::ESTOP_IMPEDANCE_TRAJECTORY_DEVIATION;
    case 2: return CS::ESTOP_COMPLIANCE_VELOCITY_LIMIT;
    case 3: return CS::ESTOP_COMPLIANCE_POSITION_LIMIT;
    case 4: return CS::ESTOP_COMPLIANCE_SATURATION;
    case 5: return CS::ESTOP_IMPEDANCE_TRAJECTORY_GOAL_TIMEOUT;
    case 6: return CS::ESTOP_TRAJECTORY_MODE_CHANGED;
    case 7: return CS::ESTOP_INVALID_STATE;
    case 8: return CS::ESTOP_INVALID_COMMAND;
    case 9: return CS::ESTOP_COMMAND_WRITE;
    case 10: return CS::ESTOP_CONTROL_EXCEPTION;
    case 11: return CS::ESTOP_COMPLIANCE_EXTERNAL_TORQUE;
    default: return CS::ESTOP_EXTERNAL;
  }
}
}  // namespace

bool Lite6CTCController::impedance_trajectory_active() const
{
  return system_state_ == SystemState::CARTESIAN_IMPEDANCE_TRAJECTORY;
}

bool Lite6CTCController::configure_impedance_trajectory()
{
  const auto node = get_node();
  imp_max_trans_deviation_ =
    node->get_parameter("impedance_execution.max_translation_deviation").as_double();
  imp_max_rot_deviation_ =
    node->get_parameter("impedance_execution.max_rotation_deviation").as_double();
  imp_max_joint_deviation_ =
    node->get_parameter("impedance_execution.max_joint_deviation").as_double();
  imp_max_joint_velocity_ =
    node->get_parameter("impedance_execution.max_joint_velocity").as_double();
  imp_hard_limit_tolerance_ =
    node->get_parameter("impedance_execution.hard_limit_tolerance").as_double();
  imp_saturation_timeout_ =
    node->get_parameter("impedance_execution.saturation_timeout").as_double();
  imp_goal_trans_tolerance_ =
    node->get_parameter("impedance_execution.goal_translation_tolerance").as_double();
  imp_goal_rot_tolerance_ =
    node->get_parameter("impedance_execution.goal_rotation_tolerance").as_double();
  imp_goal_velocity_tolerance_ =
    node->get_parameter("impedance_execution.goal_velocity_tolerance").as_double();
  imp_goal_settle_duration_ =
    node->get_parameter("impedance_execution.goal_settle_duration").as_double();
  imp_goal_time_tolerance_ =
    node->get_parameter("impedance_execution.goal_time_tolerance").as_double();
  imp_entry_max_speed_ =
    node->get_parameter("impedance_execution.entry_max_speed").as_double();
  imp_entry_position_tolerance_ =
    node->get_parameter("impedance_execution.entry_position_tolerance").as_double();
  imp_fv_scale_ = node->get_parameter("impedance_execution.friction_viscous_scale").as_double();
  imp_fc_scale_ = node->get_parameter("impedance_execution.friction_coulomb_scale").as_double();
  imp_fric_trans_speed_ =
    node->get_parameter("impedance_execution.friction_transition_speed").as_double();
  imp_fric_quiet_speed_ =
    node->get_parameter("impedance_execution.friction_quiet_speed").as_double();
  const auto imp_ext_lim =
    node->get_parameter("impedance_execution.max_external_joint_torque").as_double_array();
  imp_external_torque_timeout_ =
    node->get_parameter("impedance_execution.external_torque_timeout").as_double();

  auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
  auto nonneg = [](double v) { return std::isfinite(v) && v >= 0.0; };
  auto unit = [](double v) { return std::isfinite(v) && v >= 0.0 && v <= 1.0; };
  if (!positive(imp_max_trans_deviation_) ||
      !positive(imp_max_rot_deviation_) ||
      !positive(imp_max_joint_deviation_) ||
      !positive(imp_max_joint_velocity_) ||
      !nonneg(imp_hard_limit_tolerance_) ||
      !positive(imp_saturation_timeout_) ||
      !positive(imp_external_torque_timeout_) ||
      imp_ext_lim.size() != 6 ||
      !std::all_of(imp_ext_lim.begin(), imp_ext_lim.end(), positive) ||
      !unit(imp_fv_scale_) ||
      !unit(imp_fc_scale_) ||
      !positive(imp_fric_trans_speed_) ||
      !nonneg(imp_fric_quiet_speed_) ||
      !positive(imp_goal_trans_tolerance_) ||
      !positive(imp_goal_rot_tolerance_) ||
      !positive(imp_goal_velocity_tolerance_) ||
      !positive(imp_goal_settle_duration_) ||
      !positive(imp_goal_time_tolerance_) ||
      !positive(imp_entry_max_speed_) ||
      !positive(imp_entry_position_tolerance_) ||
      imp_goal_trans_tolerance_ > imp_max_trans_deviation_ ||
      imp_goal_rot_tolerance_ > imp_max_rot_deviation_ ||
      imp_goal_velocity_tolerance_ > imp_max_joint_velocity_ ||
      imp_goal_time_tolerance_ < imp_goal_settle_duration_ ||
      imp_goal_time_tolerance_ > 15.0 ||
      imp_entry_position_tolerance_ > imp_max_joint_deviation_) {
    RCLCPP_ERROR(node->get_logger(), "Invalid impedance_execution safety or tolerance parameters");
    return false;
  }
  for (size_t i = 0; i < 6; ++i) {
    imp_max_external_torque_[i] = imp_ext_lim[i];
  }

  imp_ref_jacobian_.setZero();
  impedance_trajectory_server_ = rclcpp_action::create_server<FollowImpedance>(
    node, "~/follow_impedance_trajectory",
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowImpedance::Goal> goal) {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      if (!trajectory_enabled_.load() || compliance_blocks_trajectory_.load() ||
          trajectory_reserved_ || impedance_trajectory_reserved_ ||
          !valid_impedance_trajectory_goal(*goal)) {
        return rclcpp_action::GoalResponse::REJECT;
      }
      impedance_trajectory_reserved_ = true;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](const std::shared_ptr<ImpedanceGoal> goal) {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      if (!active_impedance_trajectory_ || active_impedance_trajectory_->goal != goal ||
          active_impedance_trajectory_->result.load() != ImpedanceTrajectoryResult::RUNNING) {
        return rclcpp_action::CancelResponse::REJECT;
      }
      active_impedance_trajectory_->cancel.store(true);
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this](const std::shared_ptr<ImpedanceGoal> goal) {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      if (active_impedance_trajectory_) {
        retired_impedance_trajectories_.push_back(std::move(active_impedance_trajectory_));
      }
      active_impedance_trajectory_ = std::make_shared<ImpedanceTrajectoryExecution>(goal);
      if (!trajectory_enabled_.load()) {
        active_impedance_trajectory_->result.store(ImpedanceTrajectoryResult::DEACTIVATED);
      }
      impedance_trajectory_buffer_.writeFromNonRT(active_impedance_trajectory_);
    });
  return true;
}

bool Lite6CTCController::valid_impedance_trajectory_goal(
  const FollowImpedance::Goal & goal) const
{
  if (goal.joint_names != joint_names_ || goal.points.empty()) {
    return false;
  }
  double previous = -1.0;
  for (const auto & point : goal.points) {
    const double t = to_seconds(point.time_from_start);
    if (point.time_from_start.sec < 0 || point.time_from_start.nanosec >= 1000000000u ||
        t <= previous || !lite6_interfaces::valid_impedance_trajectory_gains(point)) {
      return false;
    }
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const double q = point.positions[i];
      const double dq = point.velocities[i];
      const double ddq = point.accelerations[i];
      if (!std::isfinite(q) || !std::isfinite(dq) || !std::isfinite(ddq)) {
        return false;
      }
      if (q < pin_model_.lowerPositionLimit[static_cast<int>(i)] - imp_hard_limit_tolerance_ ||
          q > pin_model_.upperPositionLimit[static_cast<int>(i)] + imp_hard_limit_tolerance_) {
        return false;
      }
    }
    previous = t;
  }
  if (to_seconds(goal.points.front().time_from_start) != 0.0) {
    return false;
  }
  const auto & last = goal.points.back();
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    if (std::abs(last.velocities[i]) > 1e-8 || std::abs(last.accelerations[i]) > 1e-8) {
      return false;
    }
  }
  return true;
}

void Lite6CTCController::fail_impedance_trajectory(
  ImpedanceTrajectoryResult reason, int joint, double value, double limit,
  const char * detail)
{
  int code = 0;
  switch (reason) {
    case ImpedanceTrajectoryResult::DEVIATION_LIMIT: code = 1; break;
    case ImpedanceTrajectoryResult::VELOCITY_LIMIT: code = 2; break;
    case ImpedanceTrajectoryResult::POSITION_LIMIT: code = 3; break;
    case ImpedanceTrajectoryResult::SATURATION: code = 4; break;
    case ImpedanceTrajectoryResult::GOAL_TIMEOUT: code = 5; break;
    case ImpedanceTrajectoryResult::MODE_CHANGED: code = 6; break;
    case ImpedanceTrajectoryResult::INVALID_STATE: code = 7; break;
    case ImpedanceTrajectoryResult::INVALID_COMMAND: code = 8; break;
    case ImpedanceTrajectoryResult::COMMAND_WRITE: code = 9; break;
    case ImpedanceTrajectoryResult::CONTROL_EXCEPTION: code = 10; break;
    case ImpedanceTrajectoryResult::EXTERNAL_TORQUE_LIMIT: code = 11; break;
    default: code = 0; break;
  }
  latch_estop_reason(map_impedance_estop_reason(code), joint, value, limit);
  request_execution_stop();
  auto * execution = current_impedance_execution_;
  if (!execution || execution->result.load() != ImpedanceTrajectoryResult::RUNNING) {
    return;
  }
  execution->failure_joint = joint;
  execution->failure_value = value;
  execution->failure_limit = limit;
  execution->failure_detail = detail;
  execution->result.store(reason);
}

void Lite6CTCController::update_impedance_trajectory_reference(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto * execution = current_impedance_execution_;
  if (!execution || execution->result.load() != ImpedanceTrajectoryResult::RUNNING) {
    return;
  }
  if (execution->cancel.load()) {
    if (!execution->cancel_applied) {
      if (system_state_ == SystemState::CARTESIAN_IMPEDANCE_TRAJECTORY) {
        system_state_ = SystemState::COMPLIANCE_STOPPING;
        estop_q_ = q_;
        estop_dq_ = dq_;
        integral_effort_.setZero();
        status_publish_pending_ = true;
      }
      execution->cancel_applied = true;
    }
    return;
  }

  const auto & points = execution->goal->get_goal()->points;
  if (!execution->started) {
    if (system_state_ != SystemState::NORMAL || execution_stop_latched_) {
      fail_impedance_trajectory(
        ImpedanceTrajectoryResult::MODE_CHANGED, -1, 0.0, 0.0,
        "Controller must be in NORMAL mode to start impedance trajectory");
      return;
    }
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (std::abs(dq_[i]) > imp_entry_max_speed_) {
        fail_impedance_trajectory(
          ImpedanceTrajectoryResult::VELOCITY_LIMIT, static_cast<int>(i),
          std::abs(dq_[i]), imp_entry_max_speed_,
          "Entry joint speed exceeds impedance_execution.entry_max_speed [rad/s]");
        return;
      }
      const double pos_diff = std::abs(points.front().positions[i] - q_[i]);
      if (pos_diff > imp_entry_position_tolerance_) {
        fail_impedance_trajectory(
          ImpedanceTrajectoryResult::DEVIATION_LIMIT, static_cast<int>(i),
          pos_diff, imp_entry_position_tolerance_,
          "Initial impedance trajectory point deviates from measured joint pose [rad]");
        return;
      }
    }
    execution->started = true;
    execution->start_time_ns = time.nanoseconds();
    system_state_ = SystemState::CARTESIAN_IMPEDANCE_TRAJECTORY;
    // Preserve last cycle's output torque for bumpless transition into impedance control.
    compliance_last_tau_ = tau_cmd_;
    compliance_saturation_time_ = 0.0;
    compliance_saturation_estop_pending_ = false;
    integral_effort_.setZero();
    const auto & first = points.front();
    for (int k = 0; k < 3; ++k) {
      imp_kt_filtered_[k] = first.translation_stiffness[k];
      imp_dt_filtered_[k] = first.translation_damping[k];
      imp_kr_filtered_[k] = first.rotation_stiffness[k];
      imp_dr_filtered_[k] = first.rotation_damping[k];
    }
    imp_null_damping_filtered_ = first.nullspace_damping;
    status_publish_pending_ = true;
  } else {
    execution->elapsed += period.seconds();
  }

  if (execution->endpoint_applied) {
    return;
  }

  size_t index = execution->last_sample_index;
  if (index >= points.size()) {
    index = 0;
  }
  while (index + 1 < points.size() &&
         to_seconds(points[index + 1].time_from_start) <= execution->elapsed) {
    ++index;
  }
  execution->last_sample_index = index;
  const auto & point = points[index];
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    q_imp_ref_[static_cast<int>(i)] = point.positions[i];
    dq_imp_ref_[static_cast<int>(i)] = point.velocities[i];
    ddq_imp_ref_[static_cast<int>(i)] = point.accelerations[i];
  }
  for (int k = 0; k < 3; ++k) {
    imp_kt_target_[k] = point.translation_stiffness[k];
    imp_dt_target_[k] = point.translation_damping[k];
    imp_kr_target_[k] = point.rotation_stiffness[k];
    imp_dr_target_[k] = point.rotation_damping[k];
  }
  imp_null_damping_target_ = point.nullspace_damping;
  execution->sample.store(index);
  execution->endpoint_applied = (index + 1 == points.size());
}

void Lite6CTCController::check_impedance_trajectory_path()
{
  auto * execution = current_impedance_execution_;
  if (!execution || execution->result.load() != ImpedanceTrajectoryResult::RUNNING ||
      execution->cancel.load() || !execution->started) {
    return;
  }
  if (system_state_ != SystemState::CARTESIAN_IMPEDANCE_TRAJECTORY) {
    fail_impedance_trajectory(
      ImpedanceTrajectoryResult::MODE_CHANGED, -1, 0.0, 0.0,
      "Controller mode changed during impedance trajectory execution");
    return;
  }
  for (int i = 0; i < 6; ++i) {
    if (q_[i] < pin_model_.lowerPositionLimit[i] - imp_hard_limit_tolerance_) {
      fail_impedance_trajectory(
        ImpedanceTrajectoryResult::POSITION_LIMIT, i,
        q_[i], pin_model_.lowerPositionLimit[i],
        "Lower mechanical joint limit exceeded during impedance trajectory [rad]");
      return;
    }
    if (q_[i] > pin_model_.upperPositionLimit[i] + imp_hard_limit_tolerance_) {
      fail_impedance_trajectory(
        ImpedanceTrajectoryResult::POSITION_LIMIT, i,
        q_[i], pin_model_.upperPositionLimit[i],
        "Upper mechanical joint limit exceeded during impedance trajectory [rad]");
      return;
    }
    if (std::abs(dq_[i]) > imp_max_joint_velocity_) {
      fail_impedance_trajectory(
        ImpedanceTrajectoryResult::VELOCITY_LIMIT, i,
        std::abs(dq_[i]), imp_max_joint_velocity_,
        "Joint velocity limit exceeded during impedance trajectory [rad/s]");
      return;
    }
    const double joint_dev = std::abs(q_imp_ref_[i] - q_[i]);
    if (joint_dev > imp_max_joint_deviation_) {
      fail_impedance_trajectory(
        ImpedanceTrajectoryResult::DEVIATION_LIMIT, i,
        joint_dev, imp_max_joint_deviation_,
        "Joint deflection from reference exceeded impedance limit [rad]");
      return;
    }
  }
}

void Lite6CTCController::compute_impedance_trajectory(double dt)
{
  const double alpha = 1.0 - std::exp(-dt / compliance_filter_time_constant_);
  imp_kt_filtered_ += alpha * (imp_kt_target_ - imp_kt_filtered_);
  imp_dt_filtered_ += alpha * (imp_dt_target_ - imp_dt_filtered_);
  imp_kr_filtered_ += alpha * (imp_kr_target_ - imp_kr_filtered_);
  imp_dr_filtered_ += alpha * (imp_dr_target_ - imp_dr_filtered_);
  imp_null_damping_filtered_ += alpha * (imp_null_damping_target_ - imp_null_damping_filtered_);

  // Gravity compensation on the measured configuration.
  tau_cmd_ = (gmo_gravity_cached_this_cycle_
    ? pin_data_gmo_.g
    : pinocchio::computeGeneralizedGravity(pin_model_, pin_data_, dynamics_q_)).head<6>();
  a_d_.setZero();
  integral_effort_.setZero();

  // 1. Evaluate time-varying equilibrium pose T_d(t) and twist v_d(t) from (q_imp_ref_, dq_imp_ref_).
  // link_eef is rigidly mounted on link6, so reference TCP kinematics depend only on the 6 arm joints.
  const bool ref_unchanged =
    imp_ref_kinematics_cached_ &&
    q_imp_ref_ == imp_ref_cached_q_ &&
    dq_imp_ref_ == imp_ref_cached_dq_;
  if (!ref_unchanged) {
    dynamics_q_ref_.head(6) = q_imp_ref_;
    dynamics_q_ref_.tail(2) = dynamics_q_.tail(2);
    dynamics_dq_ref_.head(6) = dq_imp_ref_;
    dynamics_dq_ref_.tail(2).setZero();

    pinocchio::computeJointJacobians(pin_model_, pin_data_ref_, dynamics_q_ref_);
    pinocchio::updateFramePlacement(pin_model_, pin_data_ref_, compliance_frame_);
    imp_ref_jacobian_.setZero();
    pinocchio::getFrameJacobian(
      pin_model_, pin_data_ref_, compliance_frame_,
      pinocchio::LOCAL_WORLD_ALIGNED, imp_ref_jacobian_);
    imp_ref_cached_q_ = q_imp_ref_;
    imp_ref_cached_dq_ = dq_imp_ref_;
    imp_ref_pose_cached_ = pin_data_ref_.oMf[compliance_frame_];
    imp_ref_vel_cached_ = imp_ref_jacobian_ * dynamics_dq_ref_;
    imp_ref_kinematics_cached_ = true;
  }
  const auto & ref_pose = imp_ref_pose_cached_;
  const Vec6d & ref_velocity = imp_ref_vel_cached_;

  // 2. Evaluate measured pose T(q) and twist v(q, dq), reusing GMO cache if valid this cycle.
  const pinocchio::SE3 * meas_pose_ptr = nullptr;
  const Mat6x8d * meas_jac_ptr = nullptr;
  read_cached_tcp_kinematics(meas_pose_ptr, meas_jac_ptr);
  const auto & meas_pose = *meas_pose_ptr;
  const auto & meas_jac = *meas_jac_ptr;
  const Vec6d meas_velocity = meas_jac * dynamics_dq_;

  // 3. Compute SE(3) pose & twist errors in LOCAL_WORLD_ALIGNED frame.
  imp_current_pos_error_ = ref_pose.translation() - meas_pose.translation();
  imp_current_rot_error_ =
    pinocchio::log3(ref_pose.rotation() * meas_pose.rotation().transpose());
  const Eigen::Vector3d vel_trans_error = ref_velocity.head<3>() - meas_velocity.head<3>();
  const Eigen::Vector3d vel_rot_error = ref_velocity.tail<3>() - meas_velocity.tail<3>();

  Vec6d wrench;
  wrench.head<3>() = imp_kt_filtered_.cwiseProduct(imp_current_pos_error_) +
                     imp_dt_filtered_.cwiseProduct(vel_trans_error);
  wrench.tail<3>() = imp_kr_filtered_.cwiseProduct(imp_current_rot_error_) +
                     imp_dr_filtered_.cwiseProduct(vel_rot_error);

  tau_cmd_.noalias() += meas_jac.leftCols<6>().transpose() * wrench;
  // Damping relative to reference joint velocity stabilizes null-space and near-singular motions.
  tau_cmd_ -= imp_null_damping_filtered_ * (dq_ - dq_imp_ref_);
  add_passive_friction_ff(
    imp_fv_scale_, imp_fc_scale_, imp_fric_trans_speed_, imp_fric_quiet_speed_,
    dq_imp_ref_, tau_cmd_);

  for (int i = 0; i < 6; ++i) {
    const double lo =
      std::min(pin_model_.lowerPositionLimit[i] + compliance_soft_limit_margin_, q_imp_ref_[i]);
    const double hi =
      std::max(pin_model_.upperPositionLimit[i] - compliance_soft_limit_margin_, q_imp_ref_[i]);
    if (q_[i] < lo) {
      tau_cmd_[i] += compliance_soft_limit_stiffness_ * (lo - q_[i]) -
                     compliance_soft_limit_damping_ * std::min(0.0, dq_[i]);
    }
    if (q_[i] > hi) {
      tau_cmd_[i] -= compliance_soft_limit_stiffness_ * (q_[i] - hi) +
                     compliance_soft_limit_damping_ * std::max(0.0, dq_[i]);
    }
    imp_effort_saturated_[static_cast<size_t>(i)] = std::abs(tau_cmd_[i]) > max_effort_[i];
    const double max_step = compliance_torque_slew_rate_ * dt;
    tau_cmd_[i] = std::clamp(
      tau_cmd_[i], compliance_last_tau_[i] - max_step, compliance_last_tau_[i] + max_step);
    compliance_last_tau_[i] = std::clamp(tau_cmd_[i], -max_effort_[i], max_effort_[i]);
  }
}

void Lite6CTCController::finish_impedance_trajectory_cycle(double dt)
{
  auto * execution = current_impedance_execution_;
  if (!execution || execution->result.load() != ImpedanceTrajectoryResult::RUNNING) {
    return;
  }
  if (execution->cancel.load()) {
    // Wait until COMPLIANCE_STOPPING decelerates the arm to rest and restores NORMAL
    // (or ESTOP if interrupted), ensuring a stationary, reference-synced post-cancel state.
    if (execution->cancel_applied &&
        (system_state_ == SystemState::NORMAL || system_state_ == SystemState::ESTOP)) {
      execution->result.store(ImpedanceTrajectoryResult::CANCELED);
    }
    return;
  }
  if (!execution->started) {
    return;
  }

  for (size_t k = 0; k < 3; ++k) {
    execution->pos_error[k].store(imp_current_pos_error_[static_cast<int>(k)], std::memory_order_relaxed);
    execution->rot_error[k].store(imp_current_rot_error_[static_cast<int>(k)], std::memory_order_relaxed);
  }
  for (size_t k = 0; k < 6; ++k) {
    execution->external_torque[k].store(
      gmo_residual_tau_[static_cast<Eigen::Index>(k)], std::memory_order_relaxed);
    execution->external_wrench[k].store(
      gmo_external_wrench_[static_cast<Eigen::Index>(k)], std::memory_order_relaxed);
  }
  execution->contact_detected.store(gmo_contact_detected_, std::memory_order_relaxed);

  const double pos_err_norm = imp_current_pos_error_.norm();
  const double rot_err_norm = imp_current_rot_error_.norm();
  if (pos_err_norm > imp_max_trans_deviation_) {
    fail_impedance_trajectory(
      ImpedanceTrajectoryResult::DEVIATION_LIMIT, -1,
      pos_err_norm, imp_max_trans_deviation_,
      "Cartesian translational deviation limit exceeded [m]");
    return;
  }
  if (rot_err_norm > imp_max_rot_deviation_) {
    fail_impedance_trajectory(
      ImpedanceTrajectoryResult::DEVIATION_LIMIT, -1,
      rot_err_norm, imp_max_rot_deviation_,
      "Cartesian rotational deviation limit exceeded [rad]");
    return;
  }

  bool any_saturated = false;
  int saturated_joint = -1;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    if (imp_effort_saturated_[i] || effort_saturated_[i]) {
      any_saturated = true;
      if (saturated_joint < 0) {
        saturated_joint = static_cast<int>(i);
      }
    }
  }
  execution->saturation_duration = any_saturated ? execution->saturation_duration + dt : 0.0;
  if (execution->saturation_duration >= imp_saturation_timeout_) {
    fail_impedance_trajectory(
      ImpedanceTrajectoryResult::SATURATION, saturated_joint,
      execution->saturation_duration, imp_saturation_timeout_,
      "Continuous effort saturation timeout during impedance trajectory [s]");
    return;
  }

  int ext_joint = -1;
  double ext_val = 0.0;
  double ext_limit = 0.0;
  if (check_external_torque_limit(
        dt, imp_max_external_torque_, imp_external_torque_timeout_,
        execution->external_torque_duration, ext_joint, ext_val, ext_limit)) {
    fail_impedance_trajectory(
      ImpedanceTrajectoryResult::EXTERNAL_TORQUE_LIMIT, ext_joint,
      ext_val, ext_limit,
      "External joint torque estimate exceeded impedance safety limit [Nm]");
    return;
  }

  if (!execution->endpoint_applied) {
    return;
  }

  const double pos_ratio = pos_err_norm / imp_goal_trans_tolerance_;
  const double rot_ratio = rot_err_norm / imp_goal_rot_tolerance_;
  const double vel_ratio = dq_.cwiseAbs().maxCoeff() / imp_goal_velocity_tolerance_;
  const double worst_ratio = std::max({pos_ratio, rot_ratio, vel_ratio});

  if (worst_ratio <= 1.0) {
    execution->settled_duration =
      (execution->was_settled && dt <= 0.1) ? execution->settled_duration + dt : 0.0;
    execution->was_settled = true;
  } else {
    execution->settled_duration = 0.0;
    execution->was_settled = false;
  }

  const double end_time =
    to_seconds(execution->goal->get_goal()->points.back().time_from_start);
  if (execution->elapsed > end_time + imp_goal_time_tolerance_) {
    fail_impedance_trajectory(
      ImpedanceTrajectoryResult::GOAL_TIMEOUT, -1,
      worst_ratio, 1.0,
      "Impedance trajectory goal settling deadline exceeded");
  } else if (execution->settled_duration >= imp_goal_settle_duration_) {
    // Recapture the actual resting pose into reference_interfaces_ before restoring NORMAL,
    // guaranteeing zero initial position/velocity error and bumpless handover to rigid CTC.
    const size_t n = joint_names_.size();
    for (size_t i = 0; i < n; ++i) {
      reference_interfaces_[i] = q_[i];
      reference_interfaces_[n + i] = 0.0;
      reference_interfaces_[2 * n + i] = 0.0;
    }
    q_target_ = q_;
    dq_target_.setZero();
    ddq_target_.setZero();
    integral_effort_.setZero();
    clear_estop_reason();
    system_state_ = SystemState::NORMAL;
    status_publish_pending_ = true;
    execution->result.store(ImpedanceTrajectoryResult::SUCCEEDED);
  }
}

void Lite6CTCController::monitor_impedance_trajectory()
{
  std::shared_ptr<ImpedanceTrajectoryExecution> execution;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (!active_impedance_trajectory_ && !retired_impedance_trajectories_.empty() &&
        rt_seen_impedance_execution_.load(std::memory_order_acquire) == nullptr)
    {
      impedance_trajectory_buffer_.writeFromNonRT(
        std::shared_ptr<ImpedanceTrajectoryExecution>{});
    }
    const auto * rt_seen = rt_seen_impedance_execution_.load(std::memory_order_acquire);
    retired_impedance_trajectories_.erase(
      std::remove_if(
        retired_impedance_trajectories_.begin(), retired_impedance_trajectories_.end(),
        [rt_seen](const std::shared_ptr<ImpedanceTrajectoryExecution> & ptr) {
          return ptr.use_count() == 1 && ptr.get() != rt_seen;
        }),
      retired_impedance_trajectories_.end());
    execution = active_impedance_trajectory_;
  }
  if (!execution) {
    return;
  }
  const auto outcome = execution->result.load();
  const auto & goal = execution->goal;
  if (outcome == ImpedanceTrajectoryResult::RUNNING) {
    if (goal->is_executing()) {
      auto feedback = std::make_shared<FollowImpedance::Feedback>();
      feedback->stamp = node_clock_ ? node_clock_->now() : get_node()->now();
      feedback->current_sample = static_cast<uint32_t>(execution->sample.load());
      for (size_t k = 0; k < 3; ++k) {
        feedback->position_error_m[k] = execution->pos_error[k].load(std::memory_order_relaxed);
        feedback->rotation_error_rad[k] = execution->rot_error[k].load(std::memory_order_relaxed);
      }
      for (size_t k = 0; k < 6; ++k) {
        feedback->external_joint_torque[k] =
          execution->external_torque[k].load(std::memory_order_relaxed);
        feedback->external_cartesian_wrench[k] =
          execution->external_wrench[k].load(std::memory_order_relaxed);
      }
      feedback->contact_detected =
        execution->contact_detected.load(std::memory_order_relaxed);
      goal->publish_feedback(feedback);
    }
    return;
  }

  auto result = std::make_shared<FollowImpedance::Result>();
  if (outcome == ImpedanceTrajectoryResult::CANCELED) {
    if (!goal->is_canceling()) {
      return;
    }
    result->error_code = FollowImpedance::Result::SUCCESSFUL;
    result->error_string = "Impedance trajectory canceled and safely settled to NORMAL";
    goal->canceled(result);
  } else if (outcome == ImpedanceTrajectoryResult::SUCCEEDED) {
    result->error_code = FollowImpedance::Result::SUCCESSFUL;
    result->error_string = "Impedance trajectory completed; reference synced to resting pose";
    goal->succeed(result);
  } else {
    switch (outcome) {
      case ImpedanceTrajectoryResult::DEVIATION_LIMIT:
        result->error_code = FollowImpedance::Result::DEVIATION_LIMIT_VIOLATED;
        break;
      case ImpedanceTrajectoryResult::VELOCITY_LIMIT:
        result->error_code = FollowImpedance::Result::VELOCITY_LIMIT_VIOLATED;
        break;
      case ImpedanceTrajectoryResult::POSITION_LIMIT:
        result->error_code = FollowImpedance::Result::POSITION_LIMIT_VIOLATED;
        break;
      case ImpedanceTrajectoryResult::SATURATION:
        result->error_code = FollowImpedance::Result::SATURATION_TIMEOUT;
        break;
      case ImpedanceTrajectoryResult::EXTERNAL_TORQUE_LIMIT:
        result->error_code = FollowImpedance::Result::EXTERNAL_TORQUE_LIMIT_VIOLATED;
        break;
      case ImpedanceTrajectoryResult::GOAL_TIMEOUT:
        result->error_code = FollowImpedance::Result::GOAL_TIMEOUT;
        break;
      case ImpedanceTrajectoryResult::MODE_CHANGED:
        result->error_code = FollowImpedance::Result::MODE_CHANGED;
        break;
      default:
        result->error_code = FollowImpedance::Result::CONTROLLER_FAULT;
        break;
    }
    result->error_string = (execution->failure_detail && execution->failure_detail[0] != '\0') ?
      execution->failure_detail : "Impedance trajectory execution aborted";
    if (execution->failure_joint >= 0 &&
        static_cast<size_t>(execution->failure_joint) < joint_names_.size()) {
      result->error_string += "; " + joint_names_[execution->failure_joint];
    }
    if (std::isfinite(execution->failure_limit) && execution->failure_limit > 0.0) {
      result->error_string += ", observed=" + std::to_string(execution->failure_value) +
        ", limit=" + std::to_string(execution->failure_limit);
    }
    goal->abort(result);
  }

  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  if (active_impedance_trajectory_ == execution) {
    retired_impedance_trajectories_.push_back(std::move(active_impedance_trajectory_));
    impedance_trajectory_buffer_.writeFromNonRT(
      std::shared_ptr<ImpedanceTrajectoryExecution>{});
    impedance_trajectory_reserved_ = false;
  }
}

}  // namespace lite6_controllers
