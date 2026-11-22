#include "lite6_controllers/ctc_controller.hpp"
#include "lite6_interfaces/compliance_validation.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
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
  imp_dynamics_ff_scale_ =
    node->get_parameter("impedance_execution.dynamics_ff_scale").as_double();
  imp_fv_scale_ = node->get_parameter("impedance_execution.friction_viscous_scale").as_double();
  imp_fc_scale_ = node->get_parameter("impedance_execution.friction_coulomb_scale").as_double();
  imp_fric_trans_speed_ =
    node->get_parameter("impedance_execution.friction_transition_speed").as_double();
  imp_fric_quiet_speed_ =
    node->get_parameter("impedance_execution.friction_quiet_speed").as_double();
  imp_filter_time_constant_ =
    node->get_parameter("impedance_execution.filter_time_constant").as_double();
  imp_min_translation_damping_ =
    node->get_parameter("impedance_execution.min_translation_damping").as_double();
  imp_min_rotation_damping_ =
    node->get_parameter("impedance_execution.min_rotation_damping").as_double();
  imp_min_effective_mass_ =
    node->get_parameter("impedance_execution.min_effective_mass").as_double();
  imp_max_effective_mass_ =
    node->get_parameter("impedance_execution.max_effective_mass").as_double();
  imp_min_effective_inertia_ =
    node->get_parameter("impedance_execution.min_effective_inertia").as_double();
  imp_max_effective_inertia_ =
    node->get_parameter("impedance_execution.max_effective_inertia").as_double();
  imp_operational_damping_lambda_ =
    node->get_parameter("impedance_execution.operational_damping_lambda").as_double();
  imp_soft_limit_margin_ =
    node->get_parameter("impedance_execution.soft_limit_margin").as_double();
  imp_soft_limit_stiffness_ =
    node->get_parameter("impedance_execution.soft_limit_stiffness").as_double();
  imp_soft_limit_damping_ =
    node->get_parameter("impedance_execution.soft_limit_damping").as_double();
  imp_torque_slew_rate_ =
    node->get_parameter("impedance_execution.torque_slew_rate").as_double();
  auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
  auto nonneg = [](double v) { return std::isfinite(v) && v >= 0.0; };
  auto unit = [](double v) { return std::isfinite(v) && v >= 0.0 && v <= 1.0; };
  if (!positive(imp_max_trans_deviation_) ||
      !positive(imp_max_rot_deviation_) ||
      !positive(imp_max_joint_deviation_) ||
      !positive(imp_max_joint_velocity_) ||
      !nonneg(imp_hard_limit_tolerance_) ||
      !positive(imp_saturation_timeout_) ||
      !unit(imp_dynamics_ff_scale_) ||
      !unit(imp_fv_scale_) ||
      !unit(imp_fc_scale_) ||
      !positive(imp_fric_trans_speed_) ||
      !nonneg(imp_fric_quiet_speed_) ||
      imp_fric_quiet_speed_ >= imp_fric_trans_speed_ ||
      !positive(imp_filter_time_constant_) ||
      !positive(imp_min_translation_damping_) ||
      !positive(imp_min_rotation_damping_) ||
      !positive(imp_min_effective_mass_) ||
      !positive(imp_max_effective_mass_) ||
      imp_min_effective_mass_ > imp_max_effective_mass_ ||
      !positive(imp_min_effective_inertia_) ||
      !positive(imp_max_effective_inertia_) ||
      imp_min_effective_inertia_ > imp_max_effective_inertia_ ||
      !positive(imp_operational_damping_lambda_) ||
      !nonneg(imp_soft_limit_margin_) ||
      !nonneg(imp_soft_limit_stiffness_) ||
      !nonneg(imp_soft_limit_damping_) ||
      !positive(imp_torque_slew_rate_) ||
      !positive(imp_goal_trans_tolerance_) ||
      !positive(imp_goal_rot_tolerance_) ||
      !positive(imp_goal_velocity_tolerance_) ||
      !positive(imp_goal_settle_duration_) ||
      !positive(imp_goal_time_tolerance_) ||
      !positive(imp_entry_max_speed_) ||
      imp_entry_max_speed_ > lite6_interfaces::contract::kControllerEntryMaxSpeedRadPerSec ||
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
  imp_ref_jacobian_.setZero();
  impedance_trajectory_server_ = rclcpp_action::create_server<FollowImpedance>(
    node, "~/follow_impedance_trajectory",
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowImpedance::Goal> goal) {
      using CS = lite6_interfaces::msg::ControllerStatus;
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      const uint8_t mode = current_mode_atomic_.load(std::memory_order_acquire);
      if (!trajectory_enabled_.load(std::memory_order_acquire) ||
          compliance_blocks_trajectory_.load() ||
          mode != CS::MODE_NORMAL ||
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
  if (goal.joint_names != joint_names_ || goal.points.empty() ||
      !lite6_interfaces::valid_transition_time(goal.transition_time) ||
      (goal.end_behavior != FollowImpedance::Goal::END_TRANSITION_NORMAL &&
       goal.end_behavior != FollowImpedance::Goal::END_HOLD_CARTESIAN_IMPEDANCE)) {
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
    if (!execution->started) {
      execution->cancel_applied = true;
      return;
    }
    if (!execution->cancel_applied) {
      if (system_state_ == SystemState::CARTESIAN_IMPEDANCE_TRAJECTORY) {
        begin_deceleration_tracker();
        compliance_stopping_elapsed_ = 0.0;
        integral_effort_.setZero();
        transition_slew_active_ = false;
        transition_system_state(SystemState::COMPLIANCE_STOPPING);
      }
      execution->cancel_applied = true;
    }
    return;
  }

  auto abort_prestart = [&](
    ImpedanceTrajectoryResult reason, int joint, double value, double limit,
    const char * detail) {
    execution->failure_joint = joint;
    execution->failure_value = value;
    execution->failure_limit = limit;
    execution->failure_detail = detail;
    execution->result.store(reason);
  };

  const auto & goal = *execution->goal->get_goal();
  const auto & points = goal.points;
  if (!execution->started) {
    if (goal.expected_controller_epoch > 0 &&
        goal.expected_controller_epoch != controller_epoch_) {
      abort_prestart(
        ImpedanceTrajectoryResult::EPOCH_MISMATCH, -1,
        static_cast<double>(goal.expected_controller_epoch),
        static_cast<double>(controller_epoch_),
        "Expected controller_epoch does not match active controller epoch");
      return;
    }
    if (system_state_ != SystemState::NORMAL) {
      abort_prestart(
        ImpedanceTrajectoryResult::MODE_CHANGED, -1, 0.0, 0.0,
        "Controller must be in NORMAL mode to start impedance trajectory");
      return;
    }
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (std::abs(dq_[i]) > imp_entry_max_speed_) {
        abort_prestart(
          ImpedanceTrajectoryResult::VELOCITY_LIMIT, static_cast<int>(i),
          std::abs(dq_[i]), imp_entry_max_speed_,
          "Entry joint speed exceeds impedance_execution.entry_max_speed [rad/s]");
        return;
      }
      const double pos_diff = std::abs(points.front().positions[i] - q_[i]);
      if (pos_diff > imp_entry_position_tolerance_) {
        abort_prestart(
          ImpedanceTrajectoryResult::DEVIATION_LIMIT, static_cast<int>(i),
          pos_diff, imp_entry_position_tolerance_,
          "Initial impedance trajectory point deviates from measured joint pose [rad]");
        return;
      }
    }
    // Start the trajectory clock only after payload inertia smoothing has completed.
    if (payload_smoothing_active_) return;
    execution->started = true;
    execution->start_time_ns = time.nanoseconds();
    imp_active_transition_time_ = (goal.transition_time > 0.0) ?
      goal.transition_time :
      lite6_interfaces::contract::kDefaultImpedanceTransitionTimeSec;
    transition_system_state(SystemState::CARTESIAN_IMPEDANCE_TRAJECTORY);
    // Preserve last cycle's output torque for bumpless transition into impedance control.
    begin_torque_transition(imp_active_transition_time_);
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
    imp_null_stiffness_filtered_ = first.nullspace_stiffness;
    imp_null_damping_filtered_ = first.nullspace_damping;
    for (int k = 0; k < 6; ++k) {
      imp_wrench_ff_filtered_[k] = first.wrench_ff[static_cast<size_t>(k)];
    }
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
  imp_null_stiffness_target_ = point.nullspace_stiffness;
  imp_null_damping_target_ = point.nullspace_damping;
  for (int k = 0; k < 6; ++k) {
    imp_wrench_ff_target_[k] = point.wrench_ff[static_cast<size_t>(k)];
  }
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
  ensure_dynamics_cached();
  const double tau_filt = std::min(
    imp_filter_time_constant_,
    std::max(0.02, 0.5 * imp_active_transition_time_));
  const double alpha = 1.0 - std::exp(-dt / tau_filt);

  const Eigen::Vector3d cand_dkt = alpha * (imp_kt_target_ - imp_kt_filtered_);
  const Eigen::Vector3d cand_dkr = alpha * (imp_kr_target_ - imp_kr_filtered_);
  const double cand_dknull = alpha * (imp_null_stiffness_target_ - imp_null_stiffness_filtered_);

  imp_dt_filtered_ += alpha * (imp_dt_target_ - imp_dt_filtered_);
  imp_dr_filtered_ += alpha * (imp_dr_target_ - imp_dr_filtered_);
  imp_null_damping_filtered_ += alpha * (imp_null_damping_target_ - imp_null_damping_filtered_);
  imp_wrench_ff_filtered_ += alpha * (imp_wrench_ff_target_ - imp_wrench_ff_filtered_);

  imp_kt_filtered_ += cand_dkt;
  imp_kr_filtered_ += cand_dkr;
  imp_null_stiffness_filtered_ += cand_dknull;
  Vec6d tau_model = pin_data_dynamics_.g.head<6>();
  Vec6d tau_feedforward = Vec6d::Zero();
  a_d_.setZero();
  integral_effort_.setZero();

  // Moving references and dynamic feedforward supply energy for powered tracking.
  if (imp_dynamics_ff_scale_ > 1e-6) {
    Vec6d tau_dyn_acc =
      pin_data_dynamics_.M.topLeftCorner<6, 6>() * ddq_imp_ref_ +
      pin_data_dynamics_.C.topLeftCorner<6, 6>() * dq_imp_ref_;
    for (int i = 0; i < 6; ++i) {
      tau_dyn_acc[i] += armature_[i] * ddq_imp_ref_[i];
    }
    tau_model += imp_dynamics_ff_scale_ * tau_dyn_acc;
  }

  // 1. Evaluate time-varying equilibrium pose T_d(t) and twist v_d(t) from (q_imp_ref_, dq_imp_ref_).
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

  // 2. Evaluate measured pose T(q) and twist v(q, dq), reusing the current-cycle kinematic cache.
  const pinocchio::SE3 * meas_pose_ptr = nullptr;
  const Mat6x8d * meas_jac_ptr = nullptr;
  read_cached_tcp_kinematics(meas_pose_ptr, meas_jac_ptr);
  const auto & meas_pose = *meas_pose_ptr;
  const auto & meas_jac = *meas_jac_ptr;
  const Mat6d j_arm = meas_jac.leftCols<6>();
  const Vec6d meas_velocity = meas_jac * dynamics_dq_;

  Vec6d lambda_diag = Vec6d::Ones();
  compute_cartesian_effective_inertia(
    j_arm, imp_operational_damping_lambda_,
    imp_min_effective_mass_, imp_max_effective_mass_,
    imp_min_effective_inertia_, imp_max_effective_inertia_,
    lambda_diag);

  // 3. Compute SE(3) pose & twist errors in LOCAL_WORLD_ALIGNED frame.
  imp_current_pos_error_ = ref_pose.translation() - meas_pose.translation();
  imp_current_rot_error_ =
    pinocchio::log3(ref_pose.rotation() * meas_pose.rotation().transpose());
  const Eigen::Vector3d vel_trans_error = ref_velocity.head<3>() - meas_velocity.head<3>();
  const Eigen::Vector3d vel_rot_error = ref_velocity.tail<3>() - meas_velocity.tail<3>();
  const Vec6d dq_null_err = dq_ - dq_imp_ref_;

  tau_feedforward.noalias() += j_arm.transpose() * imp_wrench_ff_filtered_;
  // Passive M-weighted singularity damping on velocity tracking error.
  tau_model += compute_nullspace_damping_torque(
    j_arm, imp_operational_damping_lambda_, imp_null_damping_filtered_, dq_null_err);
  add_trajectory_friction_ff(
    imp_fv_scale_, imp_fc_scale_, imp_fric_trans_speed_, imp_fric_quiet_speed_,
    dq_imp_ref_, tau_feedforward);

  for (int i = 0; i < 6; ++i) {
    const double lo =
      std::min(pin_model_.lowerPositionLimit[i] + imp_soft_limit_margin_, q_imp_ref_[i]);
    const double hi =
      std::max(pin_model_.upperPositionLimit[i] - imp_soft_limit_margin_, q_imp_ref_[i]);
    if (q_[i] < lo) {
      const double v_in = std::min(0.0, dq_[i]);
      tau_model[i] += imp_soft_limit_stiffness_ * (lo - q_[i]) - imp_soft_limit_damping_ * v_in;
    }
    if (q_[i] > hi) {
      const double v_out = std::max(0.0, dq_[i]);
      tau_model[i] -= imp_soft_limit_stiffness_ * (q_[i] - hi) + imp_soft_limit_damping_ * v_out;
    }
  }

  // Constant damping ratio law: D_i(q) = max(D_min, 2 * zeta_i * sqrt(K_i * Lambda_i(q))).
  Vec6d wrench_imp;
  const Eigen::Vector3d rotation_torque = rotational_spring_torque(
    ref_pose.rotation() * meas_pose.rotation().transpose(), imp_kr_filtered_);
  for (int k = 0; k < 3; ++k) {
    const double d_trans = std::max(
      imp_min_translation_damping_,
      2.0 * imp_dt_filtered_[k] * std::sqrt(std::max(0.0, imp_kt_filtered_[k]) * lambda_diag[k]));
    const double d_rot = std::max(
      imp_min_rotation_damping_,
      2.0 * imp_dr_filtered_[k] * std::sqrt(std::max(0.0, imp_kr_filtered_[k]) * lambda_diag[k + 3]));
    wrench_imp[k] = imp_kt_filtered_[k] * imp_current_pos_error_[k] + d_trans * vel_trans_error[k];
    wrench_imp[k + 3] = rotation_torque[k] + d_rot * vel_rot_error[k];
  }
  tau_model.noalias() += j_arm.transpose() * wrench_imp;

  tau_cmd_ = tau_model + tau_feedforward;

  // Apply torque slew-rate limiting ONLY during the entry transition window.
  apply_transition_torque_slew(dt, imp_torque_slew_rate_, tau_cmd_);

  for (int i = 0; i < 6; ++i) {
    imp_effort_saturated_[static_cast<size_t>(i)] = std::abs(tau_cmd_[i]) > max_effort_[i];
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
    // (or ESTOP if interrupted/timed out), ensuring a stationary, reference-synced post-cancel state.
    if (execution->cancel_applied) {
      if (!execution->started || system_state_ == SystemState::NORMAL) {
        execution->result.store(ImpedanceTrajectoryResult::CANCELED);
      } else if (system_state_ == SystemState::ESTOP) {
        if (estop_reason_ ==
            lite6_interfaces::msg::ControllerStatus::ESTOP_COMPLIANCE_STOPPING_TIMEOUT) {
          execution->failure_value = lite6_interfaces::contract::kComplianceStoppingTimeoutSec;
          execution->failure_limit = lite6_interfaces::contract::kComplianceStoppingTimeoutSec;
          execution->failure_detail =
            "COMPLIANCE_STOPPING timed out during impedance trajectory cancellation";
          execution->result.store(ImpedanceTrajectoryResult::STOPPING_TIMEOUT);
        } else {
          execution->result.store(ImpedanceTrajectoryResult::CANCELED);
        }
      }
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

  if (!execution->endpoint_applied) {
    return;
  }

  const auto & goal = *execution->goal->get_goal();
  const double vel_ratio = dq_.cwiseAbs().maxCoeff() / imp_goal_velocity_tolerance_;
  double worst_ratio = vel_ratio;
  if (goal.end_behavior != FollowImpedance::Goal::END_HOLD_CARTESIAN_IMPEDANCE) {
    // Per-axis endpoint pose settling for END_TRANSITION_NORMAL:
    // - Pure force/torque axes (kt < 1.0 N/m or kr < 0.05 Nm/rad) have no spring
    //   equilibrium target and rely on global deviation caps + velocity settling.
    // - Soft or wrench_ff-biased axes widen their per-axis tolerance to cover
    //   (|wrench_ff_k| + static LuGre friction deadband) / k_k, capped at max_*_deviation.
    // - Stiff unbiased axes retain the exact nominal Euclidean tolerance.
    constexpr double kMinPosControlledTransStiffness = 1.0;   // N/m
    constexpr double kMinPosControlledRotStiffness = 0.05;    // Nm/rad
    constexpr double kStaticFrictionMarginScale = 1.20;       // LuGre Fs = 1.15 * Fc + margin

    Vec6d stiction_wrench = Vec6d::Zero();
    const pinocchio::SE3 * meas_pose_ptr = nullptr;
    const Mat6x8d * meas_jac_ptr = nullptr;
    read_cached_tcp_kinematics(meas_pose_ptr, meas_jac_ptr);
    const Mat6d j_arm = meas_jac_ptr->leftCols<6>();
    Mat6d jjt = j_arm * j_arm.transpose();
    const double lam_rot_sq = imp_operational_damping_lambda_ * imp_operational_damping_lambda_;
    const double lam_trans = cartesian_characteristic_length_ * imp_operational_damping_lambda_;
    const double lam_trans_sq = lam_trans * lam_trans;
    for (int k = 0; k < 3; ++k) {
      jjt(k, k) += lam_trans_sq;
      jjt(k + 3, k + 3) += lam_rot_sq;
    }
    const Eigen::LDLT<Mat6d> ldlt(jjt);
    const Vec6d tau_static = kStaticFrictionMarginScale * friction_c_;
    if (ldlt.info() == Eigen::Success) {
      const Mat6d j_pinv = j_arm.transpose() * ldlt.solve(Mat6d::Identity());
      for (int k = 0; k < 6; ++k) {
        const double rms = tau_static.cwiseProduct(j_pinv.col(k)).norm();
        if (std::isfinite(rms)) {
          stiction_wrench[k] = rms;
        }
      }
    }
    // Lower-bound by single-joint static breakaway so near-singular rows (where
    // DLS J_pinv shrinks toward zero) still reflect large static resistance.
    for (int k = 0; k < 6; ++k) {
      bool has_frictionless_joint = false;
      double max_sensitivity = 0.0;
      for (int i = 0; i < 6; ++i) {
        const double j_abs = std::abs(j_arm(k, i));
        if (j_abs <= 1e-9) {
          continue;
        }
        if (tau_static[i] <= 1e-9) {
          has_frictionless_joint = true;
          break;
        }
        max_sensitivity = std::max(max_sensitivity, j_abs / tau_static[i]);
      }
      if (!has_frictionless_joint) {
        const double breakaway = (max_sensitivity > 1e-9)
          ? (1.0 / max_sensitivity)
          : std::numeric_limits<double>::infinity();
        stiction_wrench[k] = std::max(stiction_wrench[k], breakaway);
      }
    }

    double nom_pos_err_sq = 0.0;
    double widened_pos_ratio = 0.0;
    double nom_rot_err_sq = 0.0;
    double widened_rot_ratio = 0.0;
    for (int k = 0; k < 3; ++k) {
      if (imp_kt_filtered_[k] >= kMinPosControlledTransStiffness) {
        const double static_tol =
          (std::abs(imp_wrench_ff_filtered_[k]) + stiction_wrench[k]) / imp_kt_filtered_[k];
        const double eff_tol = std::min(
          imp_max_trans_deviation_, std::max(imp_goal_trans_tolerance_, static_tol));
        if (eff_tol <= imp_goal_trans_tolerance_ + 1e-12) {
          nom_pos_err_sq += imp_current_pos_error_[k] * imp_current_pos_error_[k];
        } else {
          widened_pos_ratio = std::max(
            widened_pos_ratio, std::abs(imp_current_pos_error_[k]) / eff_tol);
        }
      }
      if (imp_kr_filtered_[k] >= kMinPosControlledRotStiffness) {
        const double static_tol =
          (std::abs(imp_wrench_ff_filtered_[k + 3]) + stiction_wrench[k + 3]) / imp_kr_filtered_[k];
        const double eff_tol = std::min(
          imp_max_rot_deviation_, std::max(imp_goal_rot_tolerance_, static_tol));
        if (eff_tol <= imp_goal_rot_tolerance_ + 1e-12) {
          nom_rot_err_sq += imp_current_rot_error_[k] * imp_current_rot_error_[k];
        } else {
          widened_rot_ratio = std::max(
            widened_rot_ratio, std::abs(imp_current_rot_error_[k]) / eff_tol);
        }
      }
    }
    const double pos_ratio = std::max(
      std::sqrt(nom_pos_err_sq) / imp_goal_trans_tolerance_, widened_pos_ratio);
    const double rot_ratio = std::max(
      std::sqrt(nom_rot_err_sq) / imp_goal_rot_tolerance_, widened_rot_ratio);
    worst_ratio = std::max({pos_ratio, rot_ratio, vel_ratio});
  }

  if (worst_ratio <= 1.0) {
    execution->settled_duration =
      (execution->was_settled && dt <= 0.1) ? execution->settled_duration + dt : 0.0;
    execution->was_settled = true;
  } else {
    execution->settled_duration = 0.0;
    execution->was_settled = false;
  }

  const double end_time =
    to_seconds(goal.points.back().time_from_start);
  if (execution->elapsed > end_time + imp_goal_time_tolerance_) {
    fail_impedance_trajectory(
      ImpedanceTrajectoryResult::GOAL_TIMEOUT, -1,
      worst_ratio, 1.0,
      "Impedance trajectory goal settling deadline exceeded");
  } else if (execution->settled_duration >= imp_goal_settle_duration_) {
    if (goal.end_behavior == FollowImpedance::Goal::END_HOLD_CARTESIAN_IMPEDANCE) {
      const auto & last = goal.points.back();
      dynamics_q_ref_.head<6>() = q_imp_ref_;
      dynamics_q_ref_.tail<2>() = dynamics_q_.tail<2>();
      pinocchio::forwardKinematics(pin_model_, pin_data_ref_, dynamics_q_ref_);
      compliance_pose_ = pinocchio::updateFramePlacement(
        pin_model_, pin_data_ref_, compliance_frame_);
      compliance_q_ = q_imp_ref_;
      // Continuous handover: keep the trajectory's anisotropic per-axis gains and final
      // feedforward wrench as the hold targets, starting from their current filtered
      // values, instead of averaging axes or dropping the wrench. Joint-impedance and
      // teach settings stay as last configured.
      for (int k = 0; k < 3; ++k) {
        cart_kt_target_[k] = last.translation_stiffness[k];
        cart_dt_target_[k] = last.translation_damping[k];
        cart_kr_target_[k] = last.rotation_stiffness[k];
        cart_dr_target_[k] = last.rotation_damping[k];
      }
      for (int k = 0; k < 6; ++k) {
        cart_wrench_ff_target_[k] = last.wrench_ff[static_cast<size_t>(k)];
      }
      cart_kt_filtered_ = imp_kt_filtered_;
      cart_dt_filtered_ = imp_dt_filtered_;
      cart_kr_filtered_ = imp_kr_filtered_;
      cart_dr_filtered_ = imp_dr_filtered_;
      cart_wrench_ff_filtered_ = imp_wrench_ff_filtered_;
      compliance_settings_.recapture = false;
      compliance_settings_.nullspace_stiffness = last.nullspace_stiffness;
      compliance_settings_.nullspace_damping = last.nullspace_damping;
      compliance_filtered_ = compliance_settings_;
      compliance_filtered_.nullspace_stiffness = imp_null_stiffness_filtered_;
      compliance_filtered_.nullspace_damping = imp_null_damping_filtered_;
      begin_torque_transition(cart_imp_cfg_.transition_duration);
      compliance_saturation_time_ = 0.0;
      compliance_saturation_estop_pending_ = false;
      integral_effort_.setZero();
      transition_system_state(SystemState::CARTESIAN_IMPEDANCE);
      execution->result.store(ImpedanceTrajectoryResult::SUCCEEDED);
    } else {
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
      tau_point_ff_.setZero();
      integral_effort_.setZero();
      begin_torque_transition(imp_active_transition_time_);
      clear_estop_reason();
      transition_system_state(SystemState::NORMAL);
      execution->result.store(ImpedanceTrajectoryResult::SUCCEEDED);
    }
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
      goal->publish_feedback(feedback);
    }
    return;
  }

  if (outcome == ImpedanceTrajectoryResult::CANCELED && !goal->is_canceling()) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (active_impedance_trajectory_ == execution) {
      retired_impedance_trajectories_.push_back(std::move(active_impedance_trajectory_));
      impedance_trajectory_buffer_.writeFromNonRT(
        std::shared_ptr<ImpedanceTrajectoryExecution>{});
      impedance_trajectory_reserved_ = false;
    }
  }

  auto result = std::make_shared<FollowImpedance::Result>();
  if (outcome == ImpedanceTrajectoryResult::CANCELED) {
    result->error_code = FollowImpedance::Result::SUCCESSFUL;
    result->error_string = "Impedance trajectory canceled and safely settled to NORMAL";
    goal->canceled(result);
  } else if (outcome == ImpedanceTrajectoryResult::SUCCEEDED) {
    result->error_code = FollowImpedance::Result::SUCCESSFUL;
    result->error_string =
      goal->get_goal()->end_behavior == FollowImpedance::Goal::END_HOLD_CARTESIAN_IMPEDANCE ?
      "Impedance trajectory completed; holding pose in CARTESIAN_IMPEDANCE" :
      "Impedance trajectory completed; reference synced to resting pose";
    goal->succeed(result);
  } else {
    switch (outcome) {
      case ImpedanceTrajectoryResult::INVALID_COMMAND:
        result->error_code = FollowImpedance::Result::INVALID_GOAL;
        break;
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
      case ImpedanceTrajectoryResult::GOAL_TIMEOUT:
        result->error_code = FollowImpedance::Result::GOAL_TIMEOUT;
        break;
      case ImpedanceTrajectoryResult::MODE_CHANGED:
        result->error_code = FollowImpedance::Result::MODE_CHANGED;
        break;
      case ImpedanceTrajectoryResult::EPOCH_MISMATCH:
        result->error_code = FollowImpedance::Result::CONTROLLER_EPOCH_MISMATCH;
        break;
      case ImpedanceTrajectoryResult::STOPPING_TIMEOUT:
        result->error_code = FollowImpedance::Result::STOPPING_TIMEOUT;
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
}

}  // namespace lite6_controllers
