#include "lite6_controllers/ctc_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace lite6_controllers
{
namespace
{
double seconds(const builtin_interfaces::msg::Duration & duration)
{
  return duration.sec + 1e-9 * duration.nanosec;
}

template <typename ResultEnum>
uint8_t map_trajectory_estop_reason(ResultEnum reason)
{
  using CS = lite6_interfaces::msg::ControllerStatus;
  switch (reason) {
    case ResultEnum::PATH_POSITION: return CS::ESTOP_TRAJECTORY_PATH_POSITION;
    case ResultEnum::PATH_VELOCITY: return CS::ESTOP_TRAJECTORY_PATH_VELOCITY;
    case ResultEnum::GOAL_TIMEOUT: return CS::ESTOP_TRAJECTORY_GOAL_TIMEOUT;
    case ResultEnum::SATURATION: return CS::ESTOP_TRAJECTORY_SATURATION;
    case ResultEnum::MODE_CHANGED: return CS::ESTOP_TRAJECTORY_MODE_CHANGED;
    case ResultEnum::INVALID_STATE: return CS::ESTOP_INVALID_STATE;
    case ResultEnum::INVALID_COMMAND: return CS::ESTOP_INVALID_COMMAND;
    case ResultEnum::COMMAND_WRITE: return CS::ESTOP_COMMAND_WRITE;
    case ResultEnum::CONTROL_EXCEPTION: return CS::ESTOP_CONTROL_EXCEPTION;
    default: return CS::ESTOP_EXTERNAL;
  }
}
}  // namespace

bool Lite6CTCController::valid_trajectory_goal(
  const FollowTrajectory::Goal & goal, bool splice) const
{
  const auto & trajectory = goal.trajectory;
  // A regular goal starts immediately (zero stamp). A splice goal carries the actual
  // start time of the trajectory it continues (standard JTC header.stamp semantics).
  const bool zero_stamp =
    trajectory.header.stamp.sec == 0 && trajectory.header.stamp.nanosec == 0;
  if (trajectory.joint_names != joint_names_ || trajectory.points.empty() ||
      zero_stamp == splice ||
      !goal.multi_dof_trajectory.points.empty() ||
      !goal.multi_dof_trajectory.joint_names.empty() ||
      !goal.component_path_tolerance.empty() || !goal.component_goal_tolerance.empty())
  {
    return false;
  }

  auto valid_joint_tolerances = [this](const std::vector<control_msgs::msg::JointTolerance> & tols) {
    if (tols.size() > joint_names_.size()) return false;
    for (const auto & tol : tols) {
      const auto it = std::find(joint_names_.begin(), joint_names_.end(), tol.name);
      if (it == joint_names_.end()) return false;
      if (!std::isfinite(tol.position) || tol.position < 0.0 ||
          !std::isfinite(tol.velocity) || tol.velocity < 0.0 ||
          !std::isfinite(tol.acceleration) || tol.acceleration < 0.0)
      {
        return false;
      }
    }
    return true;
  };
  if (!valid_joint_tolerances(goal.path_tolerance) ||
      !valid_joint_tolerances(goal.goal_tolerance))
  {
    return false;
  }
  if (goal.goal_time_tolerance.sec < 0 || goal.goal_time_tolerance.nanosec >= 1000000000u) {
    return false;
  }
  const double custom_goal_time = seconds(goal.goal_time_tolerance);
  if (custom_goal_time > 0.0 &&
      (custom_goal_time < goal_settle_duration_ || custom_goal_time > 10.0))
  {
    return false;
  }

  double previous = -1.0;
  for (const auto & point : trajectory.points) {
    const double t = seconds(point.time_from_start);
    if (point.time_from_start.sec < 0 || point.time_from_start.nanosec >= 1000000000u ||
        t <= previous || point.positions.size() != joint_names_.size() ||
        point.velocities.size() != joint_names_.size() ||
        point.accelerations.size() != joint_names_.size() ||
        (!point.effort.empty() && point.effort.size() != joint_names_.size()))
    {
      return false;
    }
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (!std::isfinite(point.positions[i]) || !std::isfinite(point.velocities[i]) ||
          !std::isfinite(point.accelerations[i]))
      {
        return false;
      }
      if (!point.effort.empty()) {
        if (!std::isfinite(point.effort[i]) ||
            std::abs(point.effort[i]) > max_effort_[static_cast<Eigen::Index>(i)])
        {
          return false;
        }
      }
    }
    previous = t;
  }
  // A splice starts at its switch time on the parent's time base, never at zero.
  const double first_time = seconds(trajectory.points.front().time_from_start);
  if (splice ? !(first_time > 0.0) : first_time != 0.0) return false;
  // Holding the final reference must not leave a nonzero feed-forward command.
  const auto & last = trajectory.points.back();
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    if (std::abs(last.velocities[i]) > 1e-8 || std::abs(last.accelerations[i]) > 1e-8)
      return false;
    if (!last.effort.empty() && std::abs(last.effort[i]) > 1e-8)
      return false;
  }
  return true;
}

bool Lite6CTCController::valid_splice_goal(const FollowTrajectory::Goal & goal) const
{
  // Caller holds trajectory_mutex_.
  const auto & parent = active_trajectory_;
  if (!parent || parent->result.load() != TrajectoryResult::RUNNING ||
      parent->cancel.load() || parent->splice_reserved ||
      !valid_trajectory_goal(goal, true))
  {
    return false;
  }
  const int64_t parent_start = parent->start_time_pub_ns.load(std::memory_order_acquire);
  const auto & stamp = goal.trajectory.header.stamp;
  const int64_t stamp_ns =
    static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
  if (parent_start <= 0 || stamp_ns != parent_start) {
    return false;
  }
  // The splice must not override the parent's endpoint tolerances or timing semantics.
  if (!goal.path_tolerance.empty() || !goal.goal_tolerance.empty() ||
      goal.goal_time_tolerance.sec != 0 || goal.goal_time_tolerance.nanosec != 0)
  {
    return false;
  }
  // Seamless switch: the first splice sample must equal the parent sample at the same
  // time (Studio copies it verbatim), and must lie strictly before the parent's end.
  const auto & parent_points = parent->goal->get_goal()->trajectory.points;
  const auto & first = goal.trajectory.points.front();
  const double t_switch = seconds(first.time_from_start);
  const auto it = std::lower_bound(
    parent_points.begin(), parent_points.end(), t_switch - 1e-7,
    [](const trajectory_msgs::msg::JointTrajectoryPoint & p, double t) {
      return seconds(p.time_from_start) < t;
    });
  if (it == parent_points.end() || std::next(it) == parent_points.end() ||
      std::abs(seconds(it->time_from_start) - t_switch) > 1e-7)
  {
    return false;
  }
  if (it->effort.empty() != first.effort.empty()) {
    return false;
  }
  constexpr double kContinuityTol = 1e-9;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    if (std::abs(it->positions[i] - first.positions[i]) > kContinuityTol ||
        std::abs(it->velocities[i] - first.velocities[i]) > kContinuityTol ||
        std::abs(it->accelerations[i] - first.accelerations[i]) > kContinuityTol ||
        (!first.effort.empty() && std::abs(it->effort[i] - first.effort[i]) > kContinuityTol))
    {
      return false;
    }
  }
  return true;
}

void Lite6CTCController::populate_goal_tolerances(
  const FollowTrajectory::Goal & goal, TrajectoryExecution & exec) const
{
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    exec.path_pos_tol[i] = path_position_tolerance_[i];
    exec.path_vel_tol[i] = path_velocity_tolerance_[i];
    exec.goal_pos_tol[i] = goal_position_tolerance_[i];
    exec.goal_vel_tol[i] = goal_velocity_tolerance_[i];
  }
  exec.goal_time_tol = goal_time_tolerance_;

  for (const auto & tol : goal.path_tolerance) {
    const auto it = std::find(joint_names_.begin(), joint_names_.end(), tol.name);
    if (it != joint_names_.end()) {
      const size_t idx = static_cast<size_t>(std::distance(joint_names_.begin(), it));
      if (tol.position > 0.0) exec.path_pos_tol[idx] = tol.position;
      if (tol.velocity > 0.0) exec.path_vel_tol[idx] = tol.velocity;
    }
  }
  for (const auto & tol : goal.goal_tolerance) {
    const auto it = std::find(joint_names_.begin(), joint_names_.end(), tol.name);
    if (it != joint_names_.end()) {
      const size_t idx = static_cast<size_t>(std::distance(joint_names_.begin(), it));
      if (tol.position > 0.0) {
        exec.goal_pos_tol[idx] = std::min(tol.position, exec.path_pos_tol[idx]);
      }
      if (tol.velocity > 0.0) {
        exec.goal_vel_tol[idx] = std::min(tol.velocity, exec.path_vel_tol[idx]);
      }
    }
  }
  const double custom_goal_time = seconds(goal.goal_time_tolerance);
  if (custom_goal_time >= goal_settle_duration_ && custom_goal_time <= 10.0) {
    exec.goal_time_tol = custom_goal_time;
  }
}

void Lite6CTCController::configure_trajectory_action()
{
  trajectory_server_ = rclcpp_action::create_server<FollowTrajectory>(
    get_node(), "~/follow_joint_trajectory",
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowTrajectory::Goal> goal) {
      using CS = lite6_interfaces::msg::ControllerStatus;
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      const uint8_t mode = current_mode_atomic_.load(std::memory_order_acquire);
      const auto & stamp = goal->trajectory.header.stamp;
      if (stamp.sec != 0 || stamp.nanosec != 0) {
        // Splice goal: continues (and partially replaces) the active rigid trajectory.
        if (!trajectory_enabled_.load(std::memory_order_acquire) ||
            !valid_splice_goal(*goal))
        {
          return rclcpp_action::GoalResponse::REJECT;
        }
        active_trajectory_->splice_reserved = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }
      if (!trajectory_enabled_.load(std::memory_order_acquire) ||
          compliance_blocks_trajectory_.load() ||
          (mode != CS::MODE_NORMAL && mode != CS::MODE_SYSID) ||
          trajectory_reserved_ || impedance_trajectory_reserved_ ||
          !valid_trajectory_goal(*goal))
        return rclcpp_action::GoalResponse::REJECT;
      trajectory_reserved_ = true;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](const std::shared_ptr<TrajectoryGoal> goal) {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      // A splice is resolved by the RT thread at its switch time; it cannot be canceled.
      // Cancel the parent goal instead.
      if (!active_trajectory_ || active_trajectory_->goal != goal ||
          active_trajectory_->result.load() != TrajectoryResult::RUNNING)
        return rclcpp_action::CancelResponse::REJECT;
      active_trajectory_->cancel.store(true);
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this](const std::shared_ptr<TrajectoryGoal> goal) {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      const auto & stamp = goal->get_goal()->trajectory.header.stamp;
      if (stamp.sec != 0 || stamp.nanosec != 0) {
        auto child = std::make_shared<TrajectoryExecution>(goal);
        child->is_splice = true;
        child->splice_switch_time =
          seconds(goal->get_goal()->trajectory.points.front().time_from_start);
        pending_splices_.push_back(child);
        const auto & parent = active_trajectory_;
        if (!trajectory_enabled_.load() || !parent || !parent->splice_reserved ||
            parent->splice_owner || parent->result.load() != TrajectoryResult::RUNNING)
        {
          child->result.store(TrajectoryResult::SPLICE_REJECTED);
          return;
        }
        child->splice_parent = parent;
        parent->splice_owner = child;
        // Publish only after ownership is established; the RT thread adopts or rejects it.
        parent->splice.store(child.get(), std::memory_order_release);
        return;
      }
      if (active_trajectory_) {
        retired_trajectories_.push_back(std::move(active_trajectory_));
      }
      active_trajectory_ = std::make_shared<TrajectoryExecution>(goal);
      populate_goal_tolerances(*goal->get_goal(), *active_trajectory_);
      if (!trajectory_enabled_.load()) {
        active_trajectory_->result.store(TrajectoryResult::DEACTIVATED);
      }
      trajectory_buffer_.writeFromNonRT(active_trajectory_);
    });
  trajectory_monitor_ = get_node()->create_wall_timer(
    std::chrono::milliseconds(20), [this] {
      monitor_trajectory();
      monitor_splices();
      monitor_impedance_trajectory();
    });
}

void Lite6CTCController::update_trajectory_reference(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // The non-RT buffer retains ownership; no allocations, action calls or blocking locks here.
  auto * execution = current_execution_;
  if (!execution || execution->result.load() != TrajectoryResult::RUNNING) {
    tau_point_ff_.setZero();
    return;
  }
  const size_t n = joint_names_.size();
  if (execution->cancel.load()) {
    tau_point_ff_.setZero();
    if (!execution->started) {
      execution->cancel_applied = true;
      return;
    }
    if (!execution->cancel_applied) {
      begin_deceleration_tracker();
      rigid_cancel_braking_ = true;
      rigid_cancel_elapsed_ = 0.0;
      execution->cancel_applied = true;
    }
    rigid_cancel_elapsed_ += period.seconds();
    step_deceleration_tracker(period.seconds());
    for (size_t i = 0; i < n; ++i) {
      reference_interfaces_[i] = estop_q_[i];
      reference_interfaces_[n + i] = estop_dq_[i];
      reference_interfaces_[2 * n + i] = ddq_target_[i];
    }
    return;
  }
  if (execution->started) {
    execution->elapsed += period.seconds();
  } else {
    if (system_state_ != SystemState::NORMAL && system_state_ != SystemState::SYS_ID) {
      execution->result.store(TrajectoryResult::MODE_CHANGED);
      tau_point_ff_.setZero();
      return;
    }
    // Start the trajectory clock only after the configured payload model has settled.
    if (payload_smoothing_active_) return;
    execution->started = true;
    execution->start_time_ns = time.nanoseconds();
    execution->start_time_pub_ns.store(execution->start_time_ns, std::memory_order_release);
    execution->start_mode = static_cast<int>(system_state_);
  }

  // Path-preserving splice: decided once on first sight (rejected if its switch time
  // has already passed or the endpoint is already held), then adopted exactly at its
  // switch time. Its first sample equals the parent sample at that time, so the
  // reference switches without a discontinuity.
  if (!execution->splice_resolved) {
    auto * splice = execution->splice.load(std::memory_order_acquire);
    if (splice != nullptr) {
      auto running = TrajectoryResult::RUNNING;
      if (!execution->splice_seen) {
        execution->splice_seen = true;
        if (execution->endpoint_applied || execution->elapsed > splice->splice_switch_time) {
          execution->splice_resolved = true;
          splice->result.compare_exchange_strong(running, TrajectoryResult::SPLICE_REJECTED);
        }
      }
      if (!execution->splice_resolved && execution->elapsed >= splice->splice_switch_time &&
          splice->result.compare_exchange_strong(running, TrajectoryResult::SUCCEEDED))
      {
        execution->splice_resolved = true;
        execution->tail = splice;
        execution->last_sample_index = 0;
        execution->endpoint_applied = false;
        execution->on_tail.store(true, std::memory_order_release);
      }
    }
  }

  // Hold the exact endpoint while the physical robot settles; keep timing it.
  if (execution->endpoint_applied) {
    tau_point_ff_.setZero();
    return;
  }

  const auto & points = (execution->tail != nullptr) ?
    execution->tail->goal->get_goal()->trajectory.points :
    execution->goal->get_goal()->trajectory.points;
  size_t index = execution->last_sample_index;
  if (index >= points.size()) index = 0;
  while (index + 1 < points.size() &&
         seconds(points[index + 1].time_from_start) <= execution->elapsed) {
    ++index;
  }
  execution->last_sample_index = index;
  const auto & point = points[index];
  // Zero-order hold of the latest due sample: preserve Ruckig q/dq/ddq verbatim.
  for (size_t i = 0; i < n; ++i) {
    reference_interfaces_[i] = point.positions[i];
    reference_interfaces_[n + i] = point.velocities[i];
    reference_interfaces_[2 * n + i] = point.accelerations[i];
    tau_point_ff_[static_cast<Eigen::Index>(i)] =
      point.effort.size() == n ? point.effort[i] : 0.0;
  }
  execution->sample.store(index);
  execution->endpoint_applied = index + 1 == points.size();
}

void Lite6CTCController::latch_estop_reason(
  uint8_t reason, int joint, double value, double limit)
{
  // Keep the first latched cause until the controller leaves ESTOP.
  if (estop_reason_ != lite6_interfaces::msg::ControllerStatus::ESTOP_NONE) return;
  estop_reason_ = reason;
  estop_joint_ = static_cast<int8_t>(joint);
  estop_value_ = value;
  estop_limit_ = limit;
}

void Lite6CTCController::clear_estop_reason()
{
  estop_reason_ = lite6_interfaces::msg::ControllerStatus::ESTOP_NONE;
  estop_joint_ = -1;
  estop_value_ = 0.0;
  estop_limit_ = 0.0;
  compliance_saturation_estop_pending_ = false;
  pending_estop_joint_ = -1;
  pending_estop_value_ = 0.0;
  pending_estop_limit_ = 0.0;
}

void Lite6CTCController::request_execution_stop()
{
  rigid_cancel_braking_ = false;
  rigid_cancel_elapsed_ = 0.0;
  tau_point_ff_.setZero();
  if (system_state_ != SystemState::ESTOP) {
    // Invalid feedback is handled by fail_controller(), not fed into the brake tracker.
    begin_deceleration_tracker();
    transition_system_state(SystemState::ESTOP);
  }
  status_publish_pending_ = true;
}

void Lite6CTCController::fail_trajectory(
  TrajectoryResult reason, int joint, double value, double limit)
{
  latch_estop_reason(map_trajectory_estop_reason(reason), joint, value, limit);
  request_execution_stop();
  auto * execution = current_execution_;
  if (!execution || execution->result.load() != TrajectoryResult::RUNNING) return;
  execution->failure_joint = joint;
  execution->failure_value = value;
  execution->failure_limit = limit;
  execution->result.store(reason);
}

controller_interface::return_type Lite6CTCController::fail_controller(
  TrajectoryResult reason, int joint)
{
  trajectory_enabled_.store(false);
  fail_trajectory(reason, joint);
  ImpedanceTrajectoryResult imp_reason = ImpedanceTrajectoryResult::CONTROL_EXCEPTION;
  if (reason == TrajectoryResult::INVALID_STATE) {
    imp_reason = ImpedanceTrajectoryResult::INVALID_STATE;
  } else if (reason == TrajectoryResult::INVALID_COMMAND) {
    imp_reason = ImpedanceTrajectoryResult::INVALID_COMMAND;
  } else if (reason == TrajectoryResult::COMMAND_WRITE) {
    imp_reason = ImpedanceTrajectoryResult::COMMAND_WRITE;
  }
  fail_impedance_trajectory(imp_reason, joint);
  integral_effort_.setZero();
  tau_point_ff_.setZero();
  tau_cmd_.setZero();
  // Best effort only: an inaccessible interface cannot be made writable here.
  // ERROR asks controller_manager to deactivate this controller as well.
  for (auto & command : command_interfaces_) {
    try { (void)command.set_value(0.0); } catch (const std::exception &) {}
  }
  return controller_interface::return_type::ERROR;
}

void Lite6CTCController::check_trajectory_path()
{
  auto * execution = current_execution_;
  if (!execution || execution->result.load() != TrajectoryResult::RUNNING ||
      execution->cancel.load() || !execution->started) return;
  // A trajectory admitted while already in ESTOP is the existing recovery
  // reference-sync operation. Check its references, not the braking targets.
  if (execution->start_mode != static_cast<int>(system_state_)) {
    fail_trajectory(TrajectoryResult::MODE_CHANGED);
    return;
  }
  const size_t n = joint_names_.size();
  for (size_t i = 0; i < n; ++i) {
    const double pos_tol = execution->path_pos_tol[i] > 0.0 ?
      execution->path_pos_tol[i] : path_position_tolerance_[i];
    const double vel_tol = execution->path_vel_tol[i] > 0.0 ?
      execution->path_vel_tol[i] : path_velocity_tolerance_[i];
    const double position_error = std::abs(reference_interfaces_[i] - q_[i]);
    const double velocity_error = std::abs(reference_interfaces_[n + i] - dq_[i]);
    if (position_error > pos_tol) {
      fail_trajectory(TrajectoryResult::PATH_POSITION, static_cast<int>(i),
        position_error, pos_tol);
      return;
    }
    if (velocity_error > vel_tol) {
      fail_trajectory(TrajectoryResult::PATH_VELOCITY, static_cast<int>(i),
        velocity_error, vel_tol);
      return;
    }
  }
}

void Lite6CTCController::finish_trajectory_cycle(double dt)
{
  auto * execution = current_execution_;
  if (!execution || execution->result.load() != TrajectoryResult::RUNNING) return;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const Eigen::Index idx = static_cast<Eigen::Index>(i);
    execution->actual_pos[i].store(q_[idx], std::memory_order_relaxed);
    execution->actual_vel[i].store(dq_[idx], std::memory_order_relaxed);
    execution->actual_eff[i].store(tau_cmd_[idx], std::memory_order_relaxed);
    execution->error_pos[i].store(e_[idx], std::memory_order_relaxed);
    execution->error_vel[i].store(de_[idx], std::memory_order_relaxed);
  }

  // Numerical/interface failures have already taken precedence over cancellation.
  if (execution->cancel.load()) {
    if (execution->cancel_applied) {
      const bool settled =
        !execution->started ||
        (estop_dq_.cwiseAbs().maxCoeff() < 1e-4 &&
         dq_.cwiseAbs().maxCoeff() <= lite6_interfaces::contract::kPlannerSettleSpeedRadPerSec);
      const bool mode_left =
        execution->started && system_state_ != static_cast<SystemState>(execution->start_mode);
      if (settled || mode_left) {
        rigid_cancel_braking_ = false;
        rigid_cancel_elapsed_ = 0.0;
        tau_point_ff_.setZero();
        if (system_state_ == SystemState::NORMAL || system_state_ == SystemState::SYS_ID) {
          const size_t n = joint_names_.size();
          for (size_t i = 0; i < n; ++i) {
            reference_interfaces_[i] = q_[i];
            reference_interfaces_[n + i] = 0.0;
            reference_interfaces_[2 * n + i] = 0.0;
          }
        }
        execution->result.store(TrajectoryResult::CANCELED);
      } else if (rigid_cancel_elapsed_ >= lite6_interfaces::contract::kComplianceStoppingTimeoutSec) {
        rigid_cancel_braking_ = false;
        rigid_cancel_elapsed_ = 0.0;
        fail_trajectory(TrajectoryResult::PATH_VELOCITY);
      }
    }
    return;
  }
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    auto & duration = execution->saturation_duration[i];
    duration = effort_saturated_[i] ? duration + dt : 0.0;
    if (duration >= saturation_timeout_) {
      fail_trajectory(TrajectoryResult::SATURATION, static_cast<int>(i),
        duration, saturation_timeout_);
      return;
    }
  }
  if (!execution->endpoint_applied) return;

  // SYS_ID deliberately removes friction compensation and integral action, so
  // static friction can leave a steady position error. Use its own bounded
  // endpoint tolerance for excitation and post-alignment; NORMAL pre/return
  // alignment keeps the precision tolerance. Mode changes still abort above.
  const bool is_sysid = (execution->start_mode == static_cast<int>(SystemState::SYS_ID));
  double worst_ratio = 0.0;
  int worst_joint = -1;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const double pos_tol = is_sysid ? sysid_goal_position_tolerance_[i] :
      (execution->goal_pos_tol[i] > 0.0 ? execution->goal_pos_tol[i] : goal_position_tolerance_[i]);
    const double vel_tol = is_sysid ? sysid_goal_velocity_tolerance_[i] :
      (execution->goal_vel_tol[i] > 0.0 ? execution->goal_vel_tol[i] : goal_velocity_tolerance_[i]);
    const double ratio = std::max(
      std::abs(reference_interfaces_[i] - q_[i]) / pos_tol,
      std::abs(dq_[i]) / vel_tol);
    if (ratio > worst_ratio) {worst_ratio = ratio; worst_joint = static_cast<int>(i);}
  }
  // Do not credit an unobserved scheduling gap as continuously settled time.
  if (worst_ratio <= 1.0) {
    execution->settled_duration = execution->was_settled && dt <= 0.1 ?
      execution->settled_duration + dt : 0.0;
    execution->was_settled = true;
  } else {
    execution->settled_duration = 0.0;
    execution->was_settled = false;
  }
  const auto & end_source = (execution->tail != nullptr) ? execution->tail->goal : execution->goal;
  const double end_time = seconds(end_source->get_goal()->trajectory.points.back().time_from_start);
  const double time_tol = execution->goal_time_tol > 0.0 ?
    execution->goal_time_tol : goal_time_tolerance_;
  // The deadline includes the continuous settling window, not just first entry.
  if (execution->elapsed > end_time + time_tol) {
    fail_trajectory(TrajectoryResult::GOAL_TIMEOUT, worst_joint, worst_ratio, 1.0);
  } else if (execution->settled_duration >= goal_settle_duration_) {
    execution->result.store(TrajectoryResult::SUCCEEDED);
  }
}

void Lite6CTCController::monitor_trajectory()
{
  std::shared_ptr<TrajectoryExecution> execution;
  std::shared_ptr<TrajectoryExecution> splice_child;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (active_trajectory_) {
      splice_child = active_trajectory_->splice_owner;
    }
    if (!active_trajectory_ && !retired_trajectories_.empty() &&
        rt_seen_execution_.load(std::memory_order_acquire) == nullptr)
    {
      trajectory_buffer_.writeFromNonRT(std::shared_ptr<TrajectoryExecution>{});
    }
    const auto * rt_seen = rt_seen_execution_.load(std::memory_order_acquire);
    retired_trajectories_.erase(
      std::remove_if(
        retired_trajectories_.begin(), retired_trajectories_.end(),
        [rt_seen](const std::shared_ptr<TrajectoryExecution> & ptr) {
          return ptr.use_count() == 1 && ptr.get() != rt_seen;
        }),
      retired_trajectories_.end());
    execution = active_trajectory_;
  }
  if (!execution) return;
  const auto outcome = execution->result.load();
  const auto & goal = execution->goal;
  if (outcome == TrajectoryResult::RUNNING) {
    if (goal->is_executing()) {
      auto feedback = std::make_shared<FollowTrajectory::Feedback>();
      feedback->header.stamp = node_clock_ ? node_clock_->now() : get_node()->now();
      feedback->joint_names = joint_names_;
      const auto & fb_points =
        (splice_child && execution->on_tail.load(std::memory_order_acquire)) ?
        splice_child->goal->get_goal()->trajectory.points :
        goal->get_goal()->trajectory.points;
      const size_t sample_idx = std::min(execution->sample.load(), fb_points.size() - 1);
      feedback->desired = fb_points[sample_idx];
      const size_t n = joint_names_.size();
      feedback->actual.time_from_start = feedback->desired.time_from_start;
      feedback->actual.positions.resize(n);
      feedback->actual.velocities.resize(n);
      feedback->actual.effort.resize(n);
      feedback->error.time_from_start = feedback->desired.time_from_start;
      feedback->error.positions.resize(n);
      feedback->error.velocities.resize(n);
      for (size_t i = 0; i < n; ++i) {
        feedback->actual.positions[i] = execution->actual_pos[i].load(std::memory_order_relaxed);
        feedback->actual.velocities[i] = execution->actual_vel[i].load(std::memory_order_relaxed);
        feedback->actual.effort[i] = execution->actual_eff[i].load(std::memory_order_relaxed);
        feedback->error.positions[i] = execution->error_pos[i].load(std::memory_order_relaxed);
        feedback->error.velocities[i] = execution->error_vel[i].load(std::memory_order_relaxed);
      }
      goal->publish_feedback(feedback);
    }
    return;
  }
  if (outcome == TrajectoryResult::CANCELED && !goal->is_canceling()) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (active_trajectory_ == execution) {
      retired_trajectories_.push_back(std::move(active_trajectory_));
      trajectory_buffer_.writeFromNonRT(std::shared_ptr<TrajectoryExecution>{});
      trajectory_reserved_ = false;
    }
  }
  auto result = std::make_shared<FollowTrajectory::Result>();
  if (outcome == TrajectoryResult::CANCELED) {
    result->error_code = FollowTrajectory::Result::SUCCESSFUL;
    result->error_string = "Trajectory canceled";
    goal->canceled(result);
  } else if (outcome == TrajectoryResult::SUCCEEDED) {
    result->error_code = FollowTrajectory::Result::SUCCESSFUL;
    goal->succeed(result);
  } else {
    result->error_code = outcome == TrajectoryResult::GOAL_TIMEOUT ?
      FollowTrajectory::Result::GOAL_TOLERANCE_VIOLATED :
      FollowTrajectory::Result::PATH_TOLERANCE_VIOLATED;
    switch (outcome) {
      case TrajectoryResult::PATH_POSITION:
        result->error_string = "Path position tolerance violated [rad]"; break;
      case TrajectoryResult::PATH_VELOCITY:
        result->error_string = "Path velocity tolerance violated [rad/s]"; break;
      case TrajectoryResult::GOAL_TIMEOUT:
        result->error_string = "Goal tolerance deadline exceeded: position/velocity must remain "
          "within tolerance for " + std::to_string(goal_settle_duration_) +
          " s; normalized endpoint error"; break;
      case TrajectoryResult::SATURATION:
        result->error_string = "Continuous effort saturation timeout [s]"; break;
      case TrajectoryResult::INVALID_STATE:
        result->error_string = "Non-finite position/velocity feedback; controller stopped"; break;
      case TrajectoryResult::INVALID_COMMAND:
        result->error_string = "Non-finite control calculation; controller stopped"; break;
      case TrajectoryResult::COMMAND_WRITE:
        result->error_string = "Effort interface write failed; controller stopped"; break;
      case TrajectoryResult::MODE_CHANGED:
        result->error_string = "Controller mode changed during trajectory execution"; break;
      case TrajectoryResult::CONTROL_EXCEPTION:
        result->error_string = "Exception in control update; controller stopped"; break;
      default:
        result->error_string = "CTC controller deactivated"; break;
    }
    if (execution->failure_joint >= 0) {
      result->error_string += "; " + joint_names_[execution->failure_joint] +
        ", observed=" + std::to_string(execution->failure_value) +
        ", limit=" + std::to_string(execution->failure_limit);
    }
    goal->abort(result);
  }
}
void Lite6CTCController::monitor_splices()
{
  std::vector<std::shared_ptr<TrajectoryExecution>> finished;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    for (auto it = pending_splices_.begin(); it != pending_splices_.end();) {
      auto & child = *it;
      if (child->result.load() == TrajectoryResult::RUNNING) {
        // The RT thread only adopts while the parent runs. Once the parent is terminal
        // (or gone) an unadopted splice can never take effect.
        const auto parent = child->splice_parent.lock();
        if (!parent || parent->result.load() != TrajectoryResult::RUNNING) {
          auto running = TrajectoryResult::RUNNING;
          child->result.compare_exchange_strong(running, TrajectoryResult::SPLICE_REJECTED);
        }
      }
      if (child->result.load() != TrajectoryResult::RUNNING) {
        finished.push_back(child);
        it = pending_splices_.erase(it);
      } else {
        ++it;
      }
    }
  }
  for (const auto & child : finished) {
    auto result = std::make_shared<FollowTrajectory::Result>();
    if (child->result.load() == TrajectoryResult::SUCCEEDED) {
      result->error_code = FollowTrajectory::Result::SUCCESSFUL;
      result->error_string = "Splice adopted; the parent goal now follows the spliced reference";
      child->goal->succeed(result);
    } else {
      result->error_code = FollowTrajectory::Result::OLD_HEADER_TIMESTAMP;
      result->error_string =
        "Splice rejected: switch time already passed, endpoint reached, or parent goal ended";
      child->goal->abort(result);
    }
  }
}

}  // namespace lite6_controllers
