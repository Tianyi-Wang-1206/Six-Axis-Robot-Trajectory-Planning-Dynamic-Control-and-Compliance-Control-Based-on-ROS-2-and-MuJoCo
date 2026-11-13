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
}  // namespace

bool Lite6CTCController::valid_trajectory_goal(const FollowTrajectory::Goal & goal) const
{
  const auto & trajectory = goal.trajectory;
  // This endpoint consumes the planner's already validated, fully sampled references.
  // Reject unsupported options rather than silently ignoring their semantics.
  if (trajectory.joint_names != joint_names_ || trajectory.points.empty() ||
      trajectory.header.stamp.sec != 0 || trajectory.header.stamp.nanosec != 0 ||
      !goal.multi_dof_trajectory.points.empty() ||
      !goal.multi_dof_trajectory.joint_names.empty() ||
      !goal.path_tolerance.empty() || !goal.goal_tolerance.empty() ||
      !goal.component_path_tolerance.empty() || !goal.component_goal_tolerance.empty() ||
      goal.goal_time_tolerance.sec != 0 || goal.goal_time_tolerance.nanosec != 0) return false;
  double previous = -1.0;
  for (const auto & point : trajectory.points) {
    const double t = seconds(point.time_from_start);
    if (point.time_from_start.sec < 0 || point.time_from_start.nanosec >= 1000000000u ||
        t <= previous || point.positions.size() != joint_names_.size() ||
        point.velocities.size() != joint_names_.size() ||
        point.accelerations.size() != joint_names_.size() || !point.effort.empty()) return false;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (!std::isfinite(point.positions[i]) || !std::isfinite(point.velocities[i]) ||
          !std::isfinite(point.accelerations[i])) return false;
    }
    previous = t;
  }
  if (seconds(trajectory.points.front().time_from_start) != 0.0) return false;
  // Holding the final reference must not leave a nonzero feed-forward command.
  const auto & last = trajectory.points.back();
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    if (std::abs(last.velocities[i]) > 1e-8 || std::abs(last.accelerations[i]) > 1e-8)
      return false;
  }
  return true;
}

void Lite6CTCController::configure_trajectory_action()
{
  trajectory_server_ = rclcpp_action::create_server<FollowTrajectory>(
    get_node(), "~/follow_joint_trajectory",
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowTrajectory::Goal> goal) {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      if (!trajectory_enabled_.load() || trajectory_reserved_ || !valid_trajectory_goal(*goal))
        return rclcpp_action::GoalResponse::REJECT;
      trajectory_reserved_ = true;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](const std::shared_ptr<TrajectoryGoal> goal) {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      if (!active_trajectory_ || active_trajectory_->goal != goal ||
          active_trajectory_->result.load() != TrajectoryResult::RUNNING)
        return rclcpp_action::CancelResponse::REJECT;
      active_trajectory_->cancel.store(true);
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this](const std::shared_ptr<TrajectoryGoal> goal) {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      active_trajectory_ = std::make_shared<TrajectoryExecution>(goal);
      if (!trajectory_enabled_.load()) {
        active_trajectory_->result.store(TrajectoryResult::DEACTIVATED);
      }
      trajectory_buffer_.writeFromNonRT(active_trajectory_);
    });
  trajectory_monitor_ = get_node()->create_wall_timer(
    std::chrono::milliseconds(20), [this] {monitor_trajectory();});
}

void Lite6CTCController::update_trajectory_reference(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // The non-RT buffer retains ownership; no allocations, action calls or blocking locks here.
  auto * execution = current_execution_;
  if (!execution || execution->result.load() != TrajectoryResult::RUNNING) return;
  const size_t n = joint_names_.size();
  if (execution->cancel.load()) {
    for (size_t i = 0; i < n; ++i) {
      reference_interfaces_[i] = q_[i];
      reference_interfaces_[n + i] = 0.0;
      reference_interfaces_[2 * n + i] = 0.0;
    }
    execution->cancel_applied = true;
    return;
  }
  if (execution->started) execution->elapsed += period.seconds();
  else {
    execution->started = true;
    execution->start_time_ns = time.nanoseconds();
    execution->start_mode = static_cast<int>(system_state_);
  }
  // Hold the exact endpoint while the physical robot settles; keep timing it.
  if (execution->endpoint_applied) return;

  const auto & points = execution->goal->get_goal()->trajectory.points;
  const auto next = std::upper_bound(points.begin(), points.end(), execution->elapsed,
    [](double t, const trajectory_msgs::msg::JointTrajectoryPoint & point) {
      return t < seconds(point.time_from_start);
    });
  const size_t index = static_cast<size_t>(std::distance(points.begin(), next) - 1);
  const auto & point = points[index];
  // Zero-order hold of the latest due sample: preserve Ruckig q/dq/ddq verbatim.
  for (size_t i = 0; i < n; ++i) {
    reference_interfaces_[i] = point.positions[i];
    reference_interfaces_[n + i] = point.velocities[i];
    reference_interfaces_[2 * n + i] = point.accelerations[i];
  }
  execution->sample.store(index);
  execution->endpoint_applied = index + 1 == points.size();
}

void Lite6CTCController::request_execution_stop()
{
  if (!execution_stop_latched_) {
    execution_stop_sequence_ = current_command_sequence_;
    execution_stop_latched_ = true;
  }
  if (system_state_ != SystemState::ESTOP) {
    // Invalid feedback is handled by fail_controller(), not fed into the brake tracker.
    if (q_.allFinite() && dq_.allFinite()) {
      estop_q_ = q_;
      estop_dq_ = dq_;
    }
    system_state_ = SystemState::ESTOP;
  }
  status_publish_pending_ = true;
}

void Lite6CTCController::fail_trajectory(
  TrajectoryResult reason, int joint, double value, double limit)
{
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
  integral_effort_.setZero();
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
    const double position_error = std::abs(reference_interfaces_[i] - q_[i]);
    const double velocity_error = std::abs(reference_interfaces_[n + i] - dq_[i]);
    if (position_error > path_position_tolerance_[i]) {
      fail_trajectory(TrajectoryResult::PATH_POSITION, static_cast<int>(i),
        position_error, path_position_tolerance_[i]);
      return;
    }
    if (velocity_error > path_velocity_tolerance_[i]) {
      fail_trajectory(TrajectoryResult::PATH_VELOCITY, static_cast<int>(i),
        velocity_error, path_velocity_tolerance_[i]);
      return;
    }
  }
}

void Lite6CTCController::finish_trajectory_cycle(double dt)
{
  auto * execution = current_execution_;
  if (!execution || execution->result.load() != TrajectoryResult::RUNNING) return;
  // Numerical/interface failures have already taken precedence over cancellation.
  if (execution->cancel.load()) {
    if (execution->cancel_applied) execution->result.store(TrajectoryResult::CANCELED);
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
  const auto & position_tolerance =
    execution->start_mode == static_cast<int>(SystemState::SYS_ID) ?
    sysid_goal_position_tolerance_ : goal_position_tolerance_;
  const auto & velocity_tolerance =
    execution->start_mode == static_cast<int>(SystemState::SYS_ID) ?
    sysid_goal_velocity_tolerance_ : goal_velocity_tolerance_;
  double worst_ratio = 0.0;
  int worst_joint = -1;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const double ratio = std::max(
      std::abs(reference_interfaces_[i] - q_[i]) / position_tolerance[i],
      std::abs(dq_[i]) / velocity_tolerance[i]);
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
  const double end_time = seconds(execution->goal->get_goal()->trajectory.points.back().time_from_start);
  // The deadline includes the continuous settling window, not just first entry.
  if (execution->elapsed > end_time + goal_time_tolerance_) {
    fail_trajectory(TrajectoryResult::GOAL_TIMEOUT, worst_joint, worst_ratio, 1.0);
  } else if (execution->settled_duration >= goal_settle_duration_) {
    execution->result.store(TrajectoryResult::SUCCEEDED);
  }
}

void Lite6CTCController::monitor_trajectory()
{
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  if (!active_trajectory_) return;
  const auto & execution = active_trajectory_;
  const auto outcome = execution->result.load();
  const auto & goal = execution->goal;
  if (outcome == TrajectoryResult::RUNNING) {
    if (goal->is_executing()) {
      auto feedback = std::make_shared<FollowTrajectory::Feedback>();
      feedback->header.stamp = get_node()->now();
      feedback->joint_names = joint_names_;
      feedback->desired = goal->get_goal()->trajectory.points[execution->sample.load()];
      goal->publish_feedback(feedback);
    }
    return;
  }
  auto result = std::make_shared<FollowTrajectory::Result>();
  if (outcome == TrajectoryResult::CANCELED) {
    // The cancel callback returns before rclcpp_action enters CANCELING.
    if (!goal->is_canceling()) return;
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
  active_trajectory_.reset();
  trajectory_reserved_ = false;
}
}  // namespace lite6_controllers
