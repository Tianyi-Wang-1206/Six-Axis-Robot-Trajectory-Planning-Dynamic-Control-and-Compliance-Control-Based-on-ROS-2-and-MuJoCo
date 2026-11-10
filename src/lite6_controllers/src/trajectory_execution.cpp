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
      if (!active_trajectory_ || active_trajectory_->goal != goal)
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
  const auto & execution = *trajectory_buffer_.readFromRT();
  if (!execution || execution->result.load() != TrajectoryResult::RUNNING) return;
  const size_t n = joint_names_.size();
  if (execution->cancel.load()) {
    for (size_t i = 0; i < n; ++i) {
      reference_interfaces_[i] = state_interfaces_[i].get_value();
      reference_interfaces_[n + i] = 0.0;
      reference_interfaces_[2 * n + i] = 0.0;
    }
    execution->result.store(TrajectoryResult::CANCELED);
    return;
  }
  if (execution->endpoint_applied) {
    // A full CTC update has consumed the exact endpoint before completion is reported.
    execution->result.store(TrajectoryResult::SUCCEEDED);
    return;
  }
  if (execution->started) execution->elapsed += period.seconds();
  else {
    execution->started = true;
    execution->start_time_ns = time.nanoseconds();
  }

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
  if (execution->cancel.load()) {
    // The cancel callback returns before rclcpp_action enters CANCELING.
    if (!goal->is_canceling()) return;
    result->error_code = FollowTrajectory::Result::SUCCESSFUL;
    result->error_string = "Trajectory canceled";
    goal->canceled(result);
  } else if (outcome == TrajectoryResult::SUCCEEDED) {
    result->error_code = FollowTrajectory::Result::SUCCESSFUL;
    goal->succeed(result);
  } else {
    result->error_code = FollowTrajectory::Result::INVALID_GOAL;
    result->error_string = "CTC controller deactivated";
    goal->abort(result);
  }
  active_trajectory_.reset();
  trajectory_reserved_ = false;
}
}  // namespace lite6_controllers
