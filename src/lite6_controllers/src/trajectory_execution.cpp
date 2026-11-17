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
      using CS = lite6_interfaces::msg::ControllerStatus;
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      const uint8_t mode = current_mode_atomic_.load(std::memory_order_acquire);
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
      if (!active_trajectory_ || active_trajectory_->goal != goal ||
          active_trajectory_->result.load() != TrajectoryResult::RUNNING)
        return rclcpp_action::CancelResponse::REJECT;
      active_trajectory_->cancel.store(true);
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this](const std::shared_ptr<TrajectoryGoal> goal) {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      if (active_trajectory_) {
        retired_trajectories_.push_back(std::move(active_trajectory_));
      }
      active_trajectory_ = std::make_shared<TrajectoryExecution>(goal);
      if (!trajectory_enabled_.load()) {
        active_trajectory_->result.store(TrajectoryResult::DEACTIVATED);
      }
      trajectory_buffer_.writeFromNonRT(active_trajectory_);
    });
  trajectory_monitor_ = get_node()->create_wall_timer(
    std::chrono::milliseconds(20), [this] {
      monitor_trajectory();
      monitor_impedance_trajectory();
    });
}

void Lite6CTCController::update_trajectory_reference(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // The non-RT buffer retains ownership; no allocations, action calls or blocking locks here.
  auto * execution = current_execution_;
  if (!execution || execution->result.load() != TrajectoryResult::RUNNING) return;
  const size_t n = joint_names_.size();
  if (execution->cancel.load()) {
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
  if (execution->started) execution->elapsed += period.seconds();
  else {
    if (system_state_ != SystemState::NORMAL && system_state_ != SystemState::SYS_ID) {
      execution->result.store(TrajectoryResult::MODE_CHANGED);
      return;
    }
    execution->started = true;
    execution->start_time_ns = time.nanoseconds();
    execution->start_mode = static_cast<int>(system_state_);
  }
  // Hold the exact endpoint while the physical robot settles; keep timing it.
  if (execution->endpoint_applied) return;

  const auto & points = execution->goal->get_goal()->trajectory.points;
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
    if (execution->cancel_applied) {
      const bool settled =
        !execution->started ||
        (estop_dq_.cwiseAbs().maxCoeff() < 1e-4 &&
         dq_.cwiseAbs().maxCoeff() <= 0.01);
      const bool mode_left =
        execution->started && system_state_ != static_cast<SystemState>(execution->start_mode);
      if (settled || mode_left) {
        rigid_cancel_braking_ = false;
        rigid_cancel_elapsed_ = 0.0;
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
  std::shared_ptr<TrajectoryExecution> execution;
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
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
      feedback->desired = goal->get_goal()->trajectory.points[execution->sample.load()];
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
}  // namespace lite6_controllers
