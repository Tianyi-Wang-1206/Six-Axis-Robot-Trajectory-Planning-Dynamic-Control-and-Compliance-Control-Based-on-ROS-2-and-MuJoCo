#include "lite6_controllers/ctc_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace lite6_controllers
{
namespace
{
double seconds(const builtin_interfaces::msg::Duration & duration)
{
  return duration.sec + 1e-9 * duration.nanosec;
}

template<typename PointT>
size_t advance_sample_index(
  const std::vector<PointT> & points,
  size_t current_index,
  double elapsed,
  bool uniform_1khz,
  double t0_sec) noexcept
{
  const size_t n = points.size();
  if (n <= 1) {
    return 0;
  }
  size_t idx = (current_index < n) ? current_index : 0;
  if (idx + 1 >= n) {
    return n - 1;
  }
  // O(1) nominal 1 kHz step check (stay at idx or advance to idx + 1).
  if (elapsed < seconds(points[idx + 1].time_from_start)) {
    return idx;
  }
  if (idx + 2 >= n || elapsed < seconds(points[idx + 2].time_from_start)) {
    return idx + 1;
  }
  // Scheduling delay skipped > 1 sample: O(1) direct indexing for validated uniform 1 kHz
  // trajectories (bounded +/-1 refinement), or O(log N) binary search fallback.
  if (uniform_1khz) {
    const double est = (elapsed - t0_sec) * lite6_interfaces::contract::kControlRateHz;
    if (est >= static_cast<double>(n - 1)) {
      idx = n - 1;
    } else if (est > static_cast<double>(idx)) {
      idx = static_cast<size_t>(est);
    }
    while (idx > 0 && seconds(points[idx].time_from_start) > elapsed) {
      --idx;
    }
    while (idx + 1 < n && seconds(points[idx + 1].time_from_start) <= elapsed) {
      ++idx;
    }
    return idx;
  }
  const auto it = std::upper_bound(
    points.begin() + static_cast<std::ptrdiff_t>(idx + 1), points.end(), elapsed,
    [](double t, const PointT & p) { return t < seconds(p.time_from_start); });
  return static_cast<size_t>(std::distance(points.begin(), it) - 1);
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

// Playback uses 1 kHz sampled PVA with zero-order hold (ZOH). Regular FollowJointTrajectory
// goals must have a zero header.stamp, start at time_from_start == 0, maintain uniform 1 ms
// sample spacing within kTrajectorySampleSpacingTolSec, and end at rest.
bool Lite6CTCController::valid_trajectory_goal(
  const FollowTrajectory::Goal & goal, bool * out_uniform_1khz) const
{
  const auto & trajectory = goal.trajectory;
  const bool zero_stamp =
    trajectory.header.stamp.sec == 0 && trajectory.header.stamp.nanosec == 0;
  if (trajectory.joint_names != joint_names_ || trajectory.points.empty() ||
      !zero_stamp ||
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

  const double first_time = seconds(trajectory.points.front().time_from_start);
  if (first_time != 0.0) return false;

  double previous = -1.0;
  double max_grid_err = 0.0;
  for (size_t idx = 0; idx < trajectory.points.size(); ++idx) {
    const auto & point = trajectory.points[idx];
    const double t = seconds(point.time_from_start);
    if (point.time_from_start.sec < 0 || point.time_from_start.nanosec >= 1000000000u ||
        t <= previous || point.positions.size() != joint_names_.size() ||
        point.velocities.size() != joint_names_.size() ||
        point.accelerations.size() != joint_names_.size() ||
        (!point.effort.empty() && point.effort.size() != joint_names_.size()))
    {
      return false;
    }
    if (idx > 0) {
      const double dt = t - previous;
      if (std::abs(dt - lite6_interfaces::contract::kControlDtSec) >
          lite6_interfaces::contract::kTrajectorySampleSpacingTolSec)
      {
        return false;
      }
      const double expected_t = first_time +
        static_cast<double>(idx) * lite6_interfaces::contract::kControlDtSec;
      max_grid_err = std::max(max_grid_err, std::abs(t - expected_t));
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
  // Holding the final reference must not leave a nonzero feed-forward command.
  const auto & last = trajectory.points.back();
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    if (std::abs(last.velocities[i]) > 1e-8 || std::abs(last.accelerations[i]) > 1e-8)
      return false;
    if (!last.effort.empty() && std::abs(last.effort[i]) > 1e-8)
      return false;
  }
  if (out_uniform_1khz != nullptr) {
    *out_uniform_1khz = (max_grid_err <= 2e-4);
  }
  return true;
}

void Lite6CTCController::publish_measured_state_atomic(bool valid)
{
  if (!valid) {
    measured_state_valid_.store(false, std::memory_order_release);
    return;
  }
  const uint64_t seq = measured_state_seq_.load(std::memory_order_relaxed);
  measured_state_seq_.store(seq + 1ULL, std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_release);
  for (size_t i = 0; i < 6; ++i) {
    measured_q_atomic_[i].store(q_[static_cast<Eigen::Index>(i)], std::memory_order_relaxed);
    measured_dq_atomic_[i].store(dq_[static_cast<Eigen::Index>(i)], std::memory_order_relaxed);
  }
  measured_state_seq_.store(seq + 2ULL, std::memory_order_release);
  measured_state_valid_.store(true, std::memory_order_release);
}

bool Lite6CTCController::read_measured_state_atomic(
  std::array<double, 6> & q_out, std::array<double, 6> & dq_out) const
{
  if (!measured_state_valid_.load(std::memory_order_acquire)) {
    return false;
  }
  for (int attempt = 0; attempt < 8; ++attempt) {
    const uint64_t s1 = measured_state_seq_.load(std::memory_order_acquire);
    if ((s1 & 1ULL) != 0ULL) {
      continue;
    }
    for (size_t i = 0; i < 6; ++i) {
      q_out[i] = measured_q_atomic_[i].load(std::memory_order_relaxed);
      dq_out[i] = measured_dq_atomic_[i].load(std::memory_order_relaxed);
    }
    std::atomic_thread_fence(std::memory_order_acquire);
    const uint64_t s2 = measured_state_seq_.load(std::memory_order_relaxed);
    if (s1 == s2 && measured_state_valid_.load(std::memory_order_acquire)) {
      for (size_t i = 0; i < 6; ++i) {
        if (!std::isfinite(q_out[i]) || !std::isfinite(dq_out[i])) {
          return false;
        }
      }
      return true;
    }
  }
  return false;
}

bool Lite6CTCController::check_trajectory_entry_continuity(
  const FollowTrajectory::Goal & goal,
  const std::array<double, 6> & q_meas,
  const std::array<double, 6> & dq_meas,
  int * out_joint,
  double * out_value,
  double * out_limit) const
{
  if (goal.trajectory.points.empty()) {
    return false;
  }
  std::array<double, 6> eff_pos_tol{};
  std::array<double, 6> eff_vel_tol{};
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    eff_pos_tol[i] = path_position_tolerance_[i];
    eff_vel_tol[i] = path_velocity_tolerance_[i];
  }
  for (const auto & tol : goal.path_tolerance) {
    const auto it = std::find(joint_names_.begin(), joint_names_.end(), tol.name);
    if (it != joint_names_.end()) {
      const size_t idx = static_cast<size_t>(std::distance(joint_names_.begin(), it));
      if (tol.position > 0.0) {
        eff_pos_tol[idx] = tol.position;
      }
      if (tol.velocity > 0.0) {
        eff_vel_tol[idx] = tol.velocity;
      }
    }
  }
  const auto & first = goal.trajectory.points.front();
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const double pos_lim = std::min(entry_position_tolerance_, eff_pos_tol[i]);
    const double vel_lim = std::min(entry_velocity_tolerance_, eff_vel_tol[i]);
    const double pos_err = std::abs(first.positions[i] - q_meas[i]);
    if (pos_err > pos_lim) {
      if (out_joint) *out_joint = static_cast<int>(i);
      if (out_value) *out_value = pos_err;
      if (out_limit) *out_limit = pos_lim;
      return false;
    }
    const double vel_err = std::abs(first.velocities[i] - dq_meas[i]);
    if (vel_err > vel_lim) {
      if (out_joint) *out_joint = static_cast<int>(i);
      if (out_value) *out_value = vel_err;
      if (out_limit) *out_limit = vel_lim;
      return false;
    }
  }
  return true;
}

uint8_t Lite6CTCController::valid_splice_request(
  const SpliceTrajectory::Request & req,
  bool * out_uniform_1khz,
  std::string * out_message) const
{
  using Resp = SpliceTrajectory::Response;
  using CS = lite6_interfaces::msg::ControllerStatus;
  // Caller holds trajectory_mutex_.
  if (!trajectory_enabled_.load(std::memory_order_acquire) ||
      current_mode_atomic_.load(std::memory_order_acquire) != CS::MODE_NORMAL)
  {
    if (out_message) *out_message = "Splice is permitted only while active in MODE_NORMAL";
    return Resp::RESULT_REJECTED_STATE;
  }
  const uint64_t active_epoch = controller_epoch_atomic_.load(std::memory_order_acquire);
  if (req.expected_controller_epoch > 0 && req.expected_controller_epoch != active_epoch) {
    if (out_message) *out_message = "Controller epoch mismatch";
    return Resp::RESULT_REJECTED_EPOCH;
  }
  const auto & parent = active_trajectory_;
  if (!parent || parent->result.load(std::memory_order_acquire) != TrajectoryResult::RUNNING ||
      parent->cancel.load(std::memory_order_acquire) || parent->splice_reserved ||
      parent->goal->get_goal_id() != req.parent_goal_id ||
      parent->start_time_pub_ns.load(std::memory_order_acquire) <= 0)
  {
    if (out_message) *out_message = "No matching started unspliced rigid parent goal";
    return Resp::RESULT_REJECTED_PARENT;
  }
  if (req.switch_time.sec < 0 || req.switch_time.nanosec >= 1000000000u) {
    if (out_message) *out_message = "Invalid switch_time duration";
    return Resp::RESULT_REJECTED_INVALID_ARG;
  }
  const double t_switch = seconds(req.switch_time);
  const auto & trajectory = req.trajectory;
  if (!(t_switch > 0.0) || trajectory.joint_names != joint_names_ || trajectory.points.empty()) {
    if (out_message) *out_message = "Invalid splice trajectory header, joint names, or empty points";
    return Resp::RESULT_REJECTED_INVALID_ARG;
  }
  const double first_time = seconds(trajectory.points.front().time_from_start);
  if (std::abs(first_time - t_switch) > 1e-9) {
    if (out_message) *out_message = "Splice trajectory must begin at switch_time";
    return Resp::RESULT_REJECTED_INVALID_ARG;
  }

  double previous = -1.0;
  double max_grid_err = 0.0;
  for (size_t idx = 0; idx < trajectory.points.size(); ++idx) {
    const auto & point = trajectory.points[idx];
    const double t = seconds(point.time_from_start);
    if (point.time_from_start.sec < 0 || point.time_from_start.nanosec >= 1000000000u ||
        t <= previous || point.positions.size() != joint_names_.size() ||
        point.velocities.size() != joint_names_.size() ||
        point.accelerations.size() != joint_names_.size() ||
        (!point.effort.empty() && point.effort.size() != joint_names_.size()))
    {
      if (out_message) *out_message = "Malformed splice trajectory point";
      return Resp::RESULT_REJECTED_INVALID_ARG;
    }
    if (idx > 0) {
      const double dt = t - previous;
      if (std::abs(dt - lite6_interfaces::contract::kControlDtSec) >
          lite6_interfaces::contract::kTrajectorySampleSpacingTolSec)
      {
        if (out_message) *out_message = "Splice trajectory requires uniform 1 kHz (1 ms) sample spacing";
        return Resp::RESULT_REJECTED_INVALID_ARG;
      }
      const double expected_t = first_time +
        static_cast<double>(idx) * lite6_interfaces::contract::kControlDtSec;
      max_grid_err = std::max(max_grid_err, std::abs(t - expected_t));
    }
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (!std::isfinite(point.positions[i]) || !std::isfinite(point.velocities[i]) ||
          !std::isfinite(point.accelerations[i]))
      {
        if (out_message) *out_message = "Non-finite PVA in splice trajectory";
        return Resp::RESULT_REJECTED_INVALID_ARG;
      }
      if (!point.effort.empty() &&
          (!std::isfinite(point.effort[i]) ||
           std::abs(point.effort[i]) > max_effort_[static_cast<Eigen::Index>(i)]))
      {
        if (out_message) *out_message = "Invalid feedforward effort in splice trajectory";
        return Resp::RESULT_REJECTED_INVALID_ARG;
      }
    }
    previous = t;
  }
  const auto & last = trajectory.points.back();
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    if (std::abs(last.velocities[i]) > 1e-8 || std::abs(last.accelerations[i]) > 1e-8 ||
        (!last.effort.empty() && std::abs(last.effort[i]) > 1e-8))
    {
      if (out_message) *out_message = "Splice trajectory must end at rest";
      return Resp::RESULT_REJECTED_INVALID_ARG;
    }
  }

  // Seamless switch: the first splice sample must equal the parent sample at switch_time,
  // strictly before the parent's final sample.
  const auto & parent_points = parent->goal->get_goal()->trajectory.points;
  const auto & first = trajectory.points.front();
  const auto it = std::lower_bound(
    parent_points.begin(), parent_points.end(), t_switch - 1e-7,
    [](const trajectory_msgs::msg::JointTrajectoryPoint & p, double t) {
      return seconds(p.time_from_start) < t;
    });
  if (it == parent_points.end() || std::next(it) == parent_points.end() ||
      std::abs(seconds(it->time_from_start) - t_switch) > 1e-7)
  {
    if (out_message) *out_message = "switch_time does not match an interior parent sample";
    return Resp::RESULT_REJECTED_INVALID_ARG;
  }
  if (it->effort.empty() != first.effort.empty()) {
    if (out_message) *out_message = "Splice effort presence mismatches parent";
    return Resp::RESULT_REJECTED_INVALID_ARG;
  }
  constexpr double kContinuityTol = 1e-9;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    if (std::abs(it->positions[i] - first.positions[i]) > kContinuityTol ||
        std::abs(it->velocities[i] - first.velocities[i]) > kContinuityTol ||
        std::abs(it->accelerations[i] - first.accelerations[i]) > kContinuityTol ||
        (!first.effort.empty() && std::abs(it->effort[i] - first.effort[i]) > kContinuityTol))
    {
      if (out_message) *out_message = "Splice initial PVA does not continuously match parent at switch_time";
      return Resp::RESULT_REJECTED_INVALID_ARG;
    }
  }
  const double est_elapsed = parent->elapsed_pub.load(std::memory_order_acquire);
  if (t_switch <= est_elapsed) {
    if (out_message) *out_message = "Splice switch_time already passed";
    return Resp::RESULT_REJECTED_TIMING;
  }
  if (out_uniform_1khz != nullptr) {
    *out_uniform_1khz = (max_grid_err <= 2e-4);
  }
  return Resp::RESULT_ADOPTED;
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
      std::array<double, 6> q_meas{}, dq_meas{};
      if (!trajectory_enabled_.load(std::memory_order_acquire) ||
          compliance_blocks_trajectory_.load() ||
          (mode != CS::MODE_NORMAL && mode != CS::MODE_SYSID) ||
          trajectory_reserved_ || impedance_trajectory_reserved_ ||
          !valid_trajectory_goal(*goal) ||
          !read_measured_state_atomic(q_meas, dq_meas) ||
          !check_trajectory_entry_continuity(*goal, q_meas, dq_meas))
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
      bool uniform_1khz = true;
      (void)valid_trajectory_goal(*goal->get_goal(), &uniform_1khz);
      active_trajectory_ = std::make_shared<TrajectoryExecution>(goal);
      active_trajectory_->uniform_1khz = uniform_1khz;
      active_trajectory_->t0_sec =
        seconds(goal->get_goal()->trajectory.points.front().time_from_start);
      populate_goal_tolerances(*goal->get_goal(), *active_trajectory_);
      if (!trajectory_enabled_.load()) {
        active_trajectory_->result.store(TrajectoryResult::DEACTIVATED);
      }
      trajectory_buffer_.writeFromNonRT(active_trajectory_);
    });

  splice_service_ = get_node()->create_service<SpliceTrajectory>(
    "~/splice_joint_trajectory",
    [this](
      const std::shared_ptr<SpliceTrajectory::Request> req,
      std::shared_ptr<SpliceTrajectory::Response> resp)
    {
      using Resp = SpliceTrajectory::Response;
      std::shared_ptr<TrajectoryExecution> parent;
      std::shared_ptr<SpliceExecution> splice;
      uint64_t active_epoch = 0;
      double wait_sec = 0.25;
      {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        bool uniform_1khz = true;
        std::string msg;
        const uint8_t check = valid_splice_request(*req, &uniform_1khz, &msg);
        active_epoch = controller_epoch_atomic_.load(std::memory_order_acquire);
        if (check != Resp::RESULT_ADOPTED) {
          resp->adopted = false;
          resp->result_code = check;
          resp->controller_epoch = active_epoch;
          resp->message = msg;
          return;
        }
        parent = active_trajectory_;
        splice = std::make_shared<SpliceExecution>();
        splice->trajectory = req->trajectory;
        splice->switch_time = seconds(req->switch_time);
        splice->t0_sec = splice->switch_time;
        splice->uniform_1khz = uniform_1khz;
        splice->controller_epoch = active_epoch;

        const double est_elapsed = parent->elapsed_pub.load(std::memory_order_acquire);
        wait_sec = std::clamp(
          std::max(0.0, splice->switch_time - est_elapsed) + 0.25, 0.20, 2.0);

        parent->splice_reserved = true;
        parent->splice_owner = splice;
        parent->splice.store(splice.get(), std::memory_order_release);
      }

      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(wait_sec));
      auto finalize = [&](SpliceState st) {
        if (st != SpliceState::ADOPTED) {
          std::lock_guard<std::mutex> lock(trajectory_mutex_);
          if (parent->splice_owner == splice) {
            parent->splice.store(nullptr, std::memory_order_release);
            parent->retired_splices.push_back(std::move(parent->splice_owner));
            parent->splice_reserved = false;
          }
        }
        resp->controller_epoch = controller_epoch_atomic_.load(std::memory_order_acquire);
        switch (st) {
          case SpliceState::ADOPTED:
            resp->adopted = true;
            resp->result_code = Resp::RESULT_ADOPTED;
            resp->message = "Splice adopted by 1 kHz RT loop";
            return;
          case SpliceState::REJECTED_TIMING:
            resp->adopted = false;
            resp->result_code = Resp::RESULT_REJECTED_TIMING;
            resp->message = "Splice switch_time already passed or endpoint reached";
            return;
          case SpliceState::REJECTED_STATE:
            resp->adopted = false;
            resp->result_code = Resp::RESULT_REJECTED_STATE;
            resp->message = "Controller left MODE_NORMAL or changed activation epoch";
            return;
          case SpliceState::REJECTED_PARENT:
            resp->adopted = false;
            resp->result_code = Resp::RESULT_REJECTED_PARENT;
            resp->message = "Parent goal ended or was canceled before switch_time";
            return;
          case SpliceState::WITHDRAWN:
          default:
            resp->adopted = false;
            resp->result_code = Resp::RESULT_RT_TIMEOUT;
            resp->message = "Splice withdrawn before RT adoption (definitively not adopted)";
            return;
        }
      };

      while (std::chrono::steady_clock::now() < deadline) {
        const auto st = static_cast<SpliceState>(splice->state.load(std::memory_order_acquire));
        if (st != SpliceState::PENDING) {
          finalize(st);
          return;
        }
        if (!trajectory_enabled_.load(std::memory_order_acquire) ||
            controller_epoch_atomic_.load(std::memory_order_acquire) != active_epoch)
        {
          uint8_t expected = static_cast<uint8_t>(SpliceState::PENDING);
          if (splice->state.compare_exchange_strong(
                expected, static_cast<uint8_t>(SpliceState::REJECTED_STATE),
                std::memory_order_acq_rel, std::memory_order_acquire))
          {
            finalize(SpliceState::REJECTED_STATE);
            return;
          }
          finalize(static_cast<SpliceState>(expected));
          return;
        }
        if (parent->result.load(std::memory_order_acquire) != TrajectoryResult::RUNNING ||
            parent->cancel.load(std::memory_order_acquire))
        {
          uint8_t expected = static_cast<uint8_t>(SpliceState::PENDING);
          if (splice->state.compare_exchange_strong(
                expected, static_cast<uint8_t>(SpliceState::REJECTED_PARENT),
                std::memory_order_acq_rel, std::memory_order_acquire))
          {
            finalize(SpliceState::REJECTED_PARENT);
            return;
          }
          finalize(static_cast<SpliceState>(expected));
          return;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(250));
      }

      uint8_t expected = static_cast<uint8_t>(SpliceState::PENDING);
      if (splice->state.compare_exchange_strong(
            expected, static_cast<uint8_t>(SpliceState::WITHDRAWN),
            std::memory_order_acq_rel, std::memory_order_acquire))
      {
        finalize(SpliceState::WITHDRAWN);
        return;
      }
      finalize(static_cast<SpliceState>(expected));
    },
    rclcpp::ServicesQoS(), command_callback_group_);

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
  if (!execution || execution->result.load() != TrajectoryResult::RUNNING) {
    tau_point_ff_.setZero();
    return;
  }
  auto reject_pending_splice = [&](SpliceState reason) {
    if (execution->tail == nullptr) {
      if (auto * splice = execution->splice.load(std::memory_order_acquire)) {
        if (splice != execution->last_splice_resolved) {
          uint8_t expected = static_cast<uint8_t>(SpliceState::PENDING);
          splice->state.compare_exchange_strong(
            expected, static_cast<uint8_t>(reason),
            std::memory_order_acq_rel, std::memory_order_acquire);
          execution->last_splice_resolved = splice;
        }
      }
    }
  };

  const size_t n = joint_names_.size();
  if (execution->cancel.load()) {
    reject_pending_splice(SpliceState::REJECTED_PARENT);
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
    execution->elapsed_pub.store(execution->elapsed, std::memory_order_release);
  } else {
    if (system_state_ != SystemState::NORMAL && system_state_ != SystemState::SYS_ID) {
      reject_pending_splice(SpliceState::REJECTED_STATE);
      execution->result.store(TrajectoryResult::MODE_CHANGED);
      tau_point_ff_.setZero();
      return;
    }
    // Start the trajectory clock only after the configured payload model has settled.
    if (payload_smoothing_active_) return;
    std::array<double, 6> q_rt{}, dq_rt{};
    for (size_t i = 0; i < n; ++i) {
      q_rt[i] = q_[static_cast<Eigen::Index>(i)];
      dq_rt[i] = dq_[static_cast<Eigen::Index>(i)];
    }
    int entry_joint = -1;
    double entry_val = 0.0;
    double entry_lim = 0.0;
    if (!check_trajectory_entry_continuity(
          *execution->goal->get_goal(), q_rt, dq_rt, &entry_joint, &entry_val, &entry_lim))
    {
      // Abort pre-start cleanly without latching E-STOP if physical state drifted
      // beyond entry continuity tolerance between admission and first RT tick.
      reject_pending_splice(SpliceState::REJECTED_PARENT);
      execution->failure_joint = entry_joint;
      execution->failure_value = entry_val;
      execution->failure_limit = entry_lim;
      execution->result.store(TrajectoryResult::ENTRY_DEVIATION);
      tau_point_ff_.setZero();
      return;
    }
    execution->started = true;
    execution->start_time_ns = time.nanoseconds();
    execution->start_time_pub_ns.store(execution->start_time_ns, std::memory_order_release);
    execution->elapsed_pub.store(0.0, std::memory_order_release);
    execution->start_mode = static_cast<int>(system_state_);
  }

  // Path-preserving splice: each candidate is validated once on first sight (rejected if
  // epoch/mode changed, switch_time already passed, or endpoint already held), then adopted
  // atomically at switch_time via CAS on splice->state (racing cleanly against withdrawal).
  // Rejected/withdrawn candidates do not prevent a subsequent valid retry on this parent.
  if (execution->tail == nullptr) {
    auto * splice = execution->splice.load(std::memory_order_acquire);
    if (splice != nullptr && splice != execution->last_splice_resolved) {
      uint8_t expected = static_cast<uint8_t>(SpliceState::PENDING);
      if (splice != execution->last_splice_seen) {
        execution->last_splice_seen = splice;
        if (system_state_ != SystemState::NORMAL || splice->controller_epoch != controller_epoch_) {
          execution->last_splice_resolved = splice;
          splice->state.compare_exchange_strong(
            expected, static_cast<uint8_t>(SpliceState::REJECTED_STATE),
            std::memory_order_acq_rel, std::memory_order_acquire);
        } else if (execution->endpoint_applied || execution->elapsed > splice->switch_time) {
          execution->last_splice_resolved = splice;
          splice->state.compare_exchange_strong(
            expected, static_cast<uint8_t>(SpliceState::REJECTED_TIMING),
            std::memory_order_acq_rel, std::memory_order_acquire);
        }
      }
      if (splice != execution->last_splice_resolved && execution->elapsed >= splice->switch_time) {
        if (system_state_ != SystemState::NORMAL || splice->controller_epoch != controller_epoch_) {
          execution->last_splice_resolved = splice;
          splice->state.compare_exchange_strong(
            expected, static_cast<uint8_t>(SpliceState::REJECTED_STATE),
            std::memory_order_acq_rel, std::memory_order_acquire);
        } else if (splice->state.compare_exchange_strong(
                     expected, static_cast<uint8_t>(SpliceState::ADOPTED),
                     std::memory_order_acq_rel, std::memory_order_acquire))
        {
          execution->last_splice_resolved = splice;
          execution->tail = splice;
          execution->last_sample_index = 0;
          execution->endpoint_applied = false;
          execution->on_tail.store(true, std::memory_order_release);
        } else {
          // Service already withdrew or rejected the splice before adoption.
          execution->last_splice_resolved = splice;
        }
      }
    }
  }

  // Hold the exact endpoint while the physical robot settles; keep timing it.
  if (execution->endpoint_applied) {
    tau_point_ff_.setZero();
    return;
  }

  const bool on_tail = (execution->tail != nullptr);
  const auto & points = on_tail ?
    execution->tail->trajectory.points :
    execution->goal->get_goal()->trajectory.points;
  const bool uniform_1khz = on_tail ? execution->tail->uniform_1khz : execution->uniform_1khz;
  const double t0_sec = on_tail ? execution->tail->t0_sec : execution->t0_sec;
  const size_t index = advance_sample_index(
    points, execution->last_sample_index, execution->elapsed, uniform_1khz, t0_sec);
  execution->last_sample_index = index;
  const auto & point = points[index];
  // Zero-order hold of the latest due 1 kHz PVA sample.
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
  if (execution->tail == nullptr) {
    if (auto * splice = execution->splice.load(std::memory_order_acquire)) {
      if (splice != execution->last_splice_resolved) {
        uint8_t expected = static_cast<uint8_t>(SpliceState::PENDING);
        splice->state.compare_exchange_strong(
          expected, static_cast<uint8_t>(SpliceState::REJECTED_PARENT),
          std::memory_order_acq_rel, std::memory_order_acquire);
        execution->last_splice_resolved = splice;
      }
    }
  }
  execution->failure_joint = joint;
  execution->failure_value = value;
  execution->failure_limit = limit;
  execution->result.store(reason);
}

controller_interface::return_type Lite6CTCController::fail_controller(
  TrajectoryResult reason, int joint)
{
  trajectory_enabled_.store(false);
  publish_measured_state_atomic(false);
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
  tau_applied_prev_.setZero();
  has_applied_tau_ = false;
  recovery_bias_active_ = false;
  recovery_bias_elapsed_ = 0.0;
  recovery_bias_init_.setZero();
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
        if (system_state_ == SystemState::NORMAL) {
          bumpless_handover_to_normal(0.20);
        } else if (system_state_ == SystemState::SYS_ID) {
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
  const auto & end_points = (execution->tail != nullptr) ?
    execution->tail->trajectory.points :
    execution->goal->get_goal()->trajectory.points;
  const double end_time = seconds(end_points.back().time_from_start);
  const double time_tol = execution->goal_time_tol > 0.0 ?
    execution->goal_time_tol : goal_time_tolerance_;
  // The deadline includes the continuous settling window, not just first entry.
  if (execution->elapsed > end_time + time_tol) {
    fail_trajectory(TrajectoryResult::GOAL_TIMEOUT, worst_joint, worst_ratio, 1.0);
  } else if (execution->settled_duration >= goal_settle_duration_) {
    if (execution->tail == nullptr) {
      if (auto * splice = execution->splice.load(std::memory_order_acquire)) {
        if (splice != execution->last_splice_resolved) {
          uint8_t expected = static_cast<uint8_t>(SpliceState::PENDING);
          splice->state.compare_exchange_strong(
            expected, static_cast<uint8_t>(SpliceState::REJECTED_PARENT),
            std::memory_order_acq_rel, std::memory_order_acquire);
          execution->last_splice_resolved = splice;
        }
      }
    }
    execution->result.store(TrajectoryResult::SUCCEEDED);
  }
}

void Lite6CTCController::monitor_trajectory()
{
  std::shared_ptr<TrajectoryExecution> execution;
  std::shared_ptr<SpliceExecution> splice_child;
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
        splice_child->trajectory.points :
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
    result->error_code =
      outcome == TrajectoryResult::GOAL_TIMEOUT ? FollowTrajectory::Result::GOAL_TOLERANCE_VIOLATED :
      outcome == TrajectoryResult::ENTRY_DEVIATION ? FollowTrajectory::Result::INVALID_GOAL :
      FollowTrajectory::Result::PATH_TOLERANCE_VIOLATED;
    switch (outcome) {
      case TrajectoryResult::ENTRY_DEVIATION:
        result->error_string =
          "Initial trajectory state deviates from measured joint state [rad or rad/s]"; break;
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
