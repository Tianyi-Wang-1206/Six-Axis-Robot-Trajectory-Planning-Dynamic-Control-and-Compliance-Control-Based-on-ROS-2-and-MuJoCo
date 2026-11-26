#include "lite6_planner/motion_planner.hpp"
#include <lite6_interfaces/msg/compile_diagnostic.hpp>
#include <rclcpp/logging.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <vector>

namespace lite6_planner {
namespace {
using Action = lite6_interfaces::action::IndustrialMotion;
using Diagnostic = lite6_interfaces::msg::CompileDiagnostic;
using JointLimit = lite6_kinematics::JointLimit;
namespace tol = lite6_kinematics::tolerance;

[[noreturn]] void fail(int32_t code, const std::string& msg) {
  throw MotionError(code, msg);
}

std::string range_error(const std::string& context, size_t joint, double value,
    double lower, double upper, double allowance) {
  std::ostringstream out;
  out << std::setprecision(12) << context << ": joint" << joint + 1 << " value=" << value
      << ", range=[" << lower << ", " << upper << "], tolerance=" << allowance
      << ", excess=" << std::max({lower - value, value - upper, 0.0});
  return out.str();
}

std::array<double, 6> bounded_positions(const std::array<double, 6>& q,
    const std::array<JointLimit, 6>& limits, double allowance, int32_t code,
    const std::string& context) {
  std::array<double, 6> result{};
  for (size_t i = 0; i < 6; ++i) {
    if (!std::isfinite(q[i])) {
      fail(code, context + ": joint" + std::to_string(i + 1) + " position is not finite");
    }
    const auto& lim = limits[i];
    if (q[i] < lim.min_pos - allowance || q[i] > lim.max_pos + allowance) {
      fail(code, range_error(context, i, q[i], lim.min_pos, lim.max_pos, allowance));
    }
    result[i] = std::clamp(q[i], lim.min_pos, lim.max_pos);
  }
  return result;
}

double angle_difference(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
  return std::abs(Eigen::AngleAxisd(a.transpose() * b).angle());
}

int32_t map_diagnostic_code(const std::string& code) {
  if (code == "ERR_CANCELLED") {
    return Action::Result::ERR_CANCELLED;
  }
  if (code == "ERR_START_JOINT_LIMIT") {
    return Action::Result::ERR_NOT_READY;
  }
  if (code == "ERR_START_COLLISION" ||
      code == "ERR_JOINT_POS_LIMIT" ||
      code == "ERR_JOINT_VEL_LIMIT" ||
      code == "ERR_JOINT_ACC_LIMIT" ||
      code == "ERR_JOINT_JERK_LIMIT" ||
      code == "ERR_COLLISION" ||
      code == "ERR_IK_LIMIT_OR_COLLISION")
  {
    return Action::Result::ERR_LIMIT_VIOLATION;
  }
  if (code == "ERR_SINGULARITY") {
    return Action::Result::ERR_SINGULARITY;
  }
  if (code == "ERR_QUADRANT_JUMP") {
    return Action::Result::ERR_QUADRANT_JUMP;
  }
  if (code == "ERR_NO_IK_SOLUTION" || code == "ERR_SPLINE_RESIDUAL") {
    return Action::Result::ERR_NO_IK_SOLUTION;
  }
  if (code == "ERR_DEGENERATE_CIRCLE" ||
      code == "ERR_COLLINEAR_CIRCLE" ||
      code == "ERR_CIRCLE_RADIUS" ||
      code == "ERR_CIRCLE_ARC_ORDER" ||
      code == "ERR_CIRCLE_CENTER_COINCIDENT" ||
      code == "ERR_EMPTY_PATH" ||
      code == "ERR_UNSUPPORTED_CART_PRIMITIVE" ||
      code == "ERR_PATH_TOO_LONG" ||
      code == "ERR_INVALID_SCALE" ||
      code == "ERR_INVALID_TRAJECTORY" ||
      code == "ERR_NONFINITE_SAMPLE" ||
      code == "ERR_NONMONOTONE_TIME")
  {
    return Action::Result::ERR_INVALID_INPUT;
  }
  if (code == "ERR_TOPP_SOLVER" ||
      code == "ERR_TOO_FEW_SAMPLES" ||
      code == "ERR_JOINT_PATH" ||
      code == "ERR_DURATION_LIMIT")
  {
    return Action::Result::ERR_RUCKIG_FAILED;
  }
  return Action::Result::ERR_INTERNAL;
}

void log_warnings(const std::vector<Diagnostic>& diagnostics) {
  const auto logger = rclcpp::get_logger("lite6_planner");
  for (const auto& d : diagnostics) {
    if (d.severity == Diagnostic::SEVERITY_WARNING) {
      RCLCPP_WARN(logger, "[%s] %s", d.code.c_str(), d.message.c_str());
    }
  }
}

[[noreturn]] void throw_first_error(
    const std::vector<Diagnostic>& diagnostics,
    const std::string& prefix,
    const std::string& fallback_msg) {
  const Diagnostic* err = nullptr;
  for (const auto& d : diagnostics) {
    if (d.severity == Diagnostic::SEVERITY_ERROR) {
      err = &d;
      break;
    }
  }
  if (!err && !diagnostics.empty()) {
    err = &diagnostics.front();
  }
  if (!err) {
    fail(Action::Result::ERR_INTERNAL, prefix + fallback_msg);
  }
  const int32_t mapped = map_diagnostic_code(err->code);
  const std::string stem = (!prefix.empty() && prefix.back() == ' ')
      ? prefix.substr(0, prefix.find(':'))
      : prefix;
  if (mapped == Action::Result::ERR_CANCELLED || prefix.empty() ||
      (!stem.empty() && err->message.rfind(stem, 0) == 0))
  {
    fail(mapped, err->message);
  }
  fail(mapped, prefix + err->message);
}

} // namespace

MotionPlanner::MotionPlanner(const lite6_kinematics::KinematicsEngine& engine)
  : engine_(engine), compiler_(engine) {}

void MotionPlanner::check_cancelled(const Cancellation& c) {
  if (c && c()) {
    fail(Action::Result::ERR_CANCELLED, "Motion cancelled by supervisor");
  }
}

Eigen::Matrix4d MotionPlanner::pose_matrix(const geometry_msgs::msg::Pose& pose) const {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  const auto& p = pose.position;
  const auto& o = pose.orientation;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(o.x) || !std::isfinite(o.y) || !std::isfinite(o.z) || !std::isfinite(o.w)) {
    fail(Action::Result::ERR_INVALID_INPUT, "Non-finite Cartesian target");
  }
  Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
  const double n = q.norm();
  if (n < 1e-9 || std::abs(n - 1.0) > 1e-3) {
    fail(Action::Result::ERR_INVALID_INPUT, "Pose quaternion must be normalized");
  }
  q.normalize();
  T.topLeftCorner<3, 3>() = q.toRotationMatrix();
  T.topRightCorner<3, 1>() = Eigen::Vector3d(p.x, p.y, p.z);
  return T;
}

trajectory_msgs::msg::JointTrajectory MotionPlanner::plan(const Goal& goal,
    const InitialState& initial, const Cancellation& cancel) const {
  if (!std::isfinite(goal.velocity_scale) || !std::isfinite(goal.acceleration_scale) ||
      goal.velocity_scale < 0.01 || goal.velocity_scale > 1.0 ||
      goal.acceleration_scale < 0.01 || goal.acceleration_scale > 1.0) {
    fail(Action::Result::ERR_INVALID_INPUT, "Scaling factors must lie in [0.01,1]");
  }

  const auto& limits = engine_.get_joint_limits();
  InitialState start = initial;
  start.q = bounded_positions(initial.q, limits, tol::kObservedPosition,
      Action::Result::ERR_NOT_READY, "Current joint position [rad]");

  // Allow small residual joint velocity within the stationary admission band
  // (kStationaryVelocity) so the debugging GUI never stalls on tiny encoder noise.
  for (size_t i = 0; i < 6; ++i) {
    const double v = initial.dq[i];
    if (!std::isfinite(v)) {
      fail(Action::Result::ERR_NOT_READY,
          "Current joint" + std::to_string(i + 1) + " velocity is not finite");
    }
    if (std::abs(v) > tol::kStationaryVelocity) {
      fail(Action::Result::ERR_NOT_READY, range_error("Robot must be stationary [rad/s]",
          i, v, -tol::kStationaryVelocity, tol::kStationaryVelocity, 0.0));
    }
    if (std::abs(v) <= tol::kZeroVelocity) {
      start.dq[i] = 0.0;
    }
    // Projection must not hide real outward motion at a joint boundary.
    if ((start.q[i] <= limits[i].min_pos && start.dq[i] < 0.0) ||
        (start.q[i] >= limits[i].max_pos && start.dq[i] > 0.0)) {
      fail(Action::Result::ERR_NOT_READY,
          "Joint" + std::to_string(i + 1) + " must settle: velocity points outside joint range");
    }
  }

  // Check the measured pose too, so clamping cannot conceal a collision.
  std::string start_collision;
  if (engine_.check_collision(initial.q, &start_collision) ||
      engine_.check_collision(start.q, &start_collision)) {
    fail(Action::Result::ERR_LIMIT_VIOLATION, "Start configuration: " + start_collision);
  }
  check_cancelled(cancel);

  lite6_studio_planner::MotionStatement stmt;
  stmt.options.vel_scale = goal.velocity_scale;
  stmt.options.acc_scale = goal.acceleration_scale;
  stmt.options.jerk_scale = 1.0;
  stmt.options.blend_radius = 0.0;

  std::string err_prefix;
  if (goal.command_type == Goal::MOVEJ) {
    stmt.loc = {1, 1, 0, "MoveJ"};
    stmt.type = lite6_studio_planner::MotionCommandType::MOVE_J;
    // SysID alignment may send a snapshot of measured joint positions back as the target.
    // Apply the observation allowance only to such local alignment requests.
    bool observation_target = (goal.context == Goal::CONTEXT_SYSID);
    for (size_t i = 0; i < 6; ++i) {
      observation_target = observation_target && std::isfinite(goal.joint_target[i]) &&
          std::abs(goal.joint_target[i] - initial.q[i]) <= tol::kObservedPosition;
    }
    const double allowance = observation_target ? tol::kObservedPosition : tol::kJointPosition;
    const std::string ctx = observation_target ? "Alignment joint target [rad]" : "PTP joint target [rad]";
    stmt.joint_target = bounded_positions(
        goal.joint_target, limits, allowance, Action::Result::ERR_LIMIT_VIOLATION, ctx);
  } else if (goal.command_type == Goal::MOVEP) {
    stmt.loc = {1, 1, 0, "MoveP"};
    const Eigen::Matrix4d T = pose_matrix(goal.pose_target);
    const Eigen::Matrix4d current = engine_.forward_pose(start.q);
    if ((current.topRightCorner<3, 1>() - T.topRightCorner<3, 1>()).norm() < 1e-5 &&
        angle_difference(current.topLeftCorner<3, 3>(), T.topLeftCorner<3, 3>()) < 1e-4) {
      // Zero-distance MoveP: hold current joints directly to avoid IK flips at singularities.
      stmt.type = lite6_studio_planner::MotionCommandType::MOVE_J;
      stmt.joint_target = start.q;
    } else {
      stmt.type = lite6_studio_planner::MotionCommandType::MOVE_P;
      stmt.pose_target = T;
    }
  } else if (goal.command_type == Goal::MOVEL) {
    stmt.loc = {1, 1, 0, "MoveL"};
    err_prefix = "MoveL: ";
    const Eigen::Matrix4d T = pose_matrix(goal.pose_target);
    const Eigen::Matrix4d current = engine_.forward_pose(start.q);
    if ((current.topRightCorner<3, 1>() - T.topRightCorner<3, 1>()).norm() < 1e-5 &&
        angle_difference(current.topLeftCorner<3, 3>(), T.topLeftCorner<3, 3>()) < 1e-4) {
      // Zero-distance MoveL: hold current joints directly (safe even at singular HOME
      // and robust to sub-0.01 mm joint-limit observation clamping at joint3 = 0).
      stmt.type = lite6_studio_planner::MotionCommandType::MOVE_J;
      stmt.joint_target = start.q;
    } else {
      stmt.type = lite6_studio_planner::MotionCommandType::MOVE_L;
      stmt.pose_target = T;
    }
  } else if (goal.command_type == Goal::MOVEC) {
    stmt.loc = {1, 1, 0, "MoveC"};
    err_prefix = "MoveC: ";
    const Eigen::Matrix4d T_target = pose_matrix(goal.pose_target);
    const auto & p_aux = goal.pose_aux.position;
    if (!std::isfinite(p_aux.x) || !std::isfinite(p_aux.y) || !std::isfinite(p_aux.z)) {
      fail(Action::Result::ERR_INVALID_INPUT, "Non-finite Cartesian via position");
    }
    Eigen::Matrix4d T_aux = Eigen::Matrix4d::Identity();
    T_aux.topRightCorner<3, 1>() = Eigen::Vector3d(p_aux.x, p_aux.y, p_aux.z);
    stmt.type = lite6_studio_planner::MotionCommandType::MOVE_C_THREE_POINT;
    stmt.pose_target = T_target;
    stmt.pose_via = T_aux;
    stmt.via_has_orientation = false;
  } else {
    fail(Action::Result::ERR_INVALID_INPUT, "Unsupported motion command");
  }

  trajectory_msgs::msg::JointTrajectory traj;
  std::vector<Diagnostic> diagnostics;
  const bool ok = compiler_.plan_single_motion(stmt, start.q, traj, diagnostics, cancel);
  log_warnings(diagnostics);
  if (!ok) {
    throw_first_error(diagnostics, err_prefix, "Studio planner failed to generate trajectory");
  }
  return traj;
}

void MotionPlanner::verify_trajectory(const trajectory_msgs::msg::JointTrajectory& traj,
    bool enforce_jerk, const Cancellation& cancel) const {
  std::vector<Diagnostic> diagnostics;
  if (!compiler_.verify_joint_trajectory(
        traj, /*check_singularity_along_path=*/false, enforce_jerk, diagnostics, cancel)) {
    throw_first_error(diagnostics, "", "Trajectory verification failed");
  }
}

} // namespace lite6_planner
