#ifndef LITE6_PLANNER_MOTION_PLANNER_HPP_
#define LITE6_PLANNER_MOTION_PLANNER_HPP_

#include "lite6_planner/kinematics_engine.hpp"
#include "lite6_interfaces/action/industrial_motion.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <Eigen/Dense>
#include <array>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace lite6_planner {

class MotionError final : public std::runtime_error {
public:
  MotionError(int32_t code,const std::string& msg):std::runtime_error(msg),code(code){}
  int32_t code;
};

class MotionPlanner {
public:
  using Goal = lite6_interfaces::action::IndustrialMotion::Goal;
  using Cancellation = std::function<bool()>;
  struct InitialState {
    std::array<double,6> q{};
    std::array<double,6> dq{};
  };
  explicit MotionPlanner(const KinematicsEngine& engine);
  trajectory_msgs::msg::JointTrajectory plan(const Goal& goal, const InitialState& initial,
      const Cancellation& cancelled) const;
  void verify_trajectory(const trajectory_msgs::msg::JointTrajectory& trajectory,
      bool enforce_jerk, const Cancellation& cancelled) const;

private:
  const KinematicsEngine& engine_;
  struct CartesianPath;
  struct JointSpline;
  Eigen::Matrix4d pose_matrix(const geometry_msgs::msg::Pose& pose) const;
  trajectory_msgs::msg::JointTrajectory plan_ptp(const Goal& goal,const InitialState& initial,
      const Cancellation& cancelled) const;
  trajectory_msgs::msg::JointTrajectory ruckig_ptp(
      const InitialState& initial, const std::array<double,6>& target,
      double vscale,double ascale,const Cancellation& cancelled) const;
  trajectory_msgs::msg::JointTrajectory plan_cartesian(const Goal& goal,const InitialState& initial,
      const Cancellation& cancelled) const;
  static void check_cancelled(const Cancellation& cancelled);
};

} // namespace lite6_planner
#endif
