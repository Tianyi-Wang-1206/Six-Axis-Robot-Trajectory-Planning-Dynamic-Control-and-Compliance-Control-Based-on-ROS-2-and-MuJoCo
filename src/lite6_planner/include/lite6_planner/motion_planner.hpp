#ifndef LITE6_PLANNER_MOTION_PLANNER_HPP_
#define LITE6_PLANNER_MOTION_PLANNER_HPP_

#include "lite6_kinematics/kinematics_engine.hpp"
#include "lite6_studio_planner/program_compiler.hpp"
#include "lite6_interfaces/action/industrial_motion.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <Eigen/Dense>
#include <array>
#include <functional>
#include <stdexcept>
#include <string>

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
  explicit MotionPlanner(const lite6_kinematics::KinematicsEngine& engine);
  trajectory_msgs::msg::JointTrajectory plan(const Goal& goal, const InitialState& initial,
      const Cancellation& cancelled) const;
  void verify_trajectory(const trajectory_msgs::msg::JointTrajectory& trajectory,
      bool enforce_jerk, const Cancellation& cancelled) const;

private:
  const lite6_kinematics::KinematicsEngine& engine_;
  lite6_studio_planner::ProgramCompiler compiler_;
  Eigen::Matrix4d pose_matrix(const geometry_msgs::msg::Pose& pose) const;
  static void check_cancelled(const Cancellation& cancelled);
};

} // namespace lite6_planner
#endif
