#ifndef LITE6_STUDIO_PLANNER__PROGRAM_COMPILER_HPP_
#define LITE6_STUDIO_PLANNER__PROGRAM_COMPILER_HPP_

#include "lite6_studio_planner/curve_geometry.hpp"
#include "lite6_studio_planner/dsl_ast.hpp"
#include "lite6_studio_planner/dsl_parser.hpp"
#include "lite6_studio_planner/s_curve_solver.hpp"
#include "lite6_studio_planner/topp_jerk_solver.hpp"
#include "lite6_planner/kinematics_engine.hpp"
#include "lite6_interfaces/action/follow_impedance_trajectory.hpp"
#include "lite6_interfaces/msg/compile_diagnostic.hpp"

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <array>
#include <string>
#include <vector>

namespace lite6_studio_planner
{

enum class ExecutionBlockType
{
  RIGID_TRAJECTORY,
  IMPEDANCE_TRAJECTORY,
  GRIPPER_CMD,
  WAIT_CMD
};

struct ExecutionBlock
{
  ExecutionBlockType type = ExecutionBlockType::RIGID_TRAJECTORY;
  SourceLocation loc;
  std::array<double, 6> start_joints{};
  std::array<double, 6> end_joints{};
  double duration_sec = 0.0;

  // Populated when type == RIGID_TRAJECTORY
  trajectory_msgs::msg::JointTrajectory rigid_trajectory;

  // Populated when type == IMPEDANCE_TRAJECTORY
  lite6_interfaces::action::FollowImpedanceTrajectory::Goal impedance_goal;
  ImpedanceParams impedance_params;

  // Per-1kHz-sample source location mapping for live line highlighting and diagnostics
  std::vector<SourceLocation> sample_locations;

  // Populated when type == GRIPPER_CMD
  GripperStatement gripper_cmd;

  // Populated when type == WAIT_CMD
  WaitStatement wait_cmd;
};

struct CompiledProgram
{
  std::string program_id;
  std::string source_code;
  uint32_t total_statements = 0;
  uint32_t total_blocks = 0;
  double estimated_duration_sec = 0.0;
  uint32_t total_samples_1khz = 0;
  std::array<double, 6> initial_joints{};
  std::vector<ExecutionBlock> blocks;
};

class ProgramCompiler
{
public:
  using Diagnostic = lite6_interfaces::msg::CompileDiagnostic;

  explicit ProgramCompiler(const lite6_planner::KinematicsEngine & engine);

  // Executes the full 5-stage compilation & 1 kHz verification pipeline:
  //   Stage 1: Lexical & Syntax check
  //   Stage 2: Semantic & Parameter check
  //   Stage 3: Time-Optimal Trajectory Planning & 1 kHz Discretization
  //   Stage 4: Full 1 kHz Kinematic Limits Verification (pos, vel, acc, jerk, singularity)
  //   Stage 5: Full 1 kHz Coal Self & Environment Collision Verification
  bool compile(
    const std::string & source_code,
    const std::array<double, 6> & start_joints,
    CompiledProgram & out_program,
    std::vector<Diagnostic> & out_diagnostics) const;

  // Plans and verifies a smooth 1 kHz point-to-point joint alignment bridge trajectory.
  bool plan_alignment_bridge(
    const std::array<double, 6> & q_from,
    const std::array<double, 6> & q_to,
    double vel_scale,
    double acc_scale,
    trajectory_msgs::msg::JointTrajectory & out_traj,
    std::string & out_error) const;

  // Dynamically replans the remaining portion of an interrupted trajectory block from q_actual
  // at sample index pause_sample_idx to the end of the block (for smooth Pause -> Resume).
  bool replan_remaining_block(
    const ExecutionBlock & original_block,
    size_t pause_sample_idx,
    const std::array<double, 6> & q_actual,
    ExecutionBlock & out_resumed_block,
    std::string & out_error) const;

private:
  const lite6_planner::KinematicsEngine & engine_;

  bool verify_sampled_trajectory_1khz(
    const SampledTrajectory1kHz & sampled,
    bool check_singularity_along_path,
    std::vector<Diagnostic> & diags) const;
};

}  // namespace lite6_studio_planner

#endif  // LITE6_STUDIO_PLANNER__PROGRAM_COMPILER_HPP_
