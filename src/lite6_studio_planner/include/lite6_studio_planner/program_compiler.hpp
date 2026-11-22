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
  WAIT_CMD,
  PAYLOAD_CMD
};

enum class BlockSourceKind
{
  NONE,
  CARTESIAN_GROUP,
  JOINT_GROUP,
  MOVE_PVA
};

struct ExecutionBlock
{
  ExecutionBlockType type = ExecutionBlockType::RIGID_TRAJECTORY;
  BlockSourceKind source_kind = BlockSourceKind::NONE;
  SourceLocation loc;
  std::array<double, 6> start_joints{};
  std::array<double, 6> end_joints{};
  double duration_sec = 0.0;

  // Populated when type == RIGID_TRAJECTORY
  trajectory_msgs::msg::JointTrajectory rigid_trajectory;

  // Populated when type == IMPEDANCE_TRAJECTORY
  lite6_interfaces::action::FollowImpedanceTrajectory::Goal impedance_goal;
  ImpedanceParams impedance_params;

  // Original AST source group retained for dynamic post-impedance start-pose replanning
  std::vector<MotionStatement> source_motions;
  MovePVAStatement source_pva;

  // Per-1kHz-sample source location mapping for live line highlighting and diagnostics
  std::vector<SourceLocation> sample_locations;

  // Per-1kHz-sample normalized parameter u in [0, 1] on the block's source geometric path
  // (the full path from path_start_joints), used to resume an interrupted block.
  std::vector<double> sample_path_u;
  // Joint configuration at which the source geometric path starts (u = 0).
  std::array<double, 6> path_start_joints{};

  // Populated when type == GRIPPER_CMD
  GripperStatement gripper_cmd;

  // Populated when type == WAIT_CMD
  WaitStatement wait_cmd;

  // Populated when type == PAYLOAD_CMD
  PayloadStatement payload_cmd;
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
  // Joint speed cap [rad/s] for impedance trajectory blocks: 90 % of the CTC
  // impedance_execution.max_joint_velocity (3.0 rad/s), whose violation latches E-STOP.
  static constexpr double kImpedanceJointVelocityCap = 2.7;

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

  enum class PauseSpliceStatus
  {
    OK,           // out_tail holds a stop-along-path splice
    LET_FINISH,   // the remaining motion is shorter than a path-preserving stop
    FAILED        // no admissible splice (caller falls back to a controller cancel)
  };

  // Builds a path-preserving stop for a running rigid block: from the 1 kHz sample at
  // or after t_switch_sec (block time base), the remaining reference q(tau) is
  // re-timed with tau_dot = sigma(t), where sigma falls from 1 to 0 along a septic
  // (C^3) smoothstep, so the arm decelerates to rest ON the programmed path within
  // joint velocity/acceleration/jerk limits. out_tail starts with the original sample
  // verbatim (same time_from_start) for a seamless CTC splice; out_end_sample is the
  // nearest original sample index to the stopping point.
  PauseSpliceStatus plan_pause_splice(
    const ExecutionBlock & block,
    double t_switch_sec,
    trajectory_msgs::msg::JointTrajectory & out_tail,
    size_t & out_end_sample,
    std::string & out_error) const;

  // Replans an entire trajectory block directly from q_actual (used when an earlier impedance
  // block finished compliantly deflected against physical contact).
  bool replan_block_from_start(
    const ExecutionBlock & original_block,
    const std::array<double, 6> & q_actual,
    ExecutionBlock & out_replanned_block,
    std::vector<Diagnostic> & out_diagnostics) const;

private:
  const lite6_planner::KinematicsEngine & engine_;

  bool validate_and_clamp_start_joints(
    const std::array<double, 6> & q_in,
    const SourceLocation & loc,
    std::array<double, 6> & q_out,
    std::vector<Diagnostic> & diags) const;

  bool compile_cartesian_group(
    const std::vector<MotionStatement> & group,
    const std::array<double, 6> & start_q,
    bool is_impedance,
    const ImpedanceParams & imp_params,
    ExecutionBlock & out_block,
    std::vector<Diagnostic> & out_diagnostics) const;

  bool compile_joint_group(
    const std::vector<MotionStatement> & group,
    const std::array<double, 6> & start_q,
    ExecutionBlock & out_block,
    std::vector<Diagnostic> & out_diagnostics) const;

  // Joint-space Hermite path through IK knots of a Cartesian path; nodal derivatives
  // dq/du, d2q/du2 come from differential kinematics on the analytic Cartesian path
  // derivatives (d3q/du3 by central difference of the analytic d2q/du2).
  bool build_cartesian_joint_path(
    const CartesianPath & cart_path,
    const std::array<double, 6> & start_q,
    C3JointSpline & out_path,
    std::vector<Diagnostic> & out_diagnostics) const;

  // Cartesian path -> joint path -> TOPP-RA -> 1 kHz verification.
  bool plan_cartesian_path(
    const CartesianPath & cart_path,
    const std::array<double, 6> & start_q,
    const MotionOptions & default_options,
    const SourceLocation & default_loc,
    bool is_impedance,
    SampledTrajectory1kHz & out_sampled,
    std::vector<Diagnostic> & out_diagnostics) const;

  // Joint limits used to plan and verify a block. Impedance blocks additionally cap
  // max_vel at kImpedanceJointVelocityCap so the measured joint speed (reference plus
  // compliant tracking lag) stays below the controller's impedance E-STOP threshold.
  std::array<lite6_planner::JointLimit, 6> motion_limits(bool is_impedance) const;

  // Resolves joint-group waypoints and builds the blended joint path (>= 2 statements).
  bool build_joint_group_path(
    const std::vector<MotionStatement> & group,
    const std::array<double, 6> & start_q,
    std::vector<std::array<double, 6>> & out_waypoints,
    JointCompositePath & out_path,
    std::vector<Diagnostic> & out_diagnostics) const;

  // Joint-space length removed on the segment from corner_q towards neighbor_q so the
  // TCP lies blend_radius away from its position at corner_q (capped at 45 % of the segment).
  double tcp_blend_trim(
    const std::array<double, 6> & corner_q,
    const std::array<double, 6> & neighbor_q,
    double blend_radius) const;

  bool compile_pva_statement(
    const MovePVAStatement & pva,
    const std::array<double, 6> & start_q,
    bool is_impedance,
    const ImpedanceParams & imp_params,
    ExecutionBlock & out_block,
    std::vector<Diagnostic> & out_diagnostics) const;

  bool verify_sampled_trajectory_1khz(
    const SampledTrajectory1kHz & sampled,
    bool check_singularity_along_path,
    std::vector<Diagnostic> & diags,
    bool is_impedance = false) const;
};

}  // namespace lite6_studio_planner

#endif  // LITE6_STUDIO_PLANNER__PROGRAM_COMPILER_HPP_
