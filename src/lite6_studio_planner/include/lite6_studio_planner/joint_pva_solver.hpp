#ifndef LITE6_STUDIO_PLANNER__JOINT_PVA_SOLVER_HPP_
#define LITE6_STUDIO_PLANNER__JOINT_PVA_SOLVER_HPP_

#include "lite6_studio_planner/dsl_ast.hpp"
#include "lite6_studio_planner/topp_jerk_solver.hpp"
#include "lite6_kinematics/kinematics_engine.hpp"

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace lite6_studio_planner
{

// Native 6-DOF 3rd-order time-optimal joint-space multi-waypoint PVA solver built on
// the standalone analytic 3rd-order OTG engine (third_order_otg.hpp, no external Ruckig):
//
// Layer A — Exact analytic 3rd-order state-to-state segment solver with hard position
//   limits [min_pos, max_pos], exact blocked-interval synchronization, and exact dt:
//   - Verifies forward-time and reverse-time minimum braking feasibility from both
//     endpoints against [min_pos, max_pos] and scaled (v, a, j) limits.
//   - Enforces hard position limits via continuous 1D Pontryagin optimization over inward
//     boundary contact acceleration a_b (capturing both touch-and-go bounces and
//     zero-acceleration boundary arcs with recursive double-sided bounce handling).
//   - Synchronizes all 6 joints using exact blocked-interval union skipping and supports
//     exact integer-millisecond segment durations (round_1ms(dt)) with piecewise-constant
//     jerk throughout.
//
// Layer B — Deterministic coupled multi-waypoint (v_k, a_k) optimization:
//   - Pins user-supplied intermediate velocities/accelerations (and rest at start/end).
//   - Evaluates natural C^2 cubic-spline, direction-preserving Catmull-Rom, and zero seeds.
//   - Runs per-DoF chain pre-optimization, coupled 6-DOF smooth-max (LogSumExp) projected
//     L-BFGS continuation over all free (v_k, a_k) simultaneously, and a deterministic
//     exact 6-DOF synchronized polish (blocked-interval aware, no wall-clock timers).
class JointPVASolver
{
public:
  struct SegmentInput
  {
    std::array<double, 6> q0{};
    std::array<double, 6> v0{};
    std::array<double, 6> a0{};
    std::array<double, 6> q1{};
    std::array<double, 6> v1{};
    std::array<double, 6> a1{};
    std::optional<double> explicit_dt;
  };

  struct MultiWaypointResult
  {
    SampledTrajectory1kHz trajectory;
    std::vector<double> segment_durations;
    std::vector<std::array<double, 6>> optimized_v;  // size M + 1
    std::vector<std::array<double, 6>> optimized_a;  // size M + 1
  };

  // Solves a single 6-DOF state-to-state segment under scaled (v, a, j) limits,
  // hard joint position limits [min_pos, max_pos], and optional exact dt (rounded to 1 ms).
  static bool solve_segment(
    const SegmentInput & seg,
    const std::array<lite6_kinematics::JointLimit, 6> & limits,
    double vel_scale,
    double acc_scale,
    double jerk_scale,
    const SourceLocation & loc,
    SampledTrajectory1kHz & out_traj,
    std::string & err_code,
    std::string & err_msg);

  // Solves a multi-waypoint joint-space MovePVA starting from rest at start_q and
  // ending at rest at waypoints.back().joint_pos. Unspecified intermediate velocities
  // and accelerations are optimized deterministically to minimize synchronized duration.
  static bool solve_multi_waypoint(
    const std::array<double, 6> & start_q,
    const std::vector<PVAWaypoint> & waypoints,
    const std::array<lite6_kinematics::JointLimit, 6> & limits,
    double vel_scale,
    double acc_scale,
    double jerk_scale,
    MultiWaypointResult & out_result,
    SourceLocation & err_loc,
    std::string & err_code,
    std::string & err_msg);
};

}  // namespace lite6_studio_planner

#endif  // LITE6_STUDIO_PLANNER__JOINT_PVA_SOLVER_HPP_
