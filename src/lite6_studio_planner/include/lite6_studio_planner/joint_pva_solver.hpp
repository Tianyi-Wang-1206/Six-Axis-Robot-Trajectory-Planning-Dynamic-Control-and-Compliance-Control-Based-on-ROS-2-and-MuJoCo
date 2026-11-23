#ifndef LITE6_STUDIO_PLANNER__JOINT_PVA_SOLVER_HPP_
#define LITE6_STUDIO_PLANNER__JOINT_PVA_SOLVER_HPP_

#include "lite6_studio_planner/dsl_ast.hpp"
#include "lite6_studio_planner/topp_jerk_solver.hpp"
#include "lite6_planner/kinematics_engine.hpp"

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace lite6_studio_planner
{

// Full 3rd-order time-optimal joint-space multi-waypoint generator ("Full Ruckig" /
// Ruckig-Pro-equivalent) built on top of Ruckig Community's analytic 7-segment solver:
//
// Layer A — Exact hard position-limit enforcement ([min_pos, max_pos]) and exact dt
//   support per segment:
//   - Verifies forward-time and reverse-time minimum braking feasibility from both
//     endpoints against [min_pos, max_pos].
//   - Enforces Pontryagin state-constrained minimum-time boundary clamping whenever an
//     unconstrained 3rd-order profile overshoots a position limit, splitting the DoF
//     through (q_lim, 0, 0) (or minimal bounce acceleration when zero-acc braking exceeds
//     q_lim) with recursive double-sided bounce handling and dwell / extended-subsegment
//     slack absorption during 6-axis synchronization.
//   - Supports exact integer-millisecond segment duration (round_1ms(dt)) via
//     minimum_duration, resolving Ruckig blocked time intervals cleanly via a 2-section
//     split or verified SepticHermiteSegment fallback.
//
// Layer B — Multi-waypoint (v_k, a_k) optimization for unspecified intermediate states:
//   - Pins every user-supplied intermediate velocity/acceleration (and rest at start/end),
//     initializes omitted interior velocities via braking-safe Catmull-Rom tangents and
//     omitted accelerations to 0, and optimizes free interior (v_k, a_k) coordinates over
//     the Layer-A segment duration oracle to minimize total trajectory duration while
//     strictly obeying position, velocity, acceleration, jerk, and explicit dt bounds.
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
    const std::array<lite6_planner::JointLimit, 6> & limits,
    double vel_scale,
    double acc_scale,
    double jerk_scale,
    const SourceLocation & loc,
    SampledTrajectory1kHz & out_traj,
    std::string & err_code,
    std::string & err_msg);

  // Solves a full multi-waypoint joint-space MovePVA starting from rest at start_q and
  // ending at rest at waypoints.back().joint_pos. Unspecified intermediate velocities
  // and accelerations are optimized to minimize total duration.
  static bool solve_multi_waypoint(
    const std::array<double, 6> & start_q,
    const std::vector<PVAWaypoint> & waypoints,
    const std::array<lite6_planner::JointLimit, 6> & limits,
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
