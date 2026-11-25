#ifndef LITE6_STUDIO_PLANNER__JOINT_PVA_SOLVER_HPP_
#define LITE6_STUDIO_PLANNER__JOINT_PVA_SOLVER_HPP_

#include "lite6_studio_planner/dsl_ast.hpp"
#include "lite6_studio_planner/topp_jerk_solver.hpp"
#include "lite6_kinematics/kinematics_engine.hpp"

#include <array>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace lite6_studio_planner
{

// Native 6-DOF 3rd-order joint-space multi-waypoint PVA solver built on the standalone
// 3rd-order OTG engine (third_order_otg.hpp):
//
// Layer A — Synchronized 3rd-order state-to-state segment solver with hard position
//   limits [min_pos, max_pos], unconstrained blocked-interval synchronization, and exact dt:
//   - Verifies forward-time and reverse-time minimum braking feasibility from both
//     endpoints against [min_pos, max_pos] and scaled (v, a, j) limits.
//   - Uses analytic unconstrained 3rd-order OTG where position bounds are inactive, and
//     numerical boundary-contact search (finite grid + Brent refinement + bounded recursion)
//     when position bounds are active, distinguishing mathematical infeasibility from
//     numerical search failure.
//   - Synchronizes all 6 joints and supports 1 ms segment durations (round_1ms(dt)) with
//     piecewise-constant jerk throughout.
//
// Layer B — Deterministic coupled multi-waypoint (v_k, a_k) optimization:
//   - Pins user-supplied intermediate velocities/accelerations (and rest at start/end),
//     computing exact feasible intervals for partially constrained free components.
//   - Evaluates natural C^2 cubic-spline, direction-preserving Catmull-Rom, and zero seeds.
//   - Runs per-DoF chain pre-optimization, coupled 6-DOF smooth-max (LogSumExp) projected
//     L-BFGS continuation over all free (v_k, a_k) simultaneously, and a deterministic
//     6-DOF synchronized coordinate polish (no wall-clock timers, cooperatively cancellable).
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
  // Optional is_cancelled predicate aborts with ERR_CANCELLED.
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
    std::string & err_msg,
    const std::function<bool()> & is_cancelled = {});
};

}  // namespace lite6_studio_planner

#endif  // LITE6_STUDIO_PLANNER__JOINT_PVA_SOLVER_HPP_
