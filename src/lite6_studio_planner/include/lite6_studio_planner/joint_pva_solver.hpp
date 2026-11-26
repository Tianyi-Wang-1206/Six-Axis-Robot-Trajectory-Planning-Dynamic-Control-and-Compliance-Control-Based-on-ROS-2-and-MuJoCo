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

// Six-axis jerk-limited PVA planning with bounded-position OTG segments.
// Synchronizes 1 ms durations and optimizes free waypoint derivatives with
// deterministic, cancellable L-BFGS/coordinate searches; pinned PVA stays fixed.
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

  // Plan rest-to-rest joint MovePVA, optimizing unspecified interior derivatives.
  // Cancellation returns ERR_CANCELLED.
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
