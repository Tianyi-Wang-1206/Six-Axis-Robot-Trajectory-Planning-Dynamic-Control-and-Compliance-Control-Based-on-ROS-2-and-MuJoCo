#ifndef LITE6_STUDIO_PLANNER__TOPP_JERK_SOLVER_HPP_
#define LITE6_STUDIO_PLANNER__TOPP_JERK_SOLVER_HPP_

#include "lite6_studio_planner/curve_geometry.hpp"
#include "lite6_studio_planner/s_curve_solver.hpp"
#include "lite6_planner/kinematics_engine.hpp"

#include <array>
#include <vector>

namespace lite6_studio_planner
{

struct SampledTrajectory1kHz
{
  struct Sample
  {
    double time_sec = 0.0;
    double path_u = 0.0;
    std::array<double, 6> q{};
    std::array<double, 6> dq{};
    std::array<double, 6> ddq{};
    SourceLocation loc;
  };
  std::vector<Sample> samples;
  double duration_sec = 0.0;
};

// Self-developed TOPP-RA Phase-Plane Reachability + Jerk-Constrained S-Curve Time Parameterizer.
class TOPPJerkSolver
{
public:
  TOPPJerkSolver() = default;

  // Solves the time-optimal 1 kHz trajectory along a C^3 joint spline q(u), u in [0, 1],
  // respecting all 6-axis joint (vel, acc, jerk) limits and optional Cartesian path limits.
  static bool solve(
    const C3JointSpline & joint_spline,
    const CompositeCartesianPath * cart_path,
    const std::array<lite6_planner::JointLimit, 6> & joint_limits,
    const lite6_planner::CartesianLimit & cart_limits,
    double default_vel_scale,
    double default_acc_scale,
    double default_jerk_scale,
    const SourceLocation & default_loc,
    SampledTrajectory1kHz & out_traj,
    std::string & err_msg);
};

}  // namespace lite6_studio_planner

#endif  // LITE6_STUDIO_PLANNER__TOPP_JERK_SOLVER_HPP_
