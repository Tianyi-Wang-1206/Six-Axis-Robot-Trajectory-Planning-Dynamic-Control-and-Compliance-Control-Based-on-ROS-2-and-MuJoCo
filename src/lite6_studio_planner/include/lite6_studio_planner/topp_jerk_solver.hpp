#ifndef LITE6_STUDIO_PLANNER__TOPP_JERK_SOLVER_HPP_
#define LITE6_STUDIO_PLANNER__TOPP_JERK_SOLVER_HPP_

#include "lite6_studio_planner/curve_geometry.hpp"
#include "lite6_studio_planner/s_curve_solver.hpp"
#include "lite6_planner/kinematics_engine.hpp"

#include <array>
#include <string>
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

// Time-optimal path parameterization by reachability analysis (TOPP-RA) with jerk limiting.
//
// 1. Adaptive grid on u in [0, 1] (density from joint/Cartesian rates and path curvature).
// 2. TOPP-RA on (x = du/dt^2, a = d2u/dt2): backward pass builds the controllable sets
//    K_i from the exact 2-D projection of the per-stage linear constraints (joint velocity
//    and acceleration, polygonal inner approximations of the Cartesian translational and
//    rotational speed/acceleration norms); a greedy forward pass then picks the maximal
//    admissible acceleration, which is time-optimal for the discretized problem.
// 3. A boxcar filter on the path speed makes the acceleration continuous (finite jerk).
// 4. The 1 kHz trajectory is checked against velocity, acceleration and (discrete) jerk
//    limits; violations tighten the speed / acceleration limits only near the offending
//    path region and TOPP-RA is re-run. A uniform time dilation is a last-resort fallback.
class TOPPJerkSolver
{
public:
  TOPPJerkSolver() = default;

  // Per-u motion options come from cart_path when given, else from path.options_at(),
  // else from the default scales and location.
  static bool solve(
    const JointPath & path,
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
