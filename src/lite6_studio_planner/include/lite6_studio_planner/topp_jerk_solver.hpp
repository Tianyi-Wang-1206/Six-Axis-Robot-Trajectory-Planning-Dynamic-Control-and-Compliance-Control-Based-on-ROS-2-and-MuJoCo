#ifndef LITE6_STUDIO_PLANNER__TOPP_JERK_SOLVER_HPP_
#define LITE6_STUDIO_PLANNER__TOPP_JERK_SOLVER_HPP_

#include "lite6_studio_planner/curve_geometry.hpp"
#include "lite6_studio_planner/s_curve_solver.hpp"
#include "lite6_kinematics/kinematics_engine.hpp"

#include <array>
#include <functional>
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

// Jerk-limited path timing: TOPP-RA warm start, banded IPM and adaptive refinement.
// Midpoint derivatives are frozen per subproblem and refreshed iteratively.
// Checks finite samples and 1 kHz output; optimality and continuous-time bounds
// are not certified. Feasible early exits are reported through out_warnings.
struct TOPPSolveOptions
{
  // When > 0, overrides the IPM iteration cap (useful for testing suboptimal fallback).
  int max_ipm_iters_override = 0;
  // Optional cooperative cancellation predicate checked during grid/IPM/sampling loops.
  std::function<bool()> is_cancelled;
};

class TOPPJerkSolver
{
public:
  TOPPJerkSolver() = default;

  // Per-u motion options come from cart_path when given, else from path.options_at(),
  // else from the default scales and location.
  static bool solve(
    const JointPath & path,
    const CartesianPath * cart_path,
    const std::array<lite6_kinematics::JointLimit, 6> & joint_limits,
    const lite6_kinematics::CartesianLimit & cart_limits,
    double default_vel_scale,
    double default_acc_scale,
    double default_jerk_scale,
    const SourceLocation & default_loc,
    SampledTrajectory1kHz & out_traj,
    std::string & err_msg,
    std::vector<std::string> * out_warnings = nullptr,
    const TOPPSolveOptions & solve_options = TOPPSolveOptions{});
};

}  // namespace lite6_studio_planner

#endif  // LITE6_STUDIO_PLANNER__TOPP_JERK_SOLVER_HPP_
