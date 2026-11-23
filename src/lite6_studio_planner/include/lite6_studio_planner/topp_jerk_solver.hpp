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

// Direct-transcription 3rd-order time-optimal path parameterization (TOPP) solver
// under joint velocity, joint acceleration, joint jerk, and optional Cartesian
// translational/rotational velocity and acceleration limits.
//
// Algorithm:
// 1. Builds a curvature-adaptive path grid on u in [0, 1] pinning all piece boundaries
//    and adding geometrically graded boundary intervals sized to the rest-to-rest jerk
//    ramp distances at u = 0 and u = 1.
// 2. Runs exact 2nd-order TOPP-RA (backward reachability via Fourier-Motzkin elimination
//    and greedy forward pass on x = du/dt^2) to obtain a 2nd-order warm-start profile.
// 3. Solves the exact piecewise-constant-jerk 3rd-order direct-transcription nonlinear
//    program minimizing T = sum_i dt_i over (v_i, a_i, dt_i) subject to exact cubic-jerk
//    transition equalities, rest-to-rest boundary conditions (v = 0, a = 0 at u = 0, 1),
//    endpoint and Hermite-Simpson exact-time-midpoint velocity/acceleration/jerk bounds,
//    and exact smooth Euclidean Cartesian acceleration norm bounds.
//    Solved via a banded primal-dual interior-point Newton method (half-bandwidth 6 SQD
//    LDL^T with inertia regularization, exact Lagrangian Hessian, filter line search,
//    and second-order corrections) with fixed-point time-midpoint refinement — no post-hoc
//    boxcar smoothing, heuristic limit tightening, or uniform time dilation.
// 4. Evaluates the exact piecewise-cubic timing law at 1 kHz plus every collocation node
//    (left/right one-sided limits) and intra-interval sub-samples (verifying joint
//    vel/acc, continuous and 1 ms finite-difference joint jerk, and analytic Cartesian
//    path vel/acc). If any sample violates a bound, inserts collocation knots at the
//    offending intervals and re-solves the warm-started interior-point problem.
//
// Mathematical scope:
// - The 3rd-order jerk-constrained path parameterization NLP is non-convex; the solver
//   converges to KKT (locally time-optimal) solutions of the discretized collocation
//   problem with continuous and 1 kHz verification, rather than proving global
//   continuous-time optimality. If the IPM falls back to a primal-feasible non-KKT
//   solution, a structured warning is emitted via out_warnings.
struct TOPPSolveOptions
{
  // When > 0, overrides the IPM iteration cap (useful for testing suboptimal fallback).
  int max_ipm_iters_override = 0;
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
    const std::array<lite6_planner::JointLimit, 6> & joint_limits,
    const lite6_planner::CartesianLimit & cart_limits,
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
