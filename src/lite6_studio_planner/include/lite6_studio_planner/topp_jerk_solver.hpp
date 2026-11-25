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
// 3. Solves a discretized piecewise-constant-jerk 3rd-order direct-transcription nonlinear
//    program minimizing T = sum_i dt_i over (v_i, a_i, dt_i) subject to exact cubic-jerk
//    transition equalities, rest-to-rest boundary conditions (v = 0, a = 0 at u = 0, 1),
//    endpoint and time-midpoint velocity/acceleration/jerk bounds, and smooth Euclidean
//    Cartesian acceleration norm bounds.
//    Solved via a banded primal-dual interior-point Newton method (half-bandwidth 6 SQD
//    LDL^T with inertia regularization, filter line search, and second-order corrections).
//    Within each IPM subproblem, midpoint path derivatives q'(u_m), q''(u_m), q'''(u_m) are
//    held frozen (omitting chain-rule terms in du_m/dx) and refreshed via an outer fixed-point
//    loop — no post-hoc boxcar smoothing, heuristic limit tightening, or uniform time dilation.
// 4. Performs finite-sample kinematic verification on the piecewise-cubic timing law at every
//    collocation node (left/right one-sided limits), intra-interval sub-samples
//    tau in {0, 0.25, 0.5, 0.75, 1}, interior cubic speed-peak extrema, and all 1 kHz output
//    ticks (checking joint vel/acc, continuous and 1 ms finite-difference joint jerk, and
//    analytic Cartesian path vel/acc). If any checked sample violates a bound, inserts
//    collocation knots at the offending intervals and re-solves the warm-started IPM problem.
//
// Mathematical scope:
// - The 3rd-order jerk-constrained path parameterization NLP is non-convex, and the IPM
//   solves a frozen-midpoint collocation approximation refreshed via outer fixed-point
//   updates. The solver therefore returns a numerically optimized, feasibility-checked
//   trajectory without certifying KKT stationarity of the full moving-midpoint problem or
//   proving local/global continuous-time optimality.
// - Stage-4 verification checks a dense finite sample set (collocation nodes, quarter-interval
//   sub-samples, interior cubic speed peaks, and 1 kHz ticks); it is practical numerical
//   validation rather than a continuous-time interval-arithmetic bound proof.
// - If the frozen-midpoint IPM terminates on a primal-feasible fallback before reaching its
//   convergence tolerances, a structured warning is emitted via out_warnings.
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
