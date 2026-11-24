#ifndef LITE6_STUDIO_PLANNER__THIRD_ORDER_OTG_HPP_
#define LITE6_STUDIO_PLANNER__THIRD_ORDER_OTG_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace lite6_studio_planner
{
namespace otg
{

enum class ControlInterface
{
  POSITION,
  VELOCITY
};

enum class Synchronization
{
  TIME,
  PHASE,
  NONE
};

enum class DurationDiscretization
{
  CONTINUOUS,
  DISCRETE
};

// Kinematic and hard position bounds for a single degree of freedom.
// Supports asymmetric velocity and acceleration limits:
//   q_min <= q(t) <= q_max
//   v_min <= v(t) <= v_max   (v_min < 0 < v_max)
//   a_min <= a(t) <= a_max   (a_min < 0 < a_max)
//   |j(t)| <= j_max          (j_max > 0)
struct AxisLimits
{
  double q_min = -std::numeric_limits<double>::infinity();
  double q_max = std::numeric_limits<double>::infinity();
  double v_min = -1.0;
  double v_max = 1.0;
  double a_min = -1.0;
  double a_max = 1.0;
  double j_max = 1.0;

  bool has_pos_limits() const
  {
    return std::isfinite(q_min) || std::isfinite(q_max);
  }
};

struct AxisState
{
  double q = 0.0;
  double v = 0.0;
  double a = 0.0;
};

struct PositionExtrema
{
  double min = std::numeric_limits<double>::infinity();
  double max = -std::numeric_limits<double>::infinity();
  double t_min = 0.0;
  double t_max = 0.0;
};

// Exact piecewise-constant-jerk 1-DOF trajectory representation.
// Each phase i spans [t_cum[i], t_cum[i+1]] with constant jerk j[i].
// Evaluating at any t in [0, total_duration] is exact and O(log K).
struct AxisProfile
{
  struct Phase
  {
    double dt = 0.0;
    double q0 = 0.0;
    double v0 = 0.0;
    double a0 = 0.0;
    double j = 0.0;
  };

  std::vector<Phase> phases;
  std::vector<double> t_cum;  // size == phases.size() + 1, t_cum[0] = 0
  double duration = 0.0;
  double qf = 0.0;
  double vf = 0.0;
  double af = 0.0;
  PositionExtrema extrema{};

  void clear();
  void append_phase(double dt, double q0, double v0, double a0, double j);
  void append_profile(const AxisProfile & other);
  void finalize();

  void eval(double t, double & q, double & v, double & a, double & j) const;
  bool empty() const { return phases.empty(); }
};

// Closed-form minimum upper / maximum lower braking position excursions from (q0, v0, a0)
// under asymmetric (a_min, a_max, j_max).
double min_upper_excursion(double q0, double v0, double a0, double a_min, double j_max);
double max_lower_excursion(double q0, double v0, double a0, double a_max, double j_max);

// Exact maximum |v| with a = 0 whose stopping excursion to (0, 0) stays within distance d >= 0.
double max_speed_for_distance(double d, double a_brake, double j_max);

// Checks whether a 1-DOF state (q, v, a) is valid and forward/reverse braking-feasible
// within AxisLimits.
bool validate_axis_state(
  const AxisState & st,
  const AxisLimits & lim,
  bool check_forward,
  bool check_reverse,
  const char ** out_error_tag = nullptr);

// Projects (v, a) at fixed position q into the strictly forward+reverse braking-feasible set.
void project_state_feasible(
  double q,
  double & v,
  double & a,
  const AxisLimits & lim,
  double headroom = 1.0);

// Exact 1-DOF minimum-time velocity transition (v0, a0) -> (vf, af) ignoring position.
bool solve_axis_velocity_min_time(
  const AxisState & s0,
  double vf,
  double af,
  const AxisLimits & lim,
  AxisProfile & out_profile);

// Exact 1-DOF velocity transition at exact duration T >= T_min_vel.
bool solve_axis_velocity_at_duration(
  const AxisState & s0,
  double vf,
  double af,
  const AxisLimits & lim,
  double T_target,
  AxisProfile & out_profile);

// Result of 1-DOF unconstrained Step 1: exact minimum duration and any physical blocked
// duration intervals (b_lo, b_hi) where no (v, a, j)-limited trajectory exists.
struct Step1Result
{
  bool ok = false;
  double t_min = std::numeric_limits<double>::infinity();
  AxisProfile min_profile;
  std::vector<std::pair<double, double>> blocked_intervals;
};

// Exact 1-DOF unconstrained Step 1 (position control, ignoring [q_min, q_max]).
Step1Result solve_axis_unconstrained_step1(
  const AxisState & s0,
  const AxisState & sf,
  const AxisLimits & lim);

// Exact 1-DOF unconstrained displacement reachability at duration T:
// computes the exact [dp_min(T), dp_max(T)] interval and achieving profiles.
bool compute_displacement_bounds(
  const AxisState & s0,
  const AxisState & sf,
  const AxisLimits & lim,
  double T,
  double & out_dp_min,
  double & out_dp_max,
  AxisProfile * out_min_prof = nullptr,
  AxisProfile * out_max_prof = nullptr);

// Exact 1-DOF unconstrained Step 2 at exact target duration T.
// When lim.has_pos_limits() is true, among valid unconstrained profiles at duration T
// prefers one staying strictly within [q_min, q_max] (and with lowest peak jerk).
bool solve_axis_unconstrained_step2(
  const AxisState & s0,
  const AxisState & sf,
  const AxisLimits & lim,
  double T_target,
  AxisProfile & out_profile);

// Exact 1-DOF state-constrained minimum-time solver enforcing q(t) in [q_min, q_max].
// Uses continuous 1D Pontryagin optimization over inward boundary contact acceleration a_b
// (covering both touch-and-go bounces and zero-acceleration boundary arcs, with recursion
// for double-sided bounces).
bool solve_axis_bounded_min_time(
  const AxisState & s0,
  const AxisState & sf,
  const AxisLimits & lim,
  AxisProfile & out_profile,
  bool * out_clamped = nullptr,
  int depth = 0);

// Exact 1-DOF state-constrained fixed-duration solver enforcing q(t) in [q_min, q_max]
// at exact duration T_target. If infeasible at T_target, sets out_next_feasible_T to a
// strictly larger candidate duration (or +inf if none).
bool solve_axis_bounded_at_duration(
  const AxisState & s0,
  const AxisState & sf,
  const AxisLimits & lim,
  double T_target,
  AxisProfile & out_profile,
  double & out_next_feasible_T,
  int depth = 0);

// Multi-DOF native 3rd-order time-optimal trajectory generator input and output.
struct MultiDoFInput
{
  std::vector<AxisState> current;
  std::vector<AxisState> target;
  std::vector<AxisLimits> limits;
  ControlInterface control_interface = ControlInterface::POSITION;
  Synchronization synchronization = Synchronization::TIME;
  DurationDiscretization discretization = DurationDiscretization::CONTINUOUS;
  double dt_step = 0.001;
  std::optional<double> exact_duration;
  std::optional<double> minimum_duration;
  double max_duration = 120.0;
};

struct MultiDoFResult
{
  bool ok = false;
  double duration = 0.0;
  std::vector<double> independent_min_durations;
  std::vector<AxisProfile> profiles;
  int failed_dof = -1;
  double next_feasible_duration = std::numeric_limits<double>::infinity();
};

// Solves synchronized N-DOF 3rd-order state-to-state trajectory generation with hard
// position limits, exact blocked-interval handling, and optional exact/minimum duration.
MultiDoFResult solve_multidof(const MultiDoFInput & input);

}  // namespace otg
}  // namespace lite6_studio_planner

#endif  // LITE6_STUDIO_PLANNER__THIRD_ORDER_OTG_HPP_
