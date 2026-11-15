#ifndef LITE6_STUDIO_PLANNER__S_CURVE_SOLVER_HPP_
#define LITE6_STUDIO_PLANNER__S_CURVE_SOLVER_HPP_

#include "lite6_studio_planner/dsl_ast.hpp"
#include "lite6_planner/kinematics_engine.hpp"

#include <array>
#include <vector>

namespace lite6_studio_planner
{

// Analytical 7-segment 3rd-order time-optimal S-curve (Double-S) profile for a single axis.
class SingleAxisSCurve
{
public:
  SingleAxisSCurve() = default;

  // Builds a time-optimal rest-to-rest 7-segment S-curve from p0 to p1 under (v_max, a_max, j_max).
  void build(double p0, double p1, double v_max, double a_max, double j_max);

  double duration() const { return total_time_; }

  void eval(double t, double & pos, double & vel, double & acc, double & jerk) const;

private:
  double p0_ = 0.0;
  double p1_ = 0.0;
  double sign_ = 1.0;
  double j_lim_ = 0.0;
  std::array<double, 7> dt_{};
  std::array<double, 8> t_cum_{};
  std::array<double, 8> p_node_{};
  std::array<double, 8> v_node_{};
  std::array<double, 8> a_node_{};
  std::array<double, 7> j_seg_{};
  double total_time_ = 0.0;
};

// Synchronized 6-axis time-optimal S-curve profile.
class MultiAxisSCurve
{
public:
  MultiAxisSCurve() = default;

  void build(
    const std::array<double, 6> & q0,
    const std::array<double, 6> & q1,
    const std::array<lite6_planner::JointLimit, 6> & limits,
    double vel_scale,
    double acc_scale,
    double jerk_scale = 1.0);

  double duration() const { return sync_duration_; }

  void eval(
    double t,
    std::array<double, 6> & q,
    std::array<double, 6> & dq,
    std::array<double, 6> & ddq,
    std::array<double, 6> & jrk) const;

private:
  std::array<SingleAxisSCurve, 6> axes_{};
  double sync_duration_ = 0.0;
};

// Degree-7 C^3 Hermite polynomial segment matching (q0, v0, a0, j0=0) -> (q1, v1, a1, j1=0).
class SepticHermiteSegment
{
public:
  SepticHermiteSegment() = default;

  // Builds the segment with explicit duration T > 0.
  void build_with_duration(
    const std::array<double, 6> & q0,
    const std::array<double, 6> & v0,
    const std::array<double, 6> & a0,
    const std::array<double, 6> & q1,
    const std::array<double, 6> & v1,
    const std::array<double, 6> & a1,
    double duration);

  // Computes the minimum feasible duration T* satisfying all 6-axis (v_max, a_max, j_max) bounds.
  void build_time_optimal(
    const std::array<double, 6> & q0,
    const std::array<double, 6> & v0,
    const std::array<double, 6> & a0,
    const std::array<double, 6> & q1,
    const std::array<double, 6> & v1,
    const std::array<double, 6> & a1,
    const std::array<lite6_planner::JointLimit, 6> & limits,
    double vel_scale,
    double acc_scale,
    double jerk_scale = 1.0);

  double duration() const { return duration_; }

  void eval(
    double t,
    std::array<double, 6> & q,
    std::array<double, 6> & dq,
    std::array<double, 6> & ddq,
    std::array<double, 6> & jrk) const;

private:
  std::array<std::array<double, 8>, 6> coeffs_{};
  double duration_ = 0.001;
};

}  // namespace lite6_studio_planner

#endif  // LITE6_STUDIO_PLANNER__S_CURVE_SOLVER_HPP_
