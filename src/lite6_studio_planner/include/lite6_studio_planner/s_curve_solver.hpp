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

// Phase-synchronized 6-axis S-curve: every joint follows q(t) = q0 + (q1 - q0) * s(t) with
// one shared normalized jerk-limited profile s(t) in [0, 1], so the motion is a straight line
// in joint space. s(t) uses the tightest per-joint (v, a, j) limit mapped to the normalized
// stroke (with a small numerical safety margin) and rounds duration up to 1 ms.
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

  // Normalized path progress s(t) in [0, 1].
  double progress(double t) const;

  void eval(
    double t,
    std::array<double, 6> & q,
    std::array<double, 6> & dq,
    std::array<double, 6> & ddq,
    std::array<double, 6> & jrk) const;

private:
  SingleAxisSCurve profile_{};
  std::array<double, 6> q0_{};
  std::array<double, 6> delta_{};
  double sync_duration_ = 0.0;
};

}  // namespace lite6_studio_planner

#endif  // LITE6_STUDIO_PLANNER__S_CURVE_SOLVER_HPP_
