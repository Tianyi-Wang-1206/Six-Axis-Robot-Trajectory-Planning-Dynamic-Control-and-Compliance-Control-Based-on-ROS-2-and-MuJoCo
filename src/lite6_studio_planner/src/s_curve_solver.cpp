#include "lite6_studio_planner/s_curve_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lite6_studio_planner
{

void SingleAxisSCurve::build(double p0, double p1, double v_max, double a_max, double j_max)
{
  p0_ = p0;
  p1_ = p1;
  const double dist = std::abs(p1 - p0);
  sign_ = (p1 >= p0) ? 1.0 : -1.0;
  j_lim_ = std::max(1e-6, j_max);
  v_max = std::max(1e-6, v_max);
  a_max = std::max(1e-6, a_max);

  if (dist < 1e-11) {
    dt_.fill(0.0);
    t_cum_.fill(0.0);
    p_node_.fill(0.0);
    v_node_.fill(0.0);
    a_node_.fill(0.0);
    j_seg_.fill(0.0);
    total_time_ = 0.0;
    return;
  }

  double Tj = 0.0;
  double Ta = 0.0;
  double Tv = 0.0;

  if (v_max * j_lim_ >= a_max * a_max) {
    // Acceleration reaches a_max during ramp-up if stroke is long enough.
    const double Tj_full = a_max / j_lim_;
    const double Ta_full = Tj_full + v_max / a_max;
    const double D_v = v_max * Ta_full;
    const double D_a = 2.0 * a_max * a_max * a_max / (j_lim_ * j_lim_);

    if (dist >= D_v) {
      // Full 7-segment profile with cruise phase.
      Tj = Tj_full;
      Ta = Ta_full;
      Tv = (dist - D_v) / v_max;
    } else if (dist >= D_a) {
      // 6-segment profile: reaches a_max, no cruise phase (Tv = 0).
      Tj = Tj_full;
      const double ratio = a_max / j_lim_;
      const double v_peak =
        0.5 * a_max * (std::sqrt(ratio * ratio + 4.0 * dist / a_max) - ratio);
      Ta = Tj + v_peak / a_max;
      Tv = 0.0;
    } else {
      // 4-segment pure jerk-limited triangle acceleration profile.
      Tj = std::cbrt(dist / (2.0 * j_lim_));
      Ta = 2.0 * Tj;
      Tv = 0.0;
    }
  } else {
    // Velocity limit v_max is reached before acceleration reaches a_max.
    const double Tj_vel = std::sqrt(v_max / j_lim_);
    const double D_v = 2.0 * v_max * Tj_vel;
    if (dist >= D_v) {
      Tj = Tj_vel;
      Ta = 2.0 * Tj;
      Tv = (dist - D_v) / v_max;
    } else {
      Tj = std::cbrt(dist / (2.0 * j_lim_));
      Ta = 2.0 * Tj;
      Tv = 0.0;
    }
  }

  const double T2 = std::max(0.0, Ta - 2.0 * Tj);
  dt_[0] = Tj;
  dt_[1] = T2;
  dt_[2] = Tj;
  dt_[3] = std::max(0.0, Tv);
  dt_[4] = Tj;
  dt_[5] = T2;
  dt_[6] = Tj;

  j_seg_ = {{j_lim_, 0.0, -j_lim_, 0.0, -j_lim_, 0.0, j_lim_}};

  t_cum_[0] = 0.0;
  p_node_[0] = 0.0;
  v_node_[0] = 0.0;
  a_node_[0] = 0.0;

  for (size_t k = 0; k < 7; ++k) {
    const double h = dt_[k];
    const double j = j_seg_[k];
    t_cum_[k + 1] = t_cum_[k] + h;
    p_node_[k + 1] =
      p_node_[k] + v_node_[k] * h + 0.5 * a_node_[k] * h * h + (1.0 / 6.0) * j * h * h * h;
    v_node_[k + 1] = v_node_[k] + a_node_[k] * h + 0.5 * j * h * h;
    a_node_[k + 1] = a_node_[k] + j * h;
  }
  total_time_ = t_cum_[7];
}

void SingleAxisSCurve::eval(
  double t, double & pos, double & vel, double & acc, double & jerk) const
{
  if (total_time_ <= 1e-11 || t <= 0.0) {
    pos = p0_;
    vel = 0.0;
    acc = 0.0;
    jerk = 0.0;
    return;
  }
  if (t >= total_time_) {
    pos = p1_;
    vel = 0.0;
    acc = 0.0;
    jerk = 0.0;
    return;
  }

  size_t seg = 6;
  for (size_t k = 0; k < 7; ++k) {
    if (t <= t_cum_[k + 1]) {
      seg = k;
      break;
    }
  }
  const double tau = t - t_cum_[seg];
  const double j = j_seg_[seg];
  const double p_rel =
    p_node_[seg] + v_node_[seg] * tau + 0.5 * a_node_[seg] * tau * tau +
    (1.0 / 6.0) * j * tau * tau * tau;
  const double v_rel = v_node_[seg] + a_node_[seg] * tau + 0.5 * j * tau * tau;
  const double a_rel = a_node_[seg] + j * tau;

  pos = p0_ + sign_ * p_rel;
  vel = sign_ * v_rel;
  acc = sign_ * a_rel;
  jerk = sign_ * j;
}

void MultiAxisSCurve::build(
  const std::array<double, 6> & q0,
  const std::array<double, 6> & q1,
  const std::array<lite6_kinematics::JointLimit, 6> & limits,
  double vel_scale,
  double acc_scale,
  double jerk_scale)
{
  // Safety margin inside the physical limits so 1 kHz finite differences never
  // exceed exact bounds due to floating-point rounding.
  constexpr double kSafety = 0.992;
  q0_ = q0;
  double v_n = std::numeric_limits<double>::infinity();
  double a_n = std::numeric_limits<double>::infinity();
  double j_n = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < 6; ++i) {
    delta_[i] = q1[i] - q0[i];
    const double d = std::abs(delta_[i]);
    if (d < 1e-11) {
      continue;
    }
    // Joint i moves |delta_i| * s(t): map its limits onto the normalized stroke.
    v_n = std::min(v_n, limits[i].max_vel * vel_scale * kSafety / d);
    a_n = std::min(a_n, limits[i].max_acc * acc_scale * kSafety / d);
    j_n = std::min(j_n, limits[i].max_jerk * jerk_scale * kSafety / d);
  }

  if (!std::isfinite(v_n)) {
    delta_.fill(0.0);
    profile_.build(0.0, 1.0, 1.0, 1.0, 1.0);
    sync_duration_ = 0.001;
    return;
  }

  profile_.build(0.0, 1.0, v_n, a_n, j_n);
  const double t_min = profile_.duration();
  // Round up to an integer multiple of 1 ms so the endpoint lands on the control grid,
  // using exact time dilation (V, A, J) -> (eta V, eta^2 A, eta^3 J).
  sync_duration_ = std::max(0.001, std::ceil(t_min * 1000.0 - 1e-9) * 0.001);
  if (t_min > 1e-11) {
    const double eta = std::clamp(t_min / sync_duration_, 1e-6, 1.0);
    profile_.build(0.0, 1.0, v_n * eta, a_n * eta * eta, j_n * eta * eta * eta);
  }
}

double MultiAxisSCurve::progress(double t) const
{
  if (t >= sync_duration_) {
    return 1.0;
  }
  double s = 0.0, sd = 0.0, sdd = 0.0, sddd = 0.0;
  profile_.eval(t, s, sd, sdd, sddd);
  return std::clamp(s, 0.0, 1.0);
}

void MultiAxisSCurve::eval(
  double t,
  std::array<double, 6> & q,
  std::array<double, 6> & dq,
  std::array<double, 6> & ddq,
  std::array<double, 6> & jrk) const
{
  double s = 0.0, sd = 0.0, sdd = 0.0, sddd = 0.0;
  if (t >= sync_duration_) {
    s = 1.0;
  } else {
    profile_.eval(t, s, sd, sdd, sddd);
  }
  for (size_t i = 0; i < 6; ++i) {
    q[i] = q0_[i] + delta_[i] * s;
    dq[i] = delta_[i] * sd;
    ddq[i] = delta_[i] * sdd;
    jrk[i] = delta_[i] * sddd;
  }
}

}  // namespace lite6_studio_planner
