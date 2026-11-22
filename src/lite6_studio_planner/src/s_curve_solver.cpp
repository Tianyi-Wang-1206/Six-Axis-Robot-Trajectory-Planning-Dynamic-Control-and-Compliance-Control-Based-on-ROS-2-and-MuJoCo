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
  const std::array<lite6_planner::JointLimit, 6> & limits,
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

void SepticHermiteSegment::build_with_duration(
  const std::array<double, 6> & q0,
  const std::array<double, 6> & v0,
  const std::array<double, 6> & a0,
  const std::array<double, 6> & q1,
  const std::array<double, 6> & v1,
  const std::array<double, 6> & a1,
  double duration)
{
  duration_ = std::max(0.002, std::ceil(duration * 1000.0 - 1e-9) * 0.001);
  const double T = duration_;
  const double T2 = T * T;

  for (size_t i = 0; i < 6; ++i) {
    const double c0 = q0[i];
    const double c1 = v0[i] * T;
    const double c2 = 0.5 * a0[i] * T2;
    const double c3 = 0.0;  // Zero initial jerk for C^3 smoothness

    const double d0 = q1[i] - (c0 + c1 + c2);
    const double d1 = v1[i] * T - (c1 + 2.0 * c2);
    const double d2 = a1[i] * T2 - 2.0 * c2;
    const double d3 = 0.0;  // Zero terminal jerk for C^3 smoothness

    // Solve 4x4 system for septic coefficients c4, c5, c6, c7 on tau in [0, 1]:
    const double c4 = 35.0 * d0 - 15.0 * d1 + 2.5 * d2 - (1.0 / 6.0) * d3;
    const double c5 = -84.0 * d0 + 39.0 * d1 - 7.0 * d2 + 0.5 * d3;
    const double c6 = 70.0 * d0 - 34.0 * d1 + 6.5 * d2 - 0.5 * d3;
    const double c7 = -20.0 * d0 + 10.0 * d1 - 2.0 * d2 + (1.0 / 6.0) * d3;

    coeffs_[i] = {{c0, c1, c2, c3, c4, c5, c6, c7}};
  }
}

bool SepticHermiteSegment::build_time_optimal(
  const std::array<double, 6> & q0,
  const std::array<double, 6> & v0,
  const std::array<double, 6> & a0,
  const std::array<double, 6> & q1,
  const std::array<double, 6> & v1,
  const std::array<double, 6> & a1,
  const std::array<lite6_planner::JointLimit, 6> & limits,
  double vel_scale,
  double acc_scale,
  double jerk_scale,
  double max_duration_sec)
{
  constexpr double kSafety = 0.985;
  constexpr double kEndpointTol = 1e-6;
  const double T_max = std::max(0.002, max_duration_sec);

  std::array<double, 6> v_lim{}, a_lim{}, j_lim{};
  std::array<double, 6> v_check{}, a_check{}, j_check{};
  double t_min = 0.002;
  bool all_rest = true;
  bool zero_motion = true;

  for (size_t i = 0; i < 6; ++i) {
    v_lim[i] = limits[i].max_vel * vel_scale;
    a_lim[i] = limits[i].max_acc * acc_scale;
    j_lim[i] = limits[i].max_jerk * jerk_scale;
    if (!(v_lim[i] > 0.0) || !(a_lim[i] > 0.0) || !(j_lim[i] > 0.0)) {
      return false;
    }
    if (!std::isfinite(q0[i]) || !std::isfinite(v0[i]) || !std::isfinite(a0[i]) ||
        !std::isfinite(q1[i]) || !std::isfinite(v1[i]) || !std::isfinite(a1[i]))
    {
      return false;
    }
    // Check boundary conditions against scaled limits up front.
    if (q0[i] < limits[i].min_pos - kEndpointTol || q0[i] > limits[i].max_pos + kEndpointTol ||
        q1[i] < limits[i].min_pos - kEndpointTol || q1[i] > limits[i].max_pos + kEndpointTol ||
        std::abs(v0[i]) > v_lim[i] * (1.0 + 1e-6) || std::abs(v1[i]) > v_lim[i] * (1.0 + 1e-6) ||
        std::abs(a0[i]) > a_lim[i] * (1.0 + 1e-6) || std::abs(a1[i]) > a_lim[i] * (1.0 + 1e-6))
    {
      return false;
    }
    // Allow boundary values that lie within (kSafety, 1.0] of the scaled limit so valid
    // boundary velocities/accelerations near the limit are never spuriously rejected.
    v_check[i] = std::max(v_lim[i] * kSafety, std::min(v_lim[i], std::max(std::abs(v0[i]), std::abs(v1[i]))));
    a_check[i] = std::max(a_lim[i] * kSafety, std::min(a_lim[i], std::max(std::abs(a0[i]), std::abs(a1[i]))));
    j_check[i] = j_lim[i] * kSafety;

    const double dq = std::abs(q1[i] - q0[i]);
    if (dq > 1e-11 || std::abs(v0[i]) > 1e-9 || std::abs(v1[i]) > 1e-9 ||
        std::abs(a0[i]) > 1e-9 || std::abs(a1[i]) > 1e-9)
    {
      zero_motion = false;
    }
    if (std::abs(v0[i]) > 1e-9 || std::abs(v1[i]) > 1e-9 ||
        std::abs(a0[i]) > 1e-9 || std::abs(a1[i]) > 1e-9)
    {
      all_rest = false;
    }
    // Universal kinematic lower bounds valid for arbitrary (v0, v1, a0, a1).
    t_min = std::max({
      t_min,
      dq / v_lim[i],
      std::abs(v1[i] - v0[i]) / a_lim[i],
      std::abs(a1[i] - a0[i]) / j_lim[i]});
  }

  if (zero_motion) {
    build_with_duration(q0, v0, a0, q1, v1, a1, 0.002);
    return true;
  }

  if (all_rest) {
    // Rest-to-rest acceleration and jerk lower bounds are valid only when v0=v1=a0=a1=0.
    for (size_t i = 0; i < 6; ++i) {
      const double dq = std::abs(q1[i] - q0[i]);
      t_min = std::max({
        t_min,
        std::sqrt(dq / a_lim[i]),
        std::cbrt(dq / j_lim[i])});
    }
  }

  if (t_min > T_max) {
    return false;
  }

  // Evaluates the worst-case constraint ratio (> 1.0 means infeasible) at continuous duration T.
  auto eval_ratio = [&](double T_cand) -> double {
    const double T = std::max(0.002, T_cand);
    const double T2 = T * T;
    const double invT = 1.0 / T;
    const double invT2 = invT * invT;
    const double invT3 = invT2 * invT;

    std::array<std::array<double, 8>, 6> c_all{};
    for (size_t i = 0; i < 6; ++i) {
      const double c0 = q0[i];
      const double c1 = v0[i] * T;
      const double c2 = 0.5 * a0[i] * T2;
      const double d0 = q1[i] - (c0 + c1 + c2);
      const double d1 = v1[i] * T - (c1 + 2.0 * c2);
      const double d2 = a1[i] * T2 - 2.0 * c2;
      c_all[i] = {{
        c0, c1, c2, 0.0,
        35.0 * d0 - 15.0 * d1 + 2.5 * d2,
        -84.0 * d0 + 39.0 * d1 - 7.0 * d2,
        70.0 * d0 - 34.0 * d1 + 6.5 * d2,
        -20.0 * d0 + 10.0 * d1 - 2.0 * d2}};
    }

    constexpr int kCheckSteps = 64;
    double worst = 0.0;
    for (int k = 0; k <= kCheckSteps; ++k) {
      const double tau = static_cast<double>(k) / kCheckSteps;
      const double t2 = tau * tau;
      const double t3 = t2 * tau;
      const double t4 = t3 * tau;
      const double t5 = t4 * tau;
      const double t6 = t5 * tau;
      const double t7 = t6 * tau;
      for (size_t i = 0; i < 6; ++i) {
        const auto & c = c_all[i];
        const double qi =
          c[0] + c[1] * tau + c[2] * t2 + c[4] * t4 + c[5] * t5 + c[6] * t6 + c[7] * t7;
        const double dqi =
          (c[1] + 2.0 * c[2] * tau + 4.0 * c[4] * t3 +
           5.0 * c[5] * t4 + 6.0 * c[6] * t5 + 7.0 * c[7] * t6) * invT;
        const double ddqi =
          (2.0 * c[2] + 12.0 * c[4] * t2 +
           20.0 * c[5] * t3 + 30.0 * c[6] * t4 + 42.0 * c[7] * t5) * invT2;
        const double jrki =
          (24.0 * c[4] * tau + 60.0 * c[5] * t2 +
           120.0 * c[6] * t3 + 210.0 * c[7] * t4) * invT3;
        if (!std::isfinite(qi) || !std::isfinite(dqi) ||
            !std::isfinite(ddqi) || !std::isfinite(jrki))
        {
          return std::numeric_limits<double>::infinity();
        }
        const double pos_exc = std::max(
          {0.0, limits[i].min_pos - qi, qi - limits[i].max_pos});
        if (pos_exc > kEndpointTol) {
          worst = std::max(worst, 1.0 + 100.0 * pos_exc);
        }
        worst = std::max({
          worst,
          std::abs(dqi) / v_check[i],
          std::abs(ddqi) / a_check[i],
          std::abs(jrki) / j_check[i]});
      }
    }
    return worst;
  };

  // Strict 1 kHz verification of a candidate integer-millisecond duration (including discrete jerk).
  auto verify_1khz = [&](double T_ms) -> bool {
    if (T_ms > T_max + 1e-9) {
      return false;
    }
    build_with_duration(q0, v0, a0, q1, v1, a1, T_ms);
    const int ticks = std::max(2, static_cast<int>(std::llround(duration_ * 1000.0)));
    std::array<double, 6> prev_ddq{};
    for (int tick = 0; tick <= ticks; ++tick) {
      const double t = static_cast<double>(tick) * 0.001;
      std::array<double, 6> q{}, dq{}, ddq{}, jrk{};
      eval(t, q, dq, ddq, jrk);
      for (size_t i = 0; i < 6; ++i) {
        if (q[i] < limits[i].min_pos - kEndpointTol || q[i] > limits[i].max_pos + kEndpointTol) {
          return false;
        }
        if (std::abs(dq[i]) > v_lim[i] * 0.999 + 1e-9 ||
            std::abs(ddq[i]) > a_lim[i] * 0.999 + 1e-9 ||
            std::abs(jrk[i]) > j_lim[i] * 0.999 + 1e-9)
        {
          return false;
        }
        if (tick > 0 && std::abs(ddq[i] - prev_ddq[i]) / 0.001 > j_lim[i] * 0.999 + 1e-6) {
          return false;
        }
      }
      prev_ddq = ddq;
    }
    return true;
  };

  // Build sorted scan candidates: fine logarithmic grid plus kinematic boundary seeds.
  std::vector<double> cands;
  cands.reserve(600);
  for (double t = t_min; t < T_max; t *= 1.02) {
    cands.push_back(t);
  }
  cands.push_back(T_max);
  for (size_t i = 0; i < 6; ++i) {
    const double dq = q1[i] - q0[i];
    const double v_avg = 0.5 * (v0[i] + v1[i]);
    if (v_avg * dq > 1e-12) {
      const double ts = dq / v_avg;
      if (ts >= t_min && ts <= T_max) {
        cands.push_back(ts);
      }
    }
    if (v0[i] * dq > 1e-12) {
      const double ts = dq / v0[i];
      if (ts >= t_min && ts <= T_max) {
        cands.push_back(ts);
      }
    }
    if (v1[i] * dq > 1e-12) {
      const double ts = dq / v1[i];
      if (ts >= t_min && ts <= T_max) {
        cands.push_back(ts);
      }
    }
  }
  std::sort(cands.begin(), cands.end());
  cands.erase(
    std::unique(cands.begin(), cands.end(), [](double a, double b) {
      return std::abs(a - b) <= 1e-6 * std::max(1.0, b);
    }),
    cands.end());

  auto refine_and_accept = [&](double lo, double hi) -> bool {
    for (int iter = 0; iter < 26; ++iter) {
      const double mid = 0.5 * (lo + hi);
      if (eval_ratio(mid) <= 1.0) {
        hi = mid;
      } else {
        lo = mid;
      }
    }
    const double t_ms = std::max(0.002, std::ceil(hi * 1000.0 - 1e-9) * 0.001);
    if (verify_1khz(t_ms)) {
      return true;
    }
    // Check immediate 1 ms neighbors in case a narrow window ends right at ceil(hi).
    if (t_ms >= 0.003 && verify_1khz(t_ms - 0.001)) {
      return true;
    }
    for (int step = 1; step <= 4; ++step) {
      if (verify_1khz(t_ms + step * 0.001)) {
        return true;
      }
    }
    return false;
  };

  std::vector<double> ratios(cands.size(), std::numeric_limits<double>::infinity());
  for (size_t idx = 0; idx < cands.size(); ++idx) {
    ratios[idx] = eval_ratio(cands[idx]);
    if (ratios[idx] <= 1.0) {
      const double lo = (idx > 0) ? cands[idx - 1] : t_min;
      if (refine_and_accept(lo, cands[idx])) {
        return true;
      }
    }
    // Probe narrow local valleys that may dip below 1.0 between adjacent grid points.
    if (idx >= 2 && ratios[idx - 1] < ratios[idx - 2] && ratios[idx - 1] < ratios[idx]) {
      double a = cands[idx - 2];
      double b = cands[idx];
      constexpr double kPhi = 0.6180339887498949;
      for (int gs = 0; gs < 16; ++gs) {
        const double m1 = b - kPhi * (b - a);
        const double m2 = a + kPhi * (b - a);
        const double r1 = eval_ratio(m1);
        const double r2 = eval_ratio(m2);
        if (r1 <= 1.0) {
          if (refine_and_accept(a, m1)) {
            return true;
          }
          break;
        }
        if (r2 <= 1.0) {
          if (refine_and_accept(m1, m2)) {
            return true;
          }
          break;
        }
        if (r1 < r2) {
          b = m2;
        } else {
          a = m1;
        }
      }
    }
  }
  return false;
}

void SepticHermiteSegment::eval(
  double t,
  std::array<double, 6> & q,
  std::array<double, 6> & dq,
  std::array<double, 6> & ddq,
  std::array<double, 6> & jrk) const
{
  const double T = duration_;
  const double tau = std::clamp(t / T, 0.0, 1.0);
  const double t2 = tau * tau;
  const double t3 = t2 * tau;
  const double t4 = t3 * tau;
  const double t5 = t4 * tau;
  const double t6 = t5 * tau;
  const double t7 = t6 * tau;

  const double invT = 1.0 / T;
  const double invT2 = invT * invT;
  const double invT3 = invT2 * invT;

  for (size_t i = 0; i < 6; ++i) {
    const auto & c = coeffs_[i];
    q[i] = c[0] + c[1] * tau + c[2] * t2 + c[3] * t3 + c[4] * t4 + c[5] * t5 + c[6] * t6 + c[7] * t7;
    dq[i] = (c[1] + 2.0 * c[2] * tau + 3.0 * c[3] * t2 + 4.0 * c[4] * t3 +
             5.0 * c[5] * t4 + 6.0 * c[6] * t5 + 7.0 * c[7] * t6) * invT;
    ddq[i] = (2.0 * c[2] + 6.0 * c[3] * tau + 12.0 * c[4] * t2 +
              20.0 * c[5] * t3 + 30.0 * c[6] * t4 + 42.0 * c[7] * t5) * invT2;
    jrk[i] = (6.0 * c[3] + 24.0 * c[4] * tau + 60.0 * c[5] * t2 +
              120.0 * c[6] * t3 + 210.0 * c[7] * t4) * invT3;
  }
}

}  // namespace lite6_studio_planner
