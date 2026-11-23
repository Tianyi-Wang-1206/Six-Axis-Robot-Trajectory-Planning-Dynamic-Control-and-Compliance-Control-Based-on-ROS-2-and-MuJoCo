#include "lite6_studio_planner/joint_pva_solver.hpp"
#include "lite6_studio_planner/s_curve_solver.hpp"
#include "lite6_planner/motion_tolerances.hpp"

#include <ruckig/ruckig.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <tuple>
#include <utility>
#include <vector>

namespace lite6_studio_planner
{
namespace
{

constexpr double kDt = 0.001;
constexpr double kHeadroom = 0.996;
constexpr double kPosTol = 1e-6;
constexpr double kTimeMatchTol = 1e-6;
constexpr double kMaxSegmentDurationSec = 120.0;
constexpr double kMaxTotalDurationSec = 300.0;
constexpr int kMaxSplitDepth = 3;

struct DoFLimit
{
  double q_min = 0.0;
  double q_max = 0.0;
  double v_lim = 0.0;
  double a_lim = 0.0;
  double j_lim = 0.0;
};

struct ProfileExtrema
{
  double min = std::numeric_limits<double>::infinity();
  double max = -std::numeric_limits<double>::infinity();
  double t_min = 0.0;
  double t_max = 0.0;
};

// Exact analytical position extrema of a 1-DOF Ruckig Profile, including constant-acceleration
// segments (j == 0, a != 0) where velocity can cross zero.
void check_step_extrema(
  double t_offset, double t_dur, double p0, double v0, double a0, double j0,
  ProfileExtrema & ext)
{
  auto record = [&](double tau, double val) {
    if (val < ext.min) {
      ext.min = val;
      ext.t_min = t_offset + tau;
    }
    if (val > ext.max) {
      ext.max = val;
      ext.t_max = t_offset + tau;
    }
  };
  auto eval_p = [&](double tau) {
    return p0 + tau * (v0 + tau * (0.5 * a0 + tau * (j0 / 6.0)));
  };

  record(0.0, p0);
  if (t_dur <= 0.0) {
    return;
  }
  record(t_dur, eval_p(t_dur));

  if (std::abs(j0) > 1e-14) {
    const double D = a0 * a0 - 2.0 * j0 * v0;
    if (std::abs(D) <= 1e-16) {
      const double tau = -a0 / j0;
      if (tau > 0.0 && tau < t_dur) {
        record(tau, eval_p(tau));
      }
    } else if (D > 0.0) {
      const double sD = std::sqrt(D);
      const double r1 = (-a0 - sD) / j0;
      const double r2 = (-a0 + sD) / j0;
      if (r1 > 0.0 && r1 < t_dur) {
        record(r1, eval_p(r1));
      }
      if (r2 > 0.0 && r2 < t_dur) {
        record(r2, eval_p(r2));
      }
    }
  } else if (std::abs(a0) > 1e-14) {
    const double tau = -v0 / a0;
    if (tau > 0.0 && tau < t_dur) {
      record(tau, eval_p(tau));
    }
  }
}

ProfileExtrema compute_profile_extrema(const ruckig::Profile & prof)
{
  ProfileExtrema ext;
  double t_off = 0.0;
  if (prof.brake.duration > 0.0) {
    if (prof.brake.t[0] > 0.0) {
      check_step_extrema(
        t_off, prof.brake.t[0], prof.brake.p[0], prof.brake.v[0],
        prof.brake.a[0], prof.brake.j[0], ext);
      t_off += prof.brake.t[0];
      if (prof.brake.t[1] > 0.0) {
        check_step_extrema(
          t_off, prof.brake.t[1], prof.brake.p[1], prof.brake.v[1],
          prof.brake.a[1], prof.brake.j[1], ext);
        t_off += prof.brake.t[1];
      }
    }
  }
  for (size_t i = 0; i < 7; ++i) {
    const double t_step_off = prof.brake.duration + (i > 0 ? prof.t_sum[i - 1] : 0.0);
    check_step_extrema(
      t_step_off, prof.t[i], prof.p[i], prof.v[i], prof.a[i], prof.j[i], ext);
  }
  const double t_end = prof.brake.duration + prof.t_sum.back();
  if (prof.pf < ext.min) {
    ext.min = prof.pf;
    ext.t_min = t_end;
  }
  if (prof.pf > ext.max) {
    ext.max = prof.pf;
    ext.t_max = t_end;
  }
  return ext;
}

void eval_ruckig_profile(
  const ruckig::Profile & prof, double t,
  double & q, double & v, double & a, double & j)
{
  const double dur = prof.brake.duration + prof.t_sum.back();
  if (t <= 0.0) {
    if (prof.brake.duration > 0.0 && prof.brake.t[0] > 0.0) {
      q = prof.brake.p[0];
      v = prof.brake.v[0];
      a = prof.brake.a[0];
      j = prof.brake.j[0];
    } else {
      q = prof.p[0];
      v = prof.v[0];
      a = prof.a[0];
      j = prof.j[0];
    }
    return;
  }
  if (t >= dur) {
    q = prof.pf;
    v = prof.vf;
    a = prof.af;
    j = 0.0;
    return;
  }
  double tau = t;
  if (prof.brake.duration > 0.0) {
    if (tau < prof.brake.duration) {
      const size_t idx = (tau < prof.brake.t[0]) ? 0 : 1;
      if (idx > 0) {
        tau -= prof.brake.t[0];
      }
      j = prof.brake.j[idx];
      std::tie(q, v, a) = ruckig::integrate(
        tau, prof.brake.p[idx], prof.brake.v[idx], prof.brake.a[idx], j);
      return;
    }
    tau -= prof.brake.duration;
  }
  if (tau >= prof.t_sum.back()) {
    q = prof.pf;
    v = prof.vf;
    a = prof.af;
    j = 0.0;
    return;
  }
  const auto it = std::upper_bound(prof.t_sum.begin(), prof.t_sum.end(), tau);
  const size_t idx = std::min<size_t>(
    6, static_cast<size_t>(std::distance(prof.t_sum.begin(), it)));
  if (idx > 0) {
    tau -= prof.t_sum[idx - 1];
  }
  j = prof.j[idx];
  std::tie(q, v, a) = ruckig::integrate(tau, prof.p[idx], prof.v[idx], prof.a[idx], j);
}

// Minimum upper position peak reachable from (q0, v0, a0) under maximum downward braking
// (j = -j_max down to a = -a_max). Any (a, j)-limited trajectory from (q0, v0, a0) must
// attain at least this upper position.
double min_upper_excursion(double q0, double v0, double a0, double a_max, double j_max)
{
  if (v0 <= 0.0 && a0 <= 0.0) {
    return q0;
  }
  double q = q0;
  double v = v0;
  double a = a0;

  if (a > 0.0) {
    const double t0 = a / j_max;
    const double v_apex = v + 0.5 * a * t0;
    if (v_apex <= 0.0) {
      return q0;
    }
    q += v * t0 + (a * a * a) / (3.0 * j_max * j_max);
    v = v_apex;
    a = 0.0;
  }

  // Now a <= 0 and v > 0. Ramp a down toward -a_max at j = -j_max.
  const double t1 = std::max(0.0, (a + a_max) / j_max);
  const double v1 = v + a * t1 - 0.5 * j_max * t1 * t1;
  if (v1 <= 0.0) {
    const double disc = std::max(0.0, a * a + 2.0 * j_max * v);
    const double t_star = std::clamp((a + std::sqrt(disc)) / j_max, 0.0, t1);
    return q + v * t_star + 0.5 * a * t_star * t_star -
      (1.0 / 6.0) * j_max * t_star * t_star * t_star;
  }
  const double q1 = q + v * t1 + 0.5 * a * t1 * t1 - (1.0 / 6.0) * j_max * t1 * t1 * t1;
  return q1 + (v1 * v1) / (2.0 * a_max);
}

double max_lower_excursion(double q0, double v0, double a0, double a_max, double j_max)
{
  return -min_upper_excursion(-q0, -v0, -a0, a_max, j_max);
}

// Maximum |v| with a = 0 whose stopping excursion to (0, 0) under (a_max, j_max)
// stays within distance d >= 0.
double max_speed_for_distance(double d, double a_max, double j_max)
{
  if (d <= 0.0) {
    return 0.0;
  }
  const double d_crit = (a_max * a_max * a_max) / (j_max * j_max);
  if (d <= d_crit) {
    return std::cbrt(d * d * j_max);
  }
  const double half_b = 0.5 * (a_max * a_max) / j_max;
  return std::max(0.0, -half_b + std::sqrt(half_b * half_b + 2.0 * a_max * d));
}

// Checks whether a 1-DOF state (q, v, a) is valid and velocity-reachable/braking-feasible
// in both forward and reverse time inside [q_min, q_max] and scaled (v_lim, a_lim, j_lim).
bool validate_dof_state(
  double q, double v, double a, const DoFLimit & lim,
  bool check_forward, bool check_reverse,
  std::string & out_code, std::string & out_reason)
{
  if (!std::isfinite(q) || !std::isfinite(v) || !std::isfinite(a)) {
    out_code = "ERR_NONFINITE_SAMPLE";
    out_reason = "non-finite waypoint state";
    return false;
  }
  if (q < lim.q_min - kPosTol || q > lim.q_max + kPosTol) {
    out_code = "ERR_JOINT_POS_LIMIT";
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(5)
        << "position=" << q << " rad outside [" << lim.q_min << ", " << lim.q_max << "] rad";
    out_reason = oss.str();
    return false;
  }
  if (std::abs(v) > lim.v_lim * (1.0 + 1e-6)) {
    out_code = "ERR_PVA_VEL_LIMIT";
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(5)
        << "|v|=" << std::abs(v) << " rad/s exceeds scaled limit=" << lim.v_lim << " rad/s";
    out_reason = oss.str();
    return false;
  }
  if (std::abs(a) > lim.a_lim * (1.0 + 1e-6)) {
    out_code = "ERR_PVA_ACC_LIMIT";
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(5)
        << "|a|=" << std::abs(a) << " rad/s^2 exceeds scaled limit=" << lim.a_lim << " rad/s^2";
    out_reason = oss.str();
    return false;
  }
  // Velocity-limit reachability under jerk bound:
  // Forward: a > 0 => v + a^2/(2*j) <= v_lim; a < 0 => v - a^2/(2*j) >= -v_lim.
  if (check_forward) {
    if ((a > 0.0 && v + (a * a) / (2.0 * lim.j_lim) > lim.v_lim * (1.0 + 1e-5)) ||
        (a < 0.0 && v - (a * a) / (2.0 * lim.j_lim) < -lim.v_lim * (1.0 + 1e-5)))
    {
      out_code = "ERR_PVA_VEL_LIMIT";
      out_reason = "waypoint (v, a) unavoidably overshoots scaled velocity limit under jerk bound";
      return false;
    }
    const double q_hi = min_upper_excursion(q, v, a, lim.a_lim, lim.j_lim);
    const double q_lo = max_lower_excursion(q, v, a, lim.a_lim, lim.j_lim);
    if (q_hi > lim.q_max + kPosTol || q_lo < lim.q_min - kPosTol) {
      out_code = "ERR_JOINT_POS_LIMIT";
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(5)
          << "forward braking excursion [" << q_lo << ", " << q_hi
          << "] rad exceeds joint limits [" << lim.q_min << ", " << lim.q_max << "] rad";
      out_reason = oss.str();
      return false;
    }
  }
  // Reverse-time: (q, -v, a).
  if (check_reverse) {
    if ((a < 0.0 && v + (a * a) / (2.0 * lim.j_lim) > lim.v_lim * (1.0 + 1e-5)) ||
        (a > 0.0 && v - (a * a) / (2.0 * lim.j_lim) < -lim.v_lim * (1.0 + 1e-5)))
    {
      out_code = "ERR_PVA_VEL_LIMIT";
      out_reason = "waypoint (v, a) cannot be approached without exceeding scaled velocity limit";
      return false;
    }
    const double q_hi_rev = min_upper_excursion(q, -v, a, lim.a_lim, lim.j_lim);
    const double q_lo_rev = max_lower_excursion(q, -v, a, lim.a_lim, lim.j_lim);
    if (q_hi_rev > lim.q_max + kPosTol || q_lo_rev < lim.q_min - kPosTol) {
      out_code = "ERR_JOINT_POS_LIMIT";
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(5)
          << "reverse-time braking excursion [" << q_lo_rev << ", " << q_hi_rev
          << "] rad exceeds joint limits [" << lim.q_min << ", " << lim.q_max << "] rad";
      out_reason = oss.str();
      return false;
    }
  }
  return true;
}

// Compute working Ruckig limits (v_cap, a_cap, j_cap) for a 1-DOF segment so interior motion
// retains kHeadroom (0.996) margin while allowing valid boundary states up to 100% of limits.
void compute_working_caps(
  double v0, double a0, double v1, double a1, const DoFLimit & lim,
  double & v_cap, double & a_cap, double & j_cap)
{
  j_cap = lim.j_lim * kHeadroom;
  a_cap = std::min(
    lim.a_lim,
    std::max({lim.a_lim * kHeadroom, std::abs(a0) + 1e-9, std::abs(a1) + 1e-9}));

  // Ensure Ruckig's v_at_a_zero check passes for both initial and target states.
  auto peak_v = [](double v, double a, double j) {
    return std::abs(v) + (a * a) / (2.0 * j);
  };
  double v_req = std::max({lim.v_lim * kHeadroom, std::abs(v0) + 1e-9, std::abs(v1) + 1e-9});
  if (a0 > 0.0) {
    v_req = std::max(v_req, std::abs(v0 + (a0 * a0) / (2.0 * j_cap)) + 1e-9);
  } else if (a0 < 0.0) {
    v_req = std::max(v_req, std::abs(v0 - (a0 * a0) / (2.0 * j_cap)) + 1e-9);
  }
  if (a1 < 0.0) {
    v_req = std::max(v_req, std::abs(v1 + (a1 * a1) / (2.0 * j_cap)) + 1e-9);
  } else if (a1 > 0.0) {
    v_req = std::max(v_req, std::abs(v1 - (a1 * a1) / (2.0 * j_cap)) + 1e-9);
  }
  if (v_req > lim.v_lim) {
    // Relax jerk headroom up to hard j_lim if needed for boundary reachability.
    j_cap = lim.j_lim;
    v_req = std::max({lim.v_lim * kHeadroom, std::abs(v0) + 1e-9, std::abs(v1) + 1e-9});
    if (a0 > 0.0) {
      v_req = std::max(v_req, std::abs(v0 + (a0 * a0) / (2.0 * j_cap)) + 1e-9);
    } else if (a0 < 0.0) {
      v_req = std::max(v_req, std::abs(v0 - (a0 * a0) / (2.0 * j_cap)) + 1e-9);
    }
    if (a1 < 0.0) {
      v_req = std::max(v_req, std::abs(v1 + (a1 * a1) / (2.0 * j_cap)) + 1e-9);
    } else if (a1 > 0.0) {
      v_req = std::max(v_req, std::abs(v1 - (a1 * a1) / (2.0 * j_cap)) + 1e-9);
    }
    (void)peak_v;
  }
  v_cap = std::min(lim.v_lim * (1.0 + 1e-7), v_req);
}

enum class SubPartKind
{
  RUCKIG_PROFILE,
  DWELL,
  SEPTIC_POLY
};

struct DoFSubPart
{
  SubPartKind kind = SubPartKind::RUCKIG_PROFILE;
  double duration = 0.0;
  double dwell_q = 0.0;
  ruckig::Profile profile;
  std::array<double, 8> septic_c{};
};

struct DoFTrajectory
{
  std::vector<DoFSubPart> parts;
  double total_duration = 0.0;
  double min_pos = std::numeric_limits<double>::infinity();
  double max_pos = -std::numeric_limits<double>::infinity();

  void clear()
  {
    parts.clear();
    total_duration = 0.0;
    min_pos = std::numeric_limits<double>::infinity();
    max_pos = -std::numeric_limits<double>::infinity();
  }

  void append(const DoFSubPart & part, double part_min, double part_max)
  {
    parts.push_back(part);
    total_duration += part.duration;
    min_pos = std::min(min_pos, part_min);
    max_pos = std::max(max_pos, part_max);
  }

  void append_traj(const DoFTrajectory & other)
  {
    for (const auto & p : other.parts) {
      parts.push_back(p);
    }
    total_duration += other.total_duration;
    min_pos = std::min(min_pos, other.min_pos);
    max_pos = std::max(max_pos, other.max_pos);
  }

  void eval(double t, double & q, double & v, double & a, double & j) const
  {
    if (parts.empty()) {
      q = v = a = j = 0.0;
      return;
    }
    double tau = std::clamp(t, 0.0, total_duration);
    for (size_t i = 0; i < parts.size(); ++i) {
      const auto & part = parts[i];
      if (tau <= part.duration || i + 1 == parts.size()) {
        const double tl = std::clamp(tau, 0.0, part.duration);
        if (part.kind == SubPartKind::DWELL) {
          q = part.dwell_q;
          v = 0.0;
          a = 0.0;
          j = 0.0;
        } else if (part.kind == SubPartKind::RUCKIG_PROFILE) {
          eval_ruckig_profile(part.profile, tl, q, v, a, j);
        } else {
          const double T = std::max(1e-9, part.duration);
          const double s = std::clamp(tl / T, 0.0, 1.0);
          const double s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s, s6 = s5 * s, s7 = s6 * s;
          const double invT = 1.0 / T, invT2 = invT * invT, invT3 = invT2 * invT;
          const auto & c = part.septic_c;
          q = c[0] + c[1] * s + c[2] * s2 + c[3] * s3 + c[4] * s4 + c[5] * s5 + c[6] * s6 + c[7] * s7;
          v = (c[1] + 2.0 * c[2] * s + 3.0 * c[3] * s2 + 4.0 * c[4] * s3 +
               5.0 * c[5] * s4 + 6.0 * c[6] * s5 + 7.0 * c[7] * s6) * invT;
          a = (2.0 * c[2] + 6.0 * c[3] * s + 12.0 * c[4] * s2 +
               20.0 * c[5] * s3 + 30.0 * c[6] * s4 + 42.0 * c[7] * s5) * invT2;
          j = (6.0 * c[3] + 24.0 * c[4] * s + 60.0 * c[5] * s2 +
               120.0 * c[6] * s3 + 210.0 * c[7] * s4) * invT3;
        }
        return;
      }
      tau -= part.duration;
    }
  }
};

// Raw 1-DOF unconstrained Ruckig solve (continuous duration, optional minimum_duration).
bool solve_ruckig_1dof_raw(
  double q0, double v0, double a0,
  double q1, double v1, double a1,
  const DoFLimit & lim,
  std::optional<double> min_dur,
  ruckig::Profile & out_profile,
  double & out_duration,
  ProfileExtrema & out_ext)
{
  double v_cap = 0.0, a_cap = 0.0, j_cap = 0.0;
  compute_working_caps(v0, a0, v1, a1, lim, v_cap, a_cap, j_cap);

  ruckig::InputParameter<1> inp;
  inp.control_interface = ruckig::ControlInterface::Position;
  inp.synchronization = ruckig::Synchronization::Time;
  inp.duration_discretization = ruckig::DurationDiscretization::Continuous;
  inp.current_position[0] = q0;
  inp.current_velocity[0] = v0;
  inp.current_acceleration[0] = a0;
  inp.target_position[0] = q1;
  inp.target_velocity[0] = v1;
  inp.target_acceleration[0] = a1;
  inp.max_velocity[0] = v_cap;
  inp.max_acceleration[0] = a_cap;
  inp.max_jerk[0] = j_cap;
  if (min_dur.has_value() && *min_dur > 0.0) {
    inp.minimum_duration = *min_dur;
  }

  ruckig::Ruckig<1> otg;
  ruckig::Trajectory<1> traj;
  const auto res = otg.calculate(inp, traj);
  if (static_cast<int>(res) < 0) {
    return false;
  }
  const double dur = traj.get_duration();
  if (!std::isfinite(dur) || dur < 0.0 || dur > kMaxSegmentDurationSec) {
    return false;
  }
  out_profile = traj.get_profiles()[0][0];
  out_duration = dur;
  out_ext = compute_profile_extrema(out_profile);
  return true;
}

bool extrema_within(const ProfileExtrema & ext, const DoFLimit & lim)
{
  return ext.min >= lim.q_min - kPosTol && ext.max <= lim.q_max + kPosTol;
}

// Builds a single-DoF septic Hermite polynomial part at exact duration T and verifies
// all scaled position/velocity/acceleration/continuous+discrete jerk bounds at 1 kHz.
bool build_verified_septic_1dof(
  double q0, double v0, double a0,
  double q1, double v1, double a1,
  const DoFLimit & lim, double T,
  DoFSubPart & out_part, double & out_min, double & out_max)
{
  if (!(T > 1e-4) || T > kMaxSegmentDurationSec) {
    return false;
  }
  const double T2 = T * T;
  const double c0 = q0;
  const double c1 = v0 * T;
  const double c2 = 0.5 * a0 * T2;
  const double d0 = q1 - (c0 + c1 + c2);
  const double d1 = v1 * T - (c1 + 2.0 * c2);
  const double d2 = a1 * T2 - 2.0 * c2;

  std::array<double, 8> c{{
    c0, c1, c2, 0.0,
    35.0 * d0 - 15.0 * d1 + 2.5 * d2,
    -84.0 * d0 + 39.0 * d1 - 7.0 * d2,
    70.0 * d0 - 34.0 * d1 + 6.5 * d2,
    -20.0 * d0 + 10.0 * d1 - 2.0 * d2}};

  const int ticks = std::max(2, static_cast<int>(std::llround(T * 1000.0)));
  const double invT = 1.0 / T, invT2 = invT * invT, invT3 = invT2 * invT;
  double prev_a = a0;
  out_min = std::min(q0, q1);
  out_max = std::max(q0, q1);

  for (int k = 0; k <= ticks; ++k) {
    const double s = std::clamp(static_cast<double>(k) * kDt / T, 0.0, 1.0);
    const double s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s, s6 = s5 * s, s7 = s6 * s;
    const double q = c[0] + c[1] * s + c[2] * s2 + c[4] * s4 + c[5] * s5 + c[6] * s6 + c[7] * s7;
    const double v = (c[1] + 2.0 * c[2] * s + 4.0 * c[4] * s3 +
                      5.0 * c[5] * s4 + 6.0 * c[6] * s5 + 7.0 * c[7] * s6) * invT;
    const double a = (2.0 * c[2] + 12.0 * c[4] * s2 +
                      20.0 * c[5] * s3 + 30.0 * c[6] * s4 + 42.0 * c[7] * s5) * invT2;
    const double j = (24.0 * c[4] * s + 60.0 * c[5] * s2 +
                      120.0 * c[6] * s3 + 210.0 * c[7] * s4) * invT3;
    if (!std::isfinite(q) || !std::isfinite(v) || !std::isfinite(a) || !std::isfinite(j)) {
      return false;
    }
    if (q < lim.q_min - kPosTol || q > lim.q_max + kPosTol ||
        std::abs(v) > lim.v_lim * 0.999 + 1e-6 ||
        std::abs(a) > lim.a_lim * 0.999 + 1e-5 ||
        std::abs(j) > lim.j_lim * 0.999 + 1e-3)
    {
      return false;
    }
    if (k > 0 && std::abs(a - prev_a) / kDt > lim.j_lim * 0.999 + 1e-3) {
      return false;
    }
    prev_a = a;
    out_min = std::min(out_min, q);
    out_max = std::max(out_max, q);
  }
  out_part.kind = SubPartKind::SEPTIC_POLY;
  out_part.duration = T;
  out_part.septic_c = c;
  return true;
}

// Layer A (1-DOF): Minimum-time state-constrained trajectory keeping q(t) in [q_min, q_max].
bool solve_dof_min_time(
  double q0, double v0, double a0,
  double q1, double v1, double a1,
  const DoFLimit & lim,
  DoFTrajectory & out_traj,
  bool & out_clamped,
  int depth = 0)
{
  out_traj.clear();
  out_clamped = false;

  if (std::abs(q1 - q0) < 1e-11 && std::abs(v0) < 1e-10 && std::abs(v1) < 1e-10 &&
      std::abs(a0) < 1e-10 && std::abs(a1) < 1e-10)
  {
    DoFSubPart dwell;
    dwell.kind = SubPartKind::DWELL;
    dwell.duration = 0.0;
    dwell.dwell_q = std::clamp(q0, lim.q_min, lim.q_max);
    out_traj.append(dwell, dwell.dwell_q, dwell.dwell_q);
    return true;
  }

  ruckig::Profile prof;
  double dur = 0.0;
  ProfileExtrema ext;
  if (!solve_ruckig_1dof_raw(q0, v0, a0, q1, v1, a1, lim, std::nullopt, prof, dur, ext)) {
    return false;
  }

  if (extrema_within(ext, lim)) {
    DoFSubPart part;
    part.kind = SubPartKind::RUCKIG_PROFILE;
    part.duration = dur;
    part.profile = prof;
    out_traj.append(part, ext.min, ext.max);
    return true;
  }

  if (depth >= kMaxSplitDepth) {
    return false;
  }

  // Pontryagin state-constrained minimum-time condition: meet violated boundary tangentially
  // at (q_lim, 0, 0). Choose whichever bound is violated first in time if both overshoot.
  const bool hi_vio = (ext.max > lim.q_max + kPosTol);
  const bool lo_vio = (ext.min < lim.q_min - kPosTol);
  double q_lim = lim.q_max;
  if (hi_vio && lo_vio) {
    q_lim = (ext.t_max <= ext.t_min) ? lim.q_max : lim.q_min;
  } else if (lo_vio) {
    q_lim = lim.q_min;
  }

  auto try_boundary_split = [&](double q_b, double a_b, DoFTrajectory & res_traj) -> bool {
    DoFTrajectory sub1, sub2;
    bool c1 = false, c2 = false;
    if (!solve_dof_min_time(q0, v0, a0, q_b, 0.0, a_b, lim, sub1, c1, depth + 1)) {
      return false;
    }
    if (!solve_dof_min_time(q_b, 0.0, a_b, q1, v1, a1, lim, sub2, c2, depth + 1)) {
      return false;
    }
    if (sub1.min_pos < lim.q_min - kPosTol || sub1.max_pos > lim.q_max + kPosTol ||
        sub2.min_pos < lim.q_min - kPosTol || sub2.max_pos > lim.q_max + kPosTol)
    {
      return false;
    }
    res_traj.clear();
    res_traj.append_traj(sub1);
    res_traj.append_traj(sub2);
    return true;
  };

  // Primary Pontryagin tangential contact: (q_lim, 0, 0).
  if (try_boundary_split(q_lim, 0.0, out_traj)) {
    out_clamped = true;
    return true;
  }

  // Rare edge case: braking to a = 0 overshoots q_lim even though a nonzero bounce
  // acceleration a_b (pointing back into the feasible interior) stays strictly inside q_lim.
  const double dir = (q_lim >= lim.q_max - 1e-9) ? -1.0 : 1.0;
  const double a_work = lim.a_lim * kHeadroom;
  for (double frac : {0.25, 0.5, 0.75, 0.95, 1.0}) {
    if (try_boundary_split(q_lim, dir * frac * a_work, out_traj)) {
      out_clamped = true;
      return true;
    }
  }
  return false;
}

// Forward declaration for recursive call.
bool solve_dof_at_duration(
  double q0, double v0, double a0,
  double q1, double v1, double a1,
  const DoFLimit & lim,
  double T_target,
  bool allow_blocked_split,
  DoFTrajectory & out_traj,
  double & out_next_unblocked_T,
  int depth = 0);

// Resolve a blocked time interval at exact T_target via a 2-section split through an
// intermediate state (q_m, v_m, 0) or verified septic polynomial fallback.
bool resolve_blocked_at_exact_duration(
  double q0, double v0, double a0,
  double q1, double v1, double a1,
  const DoFLimit & lim,
  double T_target,
  DoFTrajectory & out_traj,
  int depth)
{
  if (depth >= 2) {
    DoFSubPart sp;
    double pmin = 0.0, pmax = 0.0;
    if (build_verified_septic_1dof(q0, v0, a0, q1, v1, a1, lim, T_target, sp, pmin, pmax)) {
      out_traj.clear();
      out_traj.append(sp, pmin, pmax);
      return true;
    }
    return false;
  }

  // Strategy 1: 2-section split through an intermediate rest state (q_m, 0, 0).
  // Any (q_m, 0, 0) with T1_min + T2_min <= T_target admits an exact duration T_target
  // via either extended sub-segments or a dwell at (q_m, 0, 0) with zero blocked intervals.
  const double q_mid_raw = 0.5 * (q0 + q1);
  const double q_ext_hi = std::min(lim.q_max, std::max(q0, q1) + 0.15 * (lim.q_max - std::max(q0, q1)));
  const double q_ext_lo = std::max(lim.q_min, std::min(q0, q1) - 0.15 * (std::min(q0, q1) - lim.q_min));
  const std::array<double, 7> q_cands{{
    std::clamp(q_mid_raw, lim.q_min, lim.q_max),
    std::clamp(q0 + 0.25 * (q1 - q0), lim.q_min, lim.q_max),
    std::clamp(q0 + 0.75 * (q1 - q0), lim.q_min, lim.q_max),
    q_ext_hi, q_ext_lo,
    std::clamp(q0, lim.q_min, lim.q_max),
    std::clamp(q1, lim.q_min, lim.q_max)
  }};

  for (double qm : q_cands) {
    DoFTrajectory s1_min, s2_min;
    bool c1 = false, c2 = false;
    if (!solve_dof_min_time(q0, v0, a0, qm, 0.0, 0.0, lim, s1_min, c1, depth + 1) ||
        !solve_dof_min_time(qm, 0.0, 0.0, q1, v1, a1, lim, s2_min, c2, depth + 1))
    {
      continue;
    }
    const double t_sum = s1_min.total_duration + s2_min.total_duration;
    if (t_sum <= T_target + kTimeMatchTol) {
      const double slack = std::max(0.0, T_target - t_sum);
      // Prefer smooth extended sub-segments when both are unblocked and within position limits.
      if (slack > 1e-6) {
        const double frac = (t_sum > 1e-9) ? (s1_min.total_duration / t_sum) : 0.5;
        const double T1 = s1_min.total_duration + frac * slack;
        const double T2 = T_target - T1;
        DoFTrajectory e1, e2;
        double n1 = 0.0, n2 = 0.0;
        if (solve_dof_at_duration(q0, v0, a0, qm, 0.0, 0.0, lim, T1, false, e1, n1, depth + 1) &&
            solve_dof_at_duration(qm, 0.0, 0.0, q1, v1, a1, lim, T2, false, e2, n2, depth + 1))
        {
          out_traj.clear();
          out_traj.append_traj(e1);
          out_traj.append_traj(e2);
          return true;
        }
      }
      out_traj.clear();
      out_traj.append_traj(s1_min);
      if (slack > 1e-9) {
        DoFSubPart dwell;
        dwell.kind = SubPartKind::DWELL;
        dwell.duration = slack;
        dwell.dwell_q = qm;
        out_traj.append(dwell, qm, qm);
      }
      out_traj.append_traj(s2_min);
      return true;
    }
  }

  // Strategy 2: 2-section split with non-zero midpoint velocity (q_m, v_m, a_m) at t_m = 0.5 * T_target
  // trying both cubic-Hermite and symmetric-trapezoid midpoint estimates.
  const double v_herm = 1.5 * (q1 - q0) / T_target - 0.25 * (v0 + v1);
  const double v_sym = 2.0 * (q1 - q0) / T_target - 0.5 * (v0 + v1);
  const double q_herm = std::clamp(
    0.5 * (q0 + q1) + 0.125 * T_target * (v0 - v1), lim.q_min, lim.q_max);
  const double v_lim_safe = std::min(
    lim.v_lim * kHeadroom,
    max_speed_for_distance(
      0.98 * std::min(lim.q_max - q_herm, q_herm - lim.q_min),
      lim.a_lim * kHeadroom, lim.j_lim * kHeadroom));

  const double T_half = 0.5 * T_target;
  const std::array<double, 7> vm_cands{{
    v_sym,
    v_herm,
    0.75 * v_herm,
    0.5 * v_herm,
    0.25 * v_herm,
    -0.25 * v_herm,
    0.5 * (v0 + v1)
  }};
  for (double vm_raw : vm_cands) {
    const double vm = std::clamp(vm_raw, -v_lim_safe, v_lim_safe);
    DoFTrajectory e1, e2;
    double n1 = 0.0, n2 = 0.0;
    if (solve_dof_at_duration(q0, v0, a0, q_herm, vm, 0.0, lim, T_half, false, e1, n1, depth + 1) &&
        solve_dof_at_duration(q_herm, vm, 0.0, q1, v1, a1, lim, T_target - T_half, false, e2, n2, depth + 1))
    {
      out_traj.clear();
      out_traj.append_traj(e1);
      out_traj.append_traj(e2);
      return true;
    }
  }

  // Strategy 3: Verified single-DoF septic Hermite polynomial fallback at exact T_target.
  DoFSubPart sp;
  double pmin = 0.0, pmax = 0.0;
  if (build_verified_septic_1dof(q0, v0, a0, q1, v1, a1, lim, T_target, sp, pmin, pmax)) {
    out_traj.clear();
    out_traj.append(sp, pmin, pmax);
    return true;
  }
  return false;
}

// Layer A (1-DOF): Solve for exact duration T_target keeping q(t) in [q_min, q_max].
// If T_target is blocked (and !allow_blocked_split) or below the state-constrained minimum,
// returns false and writes a strictly larger candidate duration to out_next_unblocked_T.
bool solve_dof_at_duration(
  double q0, double v0, double a0,
  double q1, double v1, double a1,
  const DoFLimit & lim,
  double T_target,
  bool allow_blocked_split,
  DoFTrajectory & out_traj,
  double & out_next_unblocked_T,
  int depth)
{
  out_traj.clear();
  out_next_unblocked_T = std::numeric_limits<double>::infinity();

  if (std::abs(q1 - q0) < 1e-11 && std::abs(v0) < 1e-10 && std::abs(v1) < 1e-10 &&
      std::abs(a0) < 1e-10 && std::abs(a1) < 1e-10)
  {
    DoFSubPart dwell;
    dwell.kind = SubPartKind::DWELL;
    dwell.duration = std::max(0.0, T_target);
    dwell.dwell_q = std::clamp(q0, lim.q_min, lim.q_max);
    out_traj.append(dwell, dwell.dwell_q, dwell.dwell_q);
    return true;
  }

  ruckig::Profile prof;
  double dur = 0.0;
  ProfileExtrema ext;
  if (!solve_ruckig_1dof_raw(q0, v0, a0, q1, v1, a1, lim, T_target, prof, dur, ext)) {
    return false;
  }

  const bool exact_dur = (std::abs(dur - T_target) <= kTimeMatchTol);
  if (exact_dur && extrema_within(ext, lim)) {
    DoFSubPart part;
    part.kind = SubPartKind::RUCKIG_PROFILE;
    part.duration = dur;
    part.profile = prof;
    out_traj.append(part, ext.min, ext.max);
    return true;
  }

  if (exact_dur && !extrema_within(ext, lim)) {
    // Unconstrained profile at T_target overshot a position limit: apply Pontryagin
    // boundary split at q_lim and absorb remaining slack in extended T1/T2 or T_dwell.
    if (depth >= kMaxSplitDepth) {
      return false;
    }
    const bool hi_vio = (ext.max > lim.q_max + kPosTol);
    const bool lo_vio = (ext.min < lim.q_min - kPosTol);
    double q_lim = lim.q_max;
    if (hi_vio && lo_vio) {
      q_lim = (ext.t_max <= ext.t_min) ? lim.q_max : lim.q_min;
    } else if (lo_vio) {
      q_lim = lim.q_min;
    }

    DoFTrajectory s1_min, s2_min;
    bool c1 = false, c2 = false;
    if (solve_dof_min_time(q0, v0, a0, q_lim, 0.0, 0.0, lim, s1_min, c1, depth + 1) &&
        solve_dof_min_time(q_lim, 0.0, 0.0, q1, v1, a1, lim, s2_min, c2, depth + 1))
    {
      const double t_sum = s1_min.total_duration + s2_min.total_duration;
      if (t_sum <= T_target + kTimeMatchTol) {
        const double slack = std::max(0.0, T_target - t_sum);
        // Try smooth proportional extension of T1, T2 first while staying inside [q_min, q_max].
        if (slack > 1e-6) {
          const double frac = (t_sum > 1e-9) ? (s1_min.total_duration / t_sum) : 0.5;
          const double T1 = s1_min.total_duration + frac * slack;
          const double T2 = T_target - T1;
          DoFTrajectory e1, e2;
          double n1 = 0.0, n2 = 0.0;
          if (solve_dof_at_duration(q0, v0, a0, q_lim, 0.0, 0.0, lim, T1, false, e1, n1, depth + 1) &&
              solve_dof_at_duration(q_lim, 0.0, 0.0, q1, v1, a1, lim, T2, false, e2, n2, depth + 1))
          {
            out_traj.clear();
            out_traj.append_traj(e1);
            out_traj.append_traj(e2);
            return true;
          }
        }
        // Fallback: exact dwell on the boundary rail (q_lim, 0, 0).
        out_traj.clear();
        out_traj.append_traj(s1_min);
        if (slack > 1e-9) {
          DoFSubPart dwell;
          dwell.kind = SubPartKind::DWELL;
          dwell.duration = slack;
          dwell.dwell_q = q_lim;
          out_traj.append(dwell, q_lim, q_lim);
        }
        out_traj.append_traj(s2_min);
        return true;
      }
      // If zero-acceleration clamping at (q_lim, 0, 0) exceeds T_target, try a tangential
      // touch-and-go bounce (q_lim, 0, a_bounce) pointing strictly into the interior.
      const double inward_sgn = (q_lim == lim.q_max) ? -1.0 : 1.0;
      for (double frac : {0.25, 0.5, 0.75, 0.95}) {
        const double a_b = inward_sgn * frac * lim.a_lim * kHeadroom;
        DoFTrajectory b1, b2;
        bool bc1 = false, bc2 = false;
        if (solve_dof_min_time(q0, v0, a0, q_lim, 0.0, a_b, lim, b1, bc1, depth + 1) &&
            solve_dof_min_time(q_lim, 0.0, a_b, q1, v1, a1, lim, b2, bc2, depth + 1))
        {
          const double tb_sum = b1.total_duration + b2.total_duration;
          if (tb_sum <= T_target + kTimeMatchTol) {
            const double slack_b = std::max(0.0, T_target - tb_sum);
            const double ratio_b = (tb_sum > 1e-9) ? (b1.total_duration / tb_sum) : 0.5;
            const double T1_b = b1.total_duration + ratio_b * slack_b;
            const double T2_b = T_target - T1_b;
            DoFTrajectory e1, e2;
            double n1 = 0.0, n2 = 0.0;
            if (solve_dof_at_duration(q0, v0, a0, q_lim, 0.0, a_b, lim, T1_b, false, e1, n1, depth + 1) &&
                solve_dof_at_duration(q_lim, 0.0, a_b, q1, v1, a1, lim, T2_b, false, e2, n2, depth + 1))
            {
              out_traj.clear();
              out_traj.append_traj(e1);
              out_traj.append_traj(e2);
              return true;
            }
          }
        }
      }
      out_next_unblocked_T = t_sum;
      return false;
    }

    // Bounce-without-zero-acceleration edge case when (q_lim, 0, 0) was unreachable.
    DoFTrajectory min_constrained;
    bool clamped = false;
    if (solve_dof_min_time(q0, v0, a0, q1, v1, a1, lim, min_constrained, clamped, depth + 1)) {
      if (min_constrained.total_duration > T_target + kTimeMatchTol) {
        out_next_unblocked_T = min_constrained.total_duration;
        return false;
      }
      if (std::abs(min_constrained.total_duration - T_target) <= kTimeMatchTol) {
        out_traj = std::move(min_constrained);
        return true;
      }
      if (allow_blocked_split) {
        return resolve_blocked_at_exact_duration(
          q0, v0, a0, q1, v1, a1, lim, T_target, out_traj, depth + 1);
      }
    }
    return false;
  }

  // Here dur > T_target + kTimeMatchTol: either T_target < T_min or T_target is in a blocked interval.
  // Check true minimum constrained duration first.
  DoFTrajectory min_traj;
  bool clamped = false;
  if (!solve_dof_min_time(q0, v0, a0, q1, v1, a1, lim, min_traj, clamped, depth + 1)) {
    return false;
  }
  if (T_target + kTimeMatchTol < min_traj.total_duration) {
    out_next_unblocked_T = min_traj.total_duration;
    return false;
  }

  // T_target >= min_traj.total_duration, so T_target landed inside a Ruckig blocked interval.
  if (allow_blocked_split) {
    if (resolve_blocked_at_exact_duration(
          q0, v0, a0, q1, v1, a1, lim, T_target, out_traj, depth))
    {
      return true;
    }
  }
  out_next_unblocked_T = dur;
  return false;
}

double ceil_1ms(double t)
{
  return std::max(kDt, std::ceil(t * 1000.0 - 1e-9) * kDt);
}

double round_1ms(double t)
{
  return std::max(kDt, std::round(t * 1000.0) * kDt);
}

struct SegmentSolution6DoF
{
  std::array<DoFTrajectory, 6> dofs{};
  std::array<double, 6> ind_min_durations{};
  double duration = 0.0;
};

// Diagnoses which scaled limit prevented reaching an explicit target duration T_target on joint j.
void diagnose_dt_failure(
  size_t j, double q0, double v0, double a0, double q1, double v1, double a1,
  const DoFLimit & lim, double T_target, double T_min_j,
  std::string & err_code, std::string & err_msg)
{
  const double dq = std::abs(q1 - q0);
  const double dv = std::abs(v1 - v0);
  const double da = std::abs(a1 - a0);

  if (dq / T_target > lim.v_lim + 1e-6) {
    err_code = "ERR_PVA_VEL_LIMIT";
  } else if (dv / T_target > lim.a_lim + 1e-6) {
    err_code = "ERR_PVA_ACC_LIMIT";
  } else if (da / T_target > lim.j_lim + 1e-3) {
    err_code = "ERR_PVA_JERK_LIMIT";
  } else {
    // Probe which bound is active by relaxing v, a, j individually.
    auto test_relaxed = [&](double vm, double am, double jm) -> bool {
      DoFLimit rlim = lim;
      rlim.v_lim *= vm;
      rlim.a_lim *= am;
      rlim.j_lim *= jm;
      DoFTrajectory tmp;
      bool cl = false;
      return solve_dof_min_time(q0, v0, a0, q1, v1, a1, rlim, tmp, cl) &&
             tmp.total_duration <= T_target + 1e-5;
    };
    const bool v_helps = test_relaxed(4.0, 1.0, 1.0);
    const bool a_helps = test_relaxed(1.0, 4.0, 1.0);
    const bool j_helps = test_relaxed(1.0, 1.0, 4.0);
    if (v_helps && !a_helps && !j_helps) {
      err_code = "ERR_PVA_VEL_LIMIT";
    } else if (a_helps && !j_helps) {
      err_code = "ERR_PVA_ACC_LIMIT";
    } else if (j_helps) {
      err_code = "ERR_PVA_JERK_LIMIT";
    } else {
      // Check if relaxing position limits makes T_target reachable.
      DoFLimit wide = lim;
      wide.q_min = -1e6;
      wide.q_max = 1e6;
      ruckig::Profile p;
      double d = 0.0;
      ProfileExtrema ex;
      if (solve_ruckig_1dof_raw(q0, v0, a0, q1, v1, a1, wide, std::nullopt, p, d, ex) &&
          d <= T_target + 1e-5)
      {
        err_code = "ERR_JOINT_POS_LIMIT";
      } else {
        err_code = "ERR_PVA_ACC_LIMIT";
      }
    }
  }

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4)
      << "MovePVA explicit dt=" << T_target << " s is infeasible on joint" << (j + 1)
      << " (minimum feasible duration under scaled limits is " << T_min_j << " s)";
  err_msg = oss.str();
}

// Layer A (6-DOF): Synchronized state-to-state solver with hard [min_pos, max_pos] and exact dt.
bool solve_segment_6dof(
  const JointPVASolver::SegmentInput & seg,
  const std::array<DoFLimit, 6> & lims,
  bool discretize_1ms,
  SegmentSolution6DoF & out_sol,
  std::string & err_code,
  std::string & err_msg)
{
  // 1. Validate both endpoint states and braking feasibility on every DoF.
  for (size_t j = 0; j < 6; ++j) {
    std::string code, reason;
    if (!validate_dof_state(
          seg.q0[j], seg.v0[j], seg.a0[j], lims[j],
          /*check_forward=*/true, /*check_reverse=*/false, code, reason))
    {
      err_code = code;
      err_msg = "Joint" + std::to_string(j + 1) + " start state invalid: " + reason;
      return false;
    }
    if (!validate_dof_state(
          seg.q1[j], seg.v1[j], seg.a1[j], lims[j],
          /*check_forward=*/false, /*check_reverse=*/true, code, reason))
    {
      err_code = code;
      err_msg = "Joint" + std::to_string(j + 1) + " target state invalid: " + reason;
      return false;
    }
  }

  // 2. Explicit dt mode: enforce exact T_sync = round_1ms(dt).
  if (seg.explicit_dt.has_value()) {
    const double T_target = round_1ms(*seg.explicit_dt);
    if (T_target > kMaxSegmentDurationSec) {
      err_code = "ERR_PVA_DT";
      err_msg = "MovePVA explicit dt exceeds maximum segment duration";
      return false;
    }

    // Check individual minimum durations first for accurate diagnostics.
    for (size_t j = 0; j < 6; ++j) {
      DoFTrajectory min_j;
      bool cl = false;
      if (!solve_dof_min_time(
            seg.q0[j], seg.v0[j], seg.a0[j], seg.q1[j], seg.v1[j], seg.a1[j],
            lims[j], min_j, cl))
      {
        err_code = "ERR_PVA_INFEASIBLE";
        err_msg = "MovePVA segment has no feasible trajectory on joint" + std::to_string(j + 1);
        return false;
      }
      out_sol.ind_min_durations[j] = min_j.total_duration;
      if (min_j.total_duration > T_target + kTimeMatchTol) {
        diagnose_dt_failure(
          j, seg.q0[j], seg.v0[j], seg.a0[j], seg.q1[j], seg.v1[j], seg.a1[j],
          lims[j], T_target, min_j.total_duration, err_code, err_msg);
        return false;
      }
    }

    for (size_t j = 0; j < 6; ++j) {
      double next_T = 0.0;
      if (!solve_dof_at_duration(
            seg.q0[j], seg.v0[j], seg.a0[j], seg.q1[j], seg.v1[j], seg.a1[j],
            lims[j], T_target, /*allow_blocked_split=*/true, out_sol.dofs[j], next_T))
      {
        diagnose_dt_failure(
          j, seg.q0[j], seg.v0[j], seg.a0[j], seg.q1[j], seg.v1[j], seg.a1[j],
          lims[j], T_target, std::isfinite(next_T) ? next_T : out_sol.ind_min_durations[j],
          err_code, err_msg);
        return false;
      }
    }
    out_sol.duration = T_target;
    return true;
  }

  // 3. Free-duration mode: fast unconstrained 6-DOF Ruckig path first.
  bool all_zero = true;
  for (size_t j = 0; j < 6; ++j) {
    if (std::abs(seg.q1[j] - seg.q0[j]) > 1e-11 ||
        std::abs(seg.v0[j]) > 1e-10 || std::abs(seg.v1[j]) > 1e-10 ||
        std::abs(seg.a0[j]) > 1e-10 || std::abs(seg.a1[j]) > 1e-10)
    {
      all_zero = false;
      break;
    }
  }
  if (all_zero) {
    const double T0 = discretize_1ms ? kDt : 0.0;
    for (size_t j = 0; j < 6; ++j) {
      out_sol.dofs[j].clear();
      DoFSubPart dwell;
      dwell.kind = SubPartKind::DWELL;
      dwell.duration = T0;
      dwell.dwell_q = std::clamp(seg.q0[j], lims[j].q_min, lims[j].q_max);
      out_sol.dofs[j].append(dwell, dwell.dwell_q, dwell.dwell_q);
      out_sol.ind_min_durations[j] = 0.0;
    }
    out_sol.duration = T0;
    return true;
  }

  {
    ruckig::InputParameter<6> inp;
    inp.control_interface = ruckig::ControlInterface::Position;
    inp.synchronization = ruckig::Synchronization::Time;
    inp.duration_discretization = discretize_1ms ?
      ruckig::DurationDiscretization::Discrete :
      ruckig::DurationDiscretization::Continuous;
    for (size_t j = 0; j < 6; ++j) {
      double vc = 0.0, ac = 0.0, jc = 0.0;
      compute_working_caps(seg.v0[j], seg.a0[j], seg.v1[j], seg.a1[j], lims[j], vc, ac, jc);
      inp.current_position[j] = seg.q0[j];
      inp.current_velocity[j] = seg.v0[j];
      inp.current_acceleration[j] = seg.a0[j];
      inp.target_position[j] = seg.q1[j];
      inp.target_velocity[j] = seg.v1[j];
      inp.target_acceleration[j] = seg.a1[j];
      inp.max_velocity[j] = vc;
      inp.max_acceleration[j] = ac;
      inp.max_jerk[j] = jc;
    }
    ruckig::Ruckig<6> otg(kDt);
    ruckig::Trajectory<6> traj;
    const auto res = otg.calculate(inp, traj);
    if (static_cast<int>(res) >= 0) {
      const double dur = traj.get_duration();
      if (std::isfinite(dur) && dur >= 0.0 && dur <= kMaxSegmentDurationSec) {
        const auto profiles = traj.get_profiles()[0];
        bool all_within = true;
        std::array<ProfileExtrema, 6> exts{};
        for (size_t j = 0; j < 6; ++j) {
          exts[j] = compute_profile_extrema(profiles[j]);
          if (!extrema_within(exts[j], lims[j])) {
            all_within = false;
            break;
          }
        }
        if (all_within) {
          const auto ind = traj.get_independent_min_durations();
          for (size_t j = 0; j < 6; ++j) {
            out_sol.dofs[j].clear();
            DoFSubPart part;
            part.kind = SubPartKind::RUCKIG_PROFILE;
            part.duration = dur;
            part.profile = profiles[j];
            out_sol.dofs[j].append(part, exts[j].min, exts[j].max);
            out_sol.ind_min_durations[j] = ind[j];
          }
          out_sol.duration = dur;
          return true;
        }
      }
    }
  }

  // 4. Position-constrained 6-DOF synchronization.
  double T_cand = 0.0;
  for (size_t j = 0; j < 6; ++j) {
    DoFTrajectory min_j;
    bool cl = false;
    if (!solve_dof_min_time(
          seg.q0[j], seg.v0[j], seg.a0[j], seg.q1[j], seg.v1[j], seg.a1[j],
          lims[j], min_j, cl))
    {
      err_code = "ERR_JOINT_POS_LIMIT";
      err_msg = "Joint" + std::to_string(j + 1) +
        " cannot satisfy position limits under scaled (v, a, j) bounds";
      return false;
    }
    out_sol.ind_min_durations[j] = min_j.total_duration;
    T_cand = std::max(T_cand, min_j.total_duration);
  }
  if (discretize_1ms) {
    T_cand = ceil_1ms(T_cand);
  }

  for (int iter = 0; iter < 16; ++iter) {
    if (T_cand > kMaxSegmentDurationSec) {
      break;
    }
    bool all_ok = true;
    double max_next = T_cand;
    for (size_t j = 0; j < 6; ++j) {
      double next_T = 0.0;
      if (!solve_dof_at_duration(
            seg.q0[j], seg.v0[j], seg.a0[j], seg.q1[j], seg.v1[j], seg.a1[j],
            lims[j], T_cand, /*allow_blocked_split=*/false, out_sol.dofs[j], next_T))
      {
        all_ok = false;
        if (std::isfinite(next_T) && next_T > max_next) {
          max_next = next_T;
        }
      }
    }
    if (all_ok) {
      out_sol.duration = T_cand;
      return true;
    }
    if (max_next <= T_cand + 1e-7) {
      break;
    }
    T_cand = discretize_1ms ? ceil_1ms(max_next) : max_next;
  }

  // Final attempt at tightest T_cand allowing 2-section blocked-interval resolution.
  T_cand = 0.0;
  for (size_t j = 0; j < 6; ++j) {
    T_cand = std::max(T_cand, out_sol.ind_min_durations[j]);
  }
  if (discretize_1ms) {
    T_cand = ceil_1ms(T_cand);
  }
  for (int bump = 0; bump < 6; ++bump) {
    bool all_ok = true;
    for (size_t j = 0; j < 6; ++j) {
      double next_T = 0.0;
      if (!solve_dof_at_duration(
            seg.q0[j], seg.v0[j], seg.a0[j], seg.q1[j], seg.v1[j], seg.a1[j],
            lims[j], T_cand, /*allow_blocked_split=*/true, out_sol.dofs[j], next_T))
      {
        all_ok = false;
        break;
      }
    }
    if (all_ok) {
      out_sol.duration = T_cand;
      return true;
    }
    T_cand = discretize_1ms ? ceil_1ms(T_cand * 1.05 + kDt) : (T_cand * 1.05 + kDt);
    if (T_cand > kMaxSegmentDurationSec) {
      break;
    }
  }

  err_code = "ERR_PVA_INFEASIBLE";
  err_msg = "MovePVA segment could not synchronize all 6 joints within limits";
  return false;
}

std::array<DoFLimit, 6> build_dof_limits(
  const std::array<lite6_planner::JointLimit, 6> & limits,
  double vel_scale, double acc_scale, double jerk_scale)
{
  std::array<DoFLimit, 6> out{};
  for (size_t j = 0; j < 6; ++j) {
    out[j].q_min = limits[j].min_pos;
    out[j].q_max = limits[j].max_pos;
    out[j].v_lim = limits[j].max_vel * vel_scale;
    out[j].a_lim = limits[j].max_acc * acc_scale;
    out[j].j_lim = limits[j].max_jerk * jerk_scale;
  }
  return out;
}

// Fast 1-DOF Layer-A cost oracle for Layer-B coordinate optimization:
// returns the minimum feasible continuous duration for joint j on a single segment
// (or +inf if infeasible or violating explicit_dt).
double eval_dof_segment_cost(
  double q0, double v0, double a0,
  double q1, double v1, double a1,
  const DoFLimit & lim,
  std::optional<double> explicit_dt)
{
  std::string dummy_c, dummy_r;
  if (!validate_dof_state(q0, v0, a0, lim, true, false, dummy_c, dummy_r) ||
      !validate_dof_state(q1, v1, a1, lim, false, true, dummy_c, dummy_r))
  {
    return std::numeric_limits<double>::infinity();
  }
  if (explicit_dt.has_value()) {
    const double T_target = round_1ms(*explicit_dt);
    DoFTrajectory tr;
    double next_T = 0.0;
    if (!solve_dof_at_duration(q0, v0, a0, q1, v1, a1, lim, T_target, true, tr, next_T)) {
      return std::numeric_limits<double>::infinity();
    }
    return T_target;
  }
  DoFTrajectory tr;
  bool clamped = false;
  if (!solve_dof_min_time(q0, v0, a0, q1, v1, a1, lim, tr, clamped)) {
    return std::numeric_limits<double>::infinity();
  }
  return tr.total_duration;
}

// Samples a verified SegmentSolution6DoF onto the 1 kHz grid.
void sample_segment_1khz(
  const SegmentSolution6DoF & sol,
  const SourceLocation & loc,
  double t_offset,
  size_t seg_idx,
  size_t num_segs,
  bool is_first_seg,
  bool is_last_seg,
  std::vector<SampledTrajectory1kHz::Sample> & out_samples)
{
  const int ticks = std::max(1, static_cast<int>(std::llround(sol.duration * 1000.0)));
  const int start_tick = is_first_seg ? 0 : 1;
  for (int tick = start_tick; tick <= ticks; ++tick) {
    const double t_loc = static_cast<double>(tick) * kDt;
    SampledTrajectory1kHz::Sample smp;
    smp.time_sec = t_offset + t_loc;
    smp.path_u = (static_cast<double>(seg_idx) +
                  static_cast<double>(tick) / static_cast<double>(ticks)) /
                 static_cast<double>(num_segs);
    for (size_t j = 0; j < 6; ++j) {
      double q = 0.0, v = 0.0, a = 0.0, jrk = 0.0;
      sol.dofs[j].eval(t_loc, q, v, a, jrk);
      smp.q[j] = q;
      smp.dq[j] = v;
      smp.ddq[j] = a;
    }
    if ((is_first_seg && tick == 0) || (is_last_seg && tick == ticks)) {
      smp.dq.fill(0.0);
      smp.ddq.fill(0.0);
    }
    smp.loc = loc;
    out_samples.push_back(smp);
  }
}

}  // namespace

bool JointPVASolver::solve_segment(
  const SegmentInput & seg,
  const std::array<lite6_planner::JointLimit, 6> & limits,
  double vel_scale,
  double acc_scale,
  double jerk_scale,
  const SourceLocation & loc,
  SampledTrajectory1kHz & out_traj,
  std::string & err_code,
  std::string & err_msg)
{
  out_traj.samples.clear();
  out_traj.duration_sec = 0.0;
  const auto lims = build_dof_limits(limits, vel_scale, acc_scale, jerk_scale);
  for (size_t j = 0; j < 6; ++j) {
    if (!(lims[j].v_lim > 0.0) || !(lims[j].a_lim > 0.0) || !(lims[j].j_lim > 0.0)) {
      err_code = "ERR_INVALID_LIMITS";
      err_msg = "Joint limits must be strictly positive";
      return false;
    }
  }

  SegmentSolution6DoF sol;
  if (!solve_segment_6dof(seg, lims, /*discretize_1ms=*/true, sol, err_code, err_msg)) {
    return false;
  }

  const int ticks = std::max(1, static_cast<int>(std::llround(sol.duration * 1000.0)));
  out_traj.samples.reserve(static_cast<size_t>(ticks + 1));
  for (int tick = 0; tick <= ticks; ++tick) {
    const double t_loc = static_cast<double>(tick) * kDt;
    SampledTrajectory1kHz::Sample smp;
    smp.time_sec = t_loc;
    smp.path_u = static_cast<double>(tick) / static_cast<double>(ticks);
    for (size_t j = 0; j < 6; ++j) {
      double q = 0.0, v = 0.0, a = 0.0, jrk = 0.0;
      sol.dofs[j].eval(t_loc, q, v, a, jrk);
      smp.q[j] = q;
      smp.dq[j] = v;
      smp.ddq[j] = a;
    }
    smp.loc = loc;
    out_traj.samples.push_back(smp);
  }
  out_traj.duration_sec = static_cast<double>(ticks) * kDt;
  return true;
}

bool JointPVASolver::solve_multi_waypoint(
  const std::array<double, 6> & start_q,
  const std::vector<PVAWaypoint> & waypoints,
  const std::array<lite6_planner::JointLimit, 6> & limits,
  double vel_scale,
  double acc_scale,
  double jerk_scale,
  MultiWaypointResult & out_result,
  SourceLocation & err_loc,
  std::string & err_code,
  std::string & err_msg)
{
  out_result = MultiWaypointResult{};
  if (waypoints.empty()) {
    err_code = "ERR_EMPTY_PVA";
    err_msg = "MovePVA requires at least one waypoint";
    return false;
  }

  const size_t M = waypoints.size();
  const auto lims = build_dof_limits(limits, vel_scale, acc_scale, jerk_scale);
  for (size_t j = 0; j < 6; ++j) {
    if (!(lims[j].v_lim > 0.0) || !(lims[j].a_lim > 0.0) || !(lims[j].j_lim > 0.0)) {
      err_loc = waypoints.front().loc;
      err_code = "ERR_INVALID_LIMITS";
      err_msg = "Scaled joint limits must be strictly positive";
      return false;
    }
  }

  // 1. Assemble node positions, fixed/free masks, and explicit dt values.
  std::vector<std::array<double, 6>> q_nodes(M + 1);
  std::vector<std::array<double, 6>> v_nodes(M + 1, std::array<double, 6>{});
  std::vector<std::array<double, 6>> a_nodes(M + 1, std::array<double, 6>{});
  std::vector<bool> v_fixed(M + 1, false);
  std::vector<bool> a_fixed(M + 1, false);
  std::vector<std::optional<double>> seg_dt(M);

  q_nodes[0] = start_q;
  v_fixed[0] = true;
  a_fixed[0] = true;
  v_fixed[M] = true;
  a_fixed[M] = true;

  for (size_t k = 0; k < M; ++k) {
    const auto & wp = waypoints[k];
    q_nodes[k + 1] = wp.joint_pos;
    seg_dt[k] = wp.dt;

    // Validate waypoint positions against mechanical limits.
    for (size_t j = 0; j < 6; ++j) {
      if (!std::isfinite(wp.joint_pos[j]) ||
          wp.joint_pos[j] < lims[j].q_min - lite6_planner::tolerance::kJointPosition ||
          wp.joint_pos[j] > lims[j].q_max + lite6_planner::tolerance::kJointPosition)
      {
        err_loc = wp.loc;
        err_code = "ERR_JOINT_POS_LIMIT";
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(5)
            << "MovePVA waypoint " << (k + 1) << " violates joint" << (j + 1)
            << " position limit: value=" << wp.joint_pos[j] << " rad, range=["
            << lims[j].q_min << ", " << lims[j].q_max << "] rad";
        err_msg = oss.str();
        return false;
      }
      q_nodes[k + 1][j] = std::clamp(wp.joint_pos[j], lims[j].q_min, lims[j].q_max);
    }

    if (k + 1 < M) {
      if (wp.velocity.has_value()) {
        v_nodes[k + 1] = *wp.velocity;
        v_fixed[k + 1] = true;
      }
      if (wp.acceleration.has_value()) {
        a_nodes[k + 1] = *wp.acceleration;
        a_fixed[k + 1] = true;
      }
      // Validate user-pinned intermediate velocity/acceleration immediately.
      for (size_t j = 0; j < 6; ++j) {
        if (v_fixed[k + 1]) {
          std::string code, reason;
          const double a_chk = a_fixed[k + 1] ? a_nodes[k + 1][j] : 0.0;
          if (!validate_dof_state(
                q_nodes[k + 1][j], v_nodes[k + 1][j], a_chk, lims[j],
                /*check_forward=*/true, /*check_reverse=*/true, code, reason))
          {
            err_loc = wp.loc;
            err_code = code;
            err_msg = "MovePVA waypoint " + std::to_string(k + 1) +
              " joint" + std::to_string(j + 1) + ": " + reason;
            return false;
          }
        }
      }
    }
  }

  // 2. Initialize free interior velocities with braking-safe Catmull-Rom tangents
  //    and free interior accelerations to 0.
  std::vector<double> est_dt(M, 0.2);
  for (size_t m = 0; m < M; ++m) {
    if (seg_dt[m].has_value()) {
      est_dt[m] = std::max(0.01, *seg_dt[m]);
    } else {
      double t_est = 0.05;
      for (size_t j = 0; j < 6; ++j) {
        const double dq = std::abs(q_nodes[m + 1][j] - q_nodes[m][j]);
        t_est = std::max(t_est, dq / std::max(1e-4, 0.65 * lims[j].v_lim));
      }
      est_dt[m] = t_est;
    }
  }

  auto braking_safe_vel_bound = [&](size_t node_k, size_t j) -> double {
    const double margin_pos = 0.96 * std::min(
      lims[j].q_max - q_nodes[node_k][j],
      q_nodes[node_k][j] - lims[j].q_min);
    const double v_pos = max_speed_for_distance(
      std::max(0.0, margin_pos), lims[j].a_lim * kHeadroom, lims[j].j_lim * kHeadroom);
    return std::min(lims[j].v_lim * kHeadroom, v_pos);
  };

  std::vector<std::array<double, 6>> v_catmull = v_nodes;
  std::vector<std::array<double, 6>> a_init = a_nodes;
  bool has_any_free = false;

  for (size_t k = 1; k < M; ++k) {
    if (!v_fixed[k] || !a_fixed[k]) {
      has_any_free = true;
    }
    if (!v_fixed[k]) {
      const double tm = est_dt[k - 1];
      const double tp = est_dt[k];
      for (size_t j = 0; j < 6; ++j) {
        const double sm = (q_nodes[k][j] - q_nodes[k - 1][j]) / tm;
        const double sp = (q_nodes[k + 1][j] - q_nodes[k][j]) / tp;
        double v_cr = 0.0;
        if (sm * sp > 0.0) {
          // Time-weighted Catmull-Rom tangent (zero at local direction reversals).
          v_cr = (tp * sm + tm * sp) / (tm + tp);
        }
        const double v_bnd = braking_safe_vel_bound(k, j);
        v_catmull[k][j] = std::clamp(v_cr, -v_bnd, v_bnd);
      }
    }
    if (!a_fixed[k]) {
      a_init[k].fill(0.0);
    }
  }

  // 3. Layer B: Bounded derivative-free multi-waypoint (v_k, a_k) optimization over
  //    the fast Layer-A per-DoF duration oracle.
  if (has_any_free) {
    // Table of per-segment, per-DoF continuous durations: T_mat[m][j].
    auto build_t_mat = [&](
      const std::vector<std::array<double, 6>> & vn,
      const std::vector<std::array<double, 6>> & an,
      std::vector<std::array<double, 6>> & t_mat) -> double
    {
      t_mat.assign(M, std::array<double, 6>{});
      double total = 0.0;
      double sum_ind = 0.0;
      for (size_t m = 0; m < M; ++m) {
        double seg_max = seg_dt[m].has_value() ? round_1ms(*seg_dt[m]) : 0.0;
        for (size_t j = 0; j < 6; ++j) {
          const double tj = eval_dof_segment_cost(
            q_nodes[m][j], vn[m][j], an[m][j],
            q_nodes[m + 1][j], vn[m + 1][j], an[m + 1][j],
            lims[j], seg_dt[m]);
          if (!std::isfinite(tj)) {
            return std::numeric_limits<double>::infinity();
          }
          t_mat[m][j] = tj;
          seg_max = std::max(seg_max, tj);
          sum_ind += tj;
        }
        total += seg_max;
      }
      return total + 1e-4 * sum_ind;
    };

    std::vector<std::array<double, 6>> t_mat;
    double best_obj = build_t_mat(v_catmull, a_init, t_mat);
    v_nodes = v_catmull;
    a_nodes = a_init;

    // Also evaluate the zero-free-state seed in case tight explicit dt prefers it.
    std::vector<std::array<double, 6>> v_zero = v_nodes;
    for (size_t k = 1; k < M; ++k) {
      if (!v_fixed[k]) {
        v_zero[k].fill(0.0);
      }
    }
    std::vector<std::array<double, 6>> t_mat_zero;
    const double zero_obj = build_t_mat(v_zero, a_init, t_mat_zero);
    if (zero_obj < best_obj) {
      best_obj = zero_obj;
      v_nodes = v_zero;
      t_mat = t_mat_zero;
    }

    // Phase-I feasibility restoration if explicit dt makes both seeds infeasible on some segment.
    if (!std::isfinite(best_obj)) {
      auto violation_cost = [&](
        const std::vector<std::array<double, 6>> & vn,
        const std::vector<std::array<double, 6>> & an) -> double
      {
        double vio = 0.0;
        for (size_t m = 0; m < M; ++m) {
          const double target = seg_dt[m].has_value() ? round_1ms(*seg_dt[m]) : kMaxSegmentDurationSec;
          for (size_t j = 0; j < 6; ++j) {
            const double tj = eval_dof_segment_cost(
              q_nodes[m][j], vn[m][j], an[m][j],
              q_nodes[m + 1][j], vn[m + 1][j], an[m + 1][j],
              lims[j], std::nullopt);
            if (!std::isfinite(tj)) {
              return std::numeric_limits<double>::infinity();
            }
            if (seg_dt[m].has_value() && tj > target) {
              vio += (tj - target);
            }
          }
        }
        return vio;
      };

      v_nodes = v_catmull;
      a_nodes = a_init;
      double cur_vio = violation_cost(v_nodes, a_nodes);
      if (!std::isfinite(cur_vio)) {
        v_nodes = v_zero;
        cur_vio = violation_cost(v_nodes, a_nodes);
      }
      if (std::isfinite(cur_vio) && cur_vio > 0.0) {
        for (int sweep = 0; sweep < 6 && cur_vio > 1e-6; ++sweep) {
          const double scale = std::pow(0.5, sweep);
          for (size_t k = 1; k < M; ++k) {
            for (size_t j = 0; j < 6; ++j) {
              if (!v_fixed[k]) {
                const double vb = braking_safe_vel_bound(k, j);
                const double dv = 0.35 * scale * vb;
                for (double sgn : {1.0, -1.0}) {
                  const double orig = v_nodes[k][j];
                  v_nodes[k][j] = std::clamp(orig + sgn * dv, -vb, vb);
                  const double nv = violation_cost(v_nodes, a_nodes);
                  if (nv < cur_vio - 1e-9) {
                    cur_vio = nv;
                    break;
                  }
                  v_nodes[k][j] = orig;
                }
              }
              if (!a_fixed[k]) {
                const double ab = lims[j].a_lim * kHeadroom;
                const double da = 0.35 * scale * ab;
                for (double sgn : {1.0, -1.0}) {
                  const double orig = a_nodes[k][j];
                  a_nodes[k][j] = std::clamp(orig + sgn * da, -ab, ab);
                  const double nv = violation_cost(v_nodes, a_nodes);
                  if (nv < cur_vio - 1e-9) {
                    cur_vio = nv;
                    break;
                  }
                  a_nodes[k][j] = orig;
                }
              }
            }
          }
        }
        best_obj = build_t_mat(v_nodes, a_nodes, t_mat);
      }
    }

    // Phase-II: Fast local coordinate-pattern + coupled (v, a) + Powell extrapolation search.
    if (std::isfinite(best_obj)) {
      const auto opt_start = std::chrono::steady_clock::now();
      constexpr auto kOptBudget = std::chrono::milliseconds(150);

      // Local delta objective when changing (v[k][j], a[k][j]): affects ONLY segments k-1 and k
      // and ONLY DoF j within those two segments!
      auto local_pair_cost = [&](
        size_t k, size_t j, double vk_try, double ak_try,
        double & out_tm1, double & out_tp) -> double
      {
        const double tm1 = eval_dof_segment_cost(
          q_nodes[k - 1][j], v_nodes[k - 1][j], a_nodes[k - 1][j],
          q_nodes[k][j], vk_try, ak_try, lims[j], seg_dt[k - 1]);
        if (!std::isfinite(tm1)) {
          return std::numeric_limits<double>::infinity();
        }
        const double tp = eval_dof_segment_cost(
          q_nodes[k][j], vk_try, ak_try,
          q_nodes[k + 1][j], v_nodes[k + 1][j], a_nodes[k + 1][j],
          lims[j], seg_dt[k]);
        if (!std::isfinite(tp)) {
          return std::numeric_limits<double>::infinity();
        }
        out_tm1 = tm1;
        out_tp = tp;

        double max_m1 = seg_dt[k - 1].has_value() ? round_1ms(*seg_dt[k - 1]) : tm1;
        double max_p = seg_dt[k].has_value() ? round_1ms(*seg_dt[k]) : tp;
        for (size_t d = 0; d < 6; ++d) {
          if (d == j) {
            continue;
          }
          max_m1 = std::max(max_m1, t_mat[k - 1][d]);
          max_p = std::max(max_p, t_mat[k][d]);
        }
        return (max_m1 + max_p) + 1e-4 * (tm1 + tp);
      };

      std::vector<std::array<double, 6>> step_v(M + 1), step_a(M + 1);
      for (size_t k = 1; k < M; ++k) {
        for (size_t j = 0; j < 6; ++j) {
          step_v[k][j] = 0.45 * lims[j].v_lim * kHeadroom;
          step_a[k][j] = 0.45 * lims[j].a_lim * kHeadroom;
        }
      }

      for (int sweep = 0; sweep < 10; ++sweep) {
        if (std::chrono::steady_clock::now() - opt_start > kOptBudget) {
          break;
        }
        const auto v_prev = v_nodes;
        const auto a_prev = a_nodes;
        const double obj_before = best_obj;
        bool any_improved = false;

        for (size_t k = 1; k < M; ++k) {
          for (size_t j = 0; j < 6; ++j) {
            if (v_fixed[k] && a_fixed[k]) {
              continue;
            }
            const double v_max_k = lims[j].v_lim * kHeadroom;
            const double a_max_k = lims[j].a_lim * kHeadroom;

            double cur_tm1 = t_mat[k - 1][j];
            double cur_tp = t_mat[k][j];
            double cur_local = local_pair_cost(k, j, v_nodes[k][j], a_nodes[k][j], cur_tm1, cur_tp);

            auto try_state = [&](double vt, double at) -> bool {
              vt = std::clamp(vt, -v_max_k, v_max_k);
              at = std::clamp(at, -a_max_k, a_max_k);
              double tm1 = 0.0, tp = 0.0;
              const double c = local_pair_cost(k, j, vt, at, tm1, tp);
              if (c < cur_local - 1e-7) {
                v_nodes[k][j] = vt;
                a_nodes[k][j] = at;
                t_mat[k - 1][j] = tm1;
                t_mat[k][j] = tp;
                cur_local = c;
                any_improved = true;
                return true;
              }
              return false;
            };

            // 1D velocity coordinate step (with adaptive expansion/contraction).
            if (!v_fixed[k]) {
              const double sv = step_v[k][j];
              if (try_state(v_nodes[k][j] + sv, a_nodes[k][j])) {
                try_state(v_nodes[k][j] + 0.5 * sv, a_nodes[k][j]);
              } else if (try_state(v_nodes[k][j] - sv, a_nodes[k][j])) {
                try_state(v_nodes[k][j] - 0.5 * sv, a_nodes[k][j]);
              } else {
                step_v[k][j] *= 0.5;
              }
            }

            // 1D acceleration coordinate step.
            if (!a_fixed[k]) {
              const double sa = step_a[k][j];
              if (try_state(v_nodes[k][j], a_nodes[k][j] + sa)) {
                try_state(v_nodes[k][j], a_nodes[k][j] + 0.5 * sa);
              } else if (try_state(v_nodes[k][j], a_nodes[k][j] - sa)) {
                try_state(v_nodes[k][j], a_nodes[k][j] - 0.5 * sa);
              } else {
                step_a[k][j] *= 0.5;
              }
            }

            // Coupled 2D (v, a) diagonal probes when both are free.
            if (!v_fixed[k] && !a_fixed[k]) {
              const double sv = step_v[k][j];
              const double sa = step_a[k][j];
              const double v0_c = v_nodes[k][j];
              const double a0_c = a_nodes[k][j];
              if (!try_state(v0_c + sv, a0_c + sa)) {
                if (!try_state(v0_c - sv, a0_c - sa)) {
                  if (!try_state(v0_c + sv, a0_c - sa)) {
                    try_state(v0_c - sv, a0_c + sa);
                  }
                }
              }
            }
          }
        }

        // Powell-style extrapolated multi-waypoint direction step across all free variables.
        if (any_improved) {
          for (double alpha : {1.0, 0.5}) {
            auto v_try = v_nodes;
            auto a_try = a_nodes;
            for (size_t k = 1; k < M; ++k) {
              for (size_t j = 0; j < 6; ++j) {
                if (!v_fixed[k]) {
                  v_try[k][j] = std::clamp(
                    v_nodes[k][j] + alpha * (v_nodes[k][j] - v_prev[k][j]),
                    -lims[j].v_lim * kHeadroom, lims[j].v_lim * kHeadroom);
                }
                if (!a_fixed[k]) {
                  a_try[k][j] = std::clamp(
                    a_nodes[k][j] + alpha * (a_nodes[k][j] - a_prev[k][j]),
                    -lims[j].a_lim * kHeadroom, lims[j].a_lim * kHeadroom);
                }
              }
            }
            std::vector<std::array<double, 6>> t_try;
            const double o_try = build_t_mat(v_try, a_try, t_try);
            if (o_try < best_obj - 1e-6) {
              v_nodes.swap(v_try);
              a_nodes.swap(a_try);
              t_mat.swap(t_try);
              best_obj = o_try;
              break;
            }
          }
          best_obj = build_t_mat(v_nodes, a_nodes, t_mat);
        }

        const double rel_imp = (obj_before - best_obj) / std::max(obj_before, 1e-6);
        if (!any_improved && sweep >= 2) {
          // Shrink remaining step sizes and check if converged.
          double max_step = 0.0;
          for (size_t k = 1; k < M; ++k) {
            for (size_t j = 0; j < 6; ++j) {
              if (!v_fixed[k]) {
                max_step = std::max(max_step, step_v[k][j] / lims[j].v_lim);
              }
              if (!a_fixed[k]) {
                max_step = std::max(max_step, step_a[k][j] / lims[j].a_lim);
              }
            }
          }
          if (max_step < 1e-3) {
            break;
          }
        } else if (rel_imp < 1e-4 && sweep >= 3) {
          break;
        }
      }
    }
  }

  // 4. Final 1 ms synchronized Layer-A solve for all M segments and 1 kHz sampling.
  std::vector<SegmentSolution6DoF> seg_sols(M);
  double total_dur = 0.0;
  for (size_t m = 0; m < M; ++m) {
    SegmentInput seg_in;
    seg_in.q0 = q_nodes[m];
    seg_in.v0 = v_nodes[m];
    seg_in.a0 = a_nodes[m];
    seg_in.q1 = q_nodes[m + 1];
    seg_in.v1 = v_nodes[m + 1];
    seg_in.a1 = a_nodes[m + 1];
    seg_in.explicit_dt = seg_dt[m];

    std::string seg_code, seg_msg;
    if (!solve_segment_6dof(seg_in, lims, /*discretize_1ms=*/true, seg_sols[m], seg_code, seg_msg)) {
      err_loc = waypoints[m].loc;
      err_code = seg_code;
      err_msg = "MovePVA segment " + std::to_string(m + 1) + ": " + seg_msg;
      return false;
    }
    total_dur += seg_sols[m].duration;
    if (total_dur > kMaxTotalDurationSec) {
      err_loc = waypoints[m].loc;
      err_code = "ERR_PVA_DURATION";
      err_msg = "MovePVA total duration=" + std::to_string(total_dur) +
        " s exceeds 300 s limit";
      return false;
    }
  }

  out_result.segment_durations.resize(M);
  out_result.optimized_v = v_nodes;
  out_result.optimized_a = a_nodes;
  const size_t est_samples = static_cast<size_t>(std::llround(total_dur * 1000.0)) + 2;
  out_result.trajectory.samples.reserve(est_samples);

  double t_accum = 0.0;
  for (size_t m = 0; m < M; ++m) {
    out_result.segment_durations[m] = seg_sols[m].duration;
    sample_segment_1khz(
      seg_sols[m], waypoints[m].loc, t_accum, m, M,
      /*is_first_seg=*/(m == 0), /*is_last_seg=*/(m + 1 == M),
      out_result.trajectory.samples);
    t_accum += static_cast<double>(std::max(1, static_cast<int>(std::llround(seg_sols[m].duration * 1000.0)))) * kDt;
  }
  out_result.trajectory.duration_sec = t_accum;
  return true;
}

}  // namespace lite6_studio_planner
