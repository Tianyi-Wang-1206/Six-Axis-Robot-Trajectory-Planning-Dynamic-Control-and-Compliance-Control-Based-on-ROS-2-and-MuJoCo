#include "lite6_studio_planner/joint_pva_solver.hpp"
#include "lite6_studio_planner/third_order_otg.hpp"
#include "lite6_kinematics/motion_tolerances.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

namespace lite6_studio_planner
{
namespace
{

constexpr double kDt = 0.001;
constexpr double kHeadroom = 0.996;
constexpr double kPosTol = 1e-6;
constexpr double kMaxSegmentDurationSec = 120.0;
constexpr double kMaxTotalDurationSec = 300.0;

struct DoFLimit
{
  double q_min = 0.0;
  double q_max = 0.0;
  double v_lim = 0.0;
  double a_lim = 0.0;
  double j_lim = 0.0;
};

double round_1ms(double t)
{
  return std::max(kDt, std::round(t * 1000.0) * kDt);
}

std::array<DoFLimit, 6> build_dof_limits(
  const std::array<lite6_kinematics::JointLimit, 6> & limits,
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

otg::AxisLimits to_hard_axis_limits(const DoFLimit & lim)
{
  otg::AxisLimits al;
  al.q_min = lim.q_min;
  al.q_max = lim.q_max;
  al.v_min = -lim.v_lim;
  al.v_max = lim.v_lim;
  al.a_min = -lim.a_lim;
  al.a_max = lim.a_lim;
  al.j_max = lim.j_lim;
  return al;
}

// Compute working 1-DOF OTG limits preserving kHeadroom (0.996) interior margin while
// permitting valid user-pinned boundary states up to 100% of scaled limits.
otg::AxisLimits build_working_axis_limits(
  double v0, double a0, double v1, double a1, const DoFLimit & lim)
{
  double j_cap = lim.j_lim * kHeadroom;
  const double a_cap = std::min(
    lim.a_lim,
    std::max({lim.a_lim * kHeadroom, std::abs(a0) + 1e-9, std::abs(a1) + 1e-9}));

  auto compute_v_req = [&](double j_use) {
    double vr = std::max({lim.v_lim * kHeadroom, std::abs(v0) + 1e-9, std::abs(v1) + 1e-9});
    if (a0 > 0.0) {
      vr = std::max(vr, std::abs(v0 + (a0 * a0) / (2.0 * j_use)) + 1e-9);
    } else if (a0 < 0.0) {
      vr = std::max(vr, std::abs(v0 - (a0 * a0) / (2.0 * j_use)) + 1e-9);
    }
    if (a1 < 0.0) {
      vr = std::max(vr, std::abs(v1 + (a1 * a1) / (2.0 * j_use)) + 1e-9);
    } else if (a1 > 0.0) {
      vr = std::max(vr, std::abs(v1 - (a1 * a1) / (2.0 * j_use)) + 1e-9);
    }
    return vr;
  };

  double v_req = compute_v_req(j_cap);
  if (v_req > lim.v_lim) {
    j_cap = lim.j_lim;
    v_req = compute_v_req(j_cap);
  }
  const double v_cap = std::min(lim.v_lim * (1.0 + 1e-7), v_req);

  otg::AxisLimits al;
  al.q_min = lim.q_min;
  al.q_max = lim.q_max;
  al.v_min = -v_cap;
  al.v_max = v_cap;
  al.a_min = -a_cap;
  al.a_max = a_cap;
  al.j_max = j_cap;
  return al;
}

bool validate_dof_state_diag(
  double q, double v, double a, const DoFLimit & lim,
  bool check_forward, bool check_reverse,
  std::string & out_code, std::string & out_reason)
{
  const otg::AxisLimits al = to_hard_axis_limits(lim);
  const char * tag = nullptr;
  if (otg::validate_axis_state({q, v, a}, al, check_forward, check_reverse, &tag)) {
    return true;
  }
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(5);
  if (tag != nullptr && std::strcmp(tag, "NONFINITE") == 0) {
    out_code = "ERR_NONFINITE_SAMPLE";
    out_reason = "non-finite waypoint state";
  } else if (tag != nullptr && std::strcmp(tag, "POS_LIMIT") == 0) {
    out_code = "ERR_JOINT_POS_LIMIT";
    oss << "position=" << q << " rad outside [" << lim.q_min << ", " << lim.q_max << "] rad";
    out_reason = oss.str();
  } else if (tag != nullptr && std::strcmp(tag, "VEL_LIMIT") == 0) {
    out_code = "ERR_PVA_VEL_LIMIT";
    oss << "|v|=" << std::abs(v) << " rad/s exceeds scaled limit=" << lim.v_lim << " rad/s";
    out_reason = oss.str();
  } else if (tag != nullptr && std::strcmp(tag, "ACC_LIMIT") == 0) {
    out_code = "ERR_PVA_ACC_LIMIT";
    oss << "|a|=" << std::abs(a) << " rad/s^2 exceeds scaled limit=" << lim.a_lim << " rad/s^2";
    out_reason = oss.str();
  } else if (tag != nullptr && std::strcmp(tag, "VEL_REACH_FWD") == 0) {
    out_code = "ERR_PVA_VEL_LIMIT";
    out_reason = "waypoint (v, a) unavoidably overshoots scaled velocity limit under jerk bound";
  } else if (tag != nullptr && std::strcmp(tag, "VEL_REACH_REV") == 0) {
    out_code = "ERR_PVA_VEL_LIMIT";
    out_reason = "waypoint (v, a) cannot be approached without exceeding scaled velocity limit";
  } else if (tag != nullptr && std::strcmp(tag, "POS_BRAKE_FWD") == 0) {
    out_code = "ERR_JOINT_POS_LIMIT";
    const double q_hi = otg::min_upper_excursion(q, v, a, -lim.a_lim, lim.j_lim);
    const double q_lo = otg::max_lower_excursion(q, v, a, lim.a_lim, lim.j_lim);
    oss << "forward braking excursion [" << q_lo << ", " << q_hi
        << "] rad exceeds joint limits [" << lim.q_min << ", " << lim.q_max << "] rad";
    out_reason = oss.str();
  } else {
    out_code = "ERR_JOINT_POS_LIMIT";
    const double q_hi = otg::min_upper_excursion(q, -v, a, -lim.a_lim, lim.j_lim);
    const double q_lo = otg::max_lower_excursion(q, -v, a, lim.a_lim, lim.j_lim);
    oss << "reverse-time braking excursion [" << q_lo << ", " << q_hi
        << "] rad exceeds joint limits [" << lim.q_min << ", " << lim.q_max << "] rad";
    out_reason = oss.str();
  }
  return false;
}

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
    auto test_relaxed = [&](double vm, double am, double jm) -> bool {
      DoFLimit rlim = lim;
      rlim.v_lim *= vm;
      rlim.a_lim *= am;
      rlim.j_lim *= jm;
      const otg::AxisLimits al = build_working_axis_limits(v0, a0, v1, a1, rlim);
      otg::AxisProfile p;
      double next_t = 0.0;
      return otg::solve_axis_bounded_at_duration({q0, v0, a0}, {q1, v1, a1}, al, T_target, p, next_t);
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
      otg::AxisLimits wide = build_working_axis_limits(v0, a0, v1, a1, lim);
      wide.q_min = -std::numeric_limits<double>::infinity();
      wide.q_max = std::numeric_limits<double>::infinity();
      otg::AxisProfile unc;
      if (otg::solve_axis_unconstrained_step2({q0, v0, a0}, {q1, v1, a1}, wide, T_target, unc)) {
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

struct SegmentSolution6DoF
{
  std::array<otg::AxisProfile, 6> dofs{};
  std::array<double, 6> ind_min_durations{};
  double duration = 0.0;
};

// Layer A (6-DOF): Native synchronized state-to-state solver with hard [min_pos, max_pos]
// and exact blocked-interval handling.
bool solve_segment_6dof(
  const JointPVASolver::SegmentInput & seg,
  const std::array<DoFLimit, 6> & lims,
  bool discretize_1ms,
  SegmentSolution6DoF & out_sol,
  std::string & err_code,
  std::string & err_msg)
{
  for (size_t j = 0; j < 6; ++j) {
    std::string code, reason;
    if (!validate_dof_state_diag(
          seg.q0[j], seg.v0[j], seg.a0[j], lims[j],
          /*check_forward=*/true, /*check_reverse=*/false, code, reason))
    {
      err_code = code;
      err_msg = "Joint" + std::to_string(j + 1) + " start state invalid: " + reason;
      return false;
    }
    if (!validate_dof_state_diag(
          seg.q1[j], seg.v1[j], seg.a1[j], lims[j],
          /*check_forward=*/false, /*check_reverse=*/true, code, reason))
    {
      err_code = code;
      err_msg = "Joint" + std::to_string(j + 1) + " target state invalid: " + reason;
      return false;
    }
  }

  otg::MultiDoFInput inp;
  inp.current.resize(6);
  inp.target.resize(6);
  inp.limits.resize(6);
  inp.control_interface = otg::ControlInterface::POSITION;
  inp.synchronization = otg::Synchronization::TIME;
  inp.discretization = discretize_1ms ?
    otg::DurationDiscretization::DISCRETE :
    otg::DurationDiscretization::CONTINUOUS;
  inp.dt_step = kDt;
  inp.max_duration = kMaxSegmentDurationSec;

  if (seg.explicit_dt.has_value()) {
    const double T_target = round_1ms(*seg.explicit_dt);
    if (T_target > kMaxSegmentDurationSec) {
      err_code = "ERR_PVA_DT";
      err_msg = "MovePVA explicit dt exceeds maximum segment duration";
      return false;
    }
    inp.exact_duration = T_target;
  }

  for (size_t j = 0; j < 6; ++j) {
    inp.current[j] = {seg.q0[j], seg.v0[j], seg.a0[j]};
    inp.target[j] = {seg.q1[j], seg.v1[j], seg.a1[j]};
    inp.limits[j] = build_working_axis_limits(seg.v0[j], seg.a0[j], seg.v1[j], seg.a1[j], lims[j]);
  }

  const otg::MultiDoFResult res = otg::solve_multidof(inp);
  if (!res.ok) {
    const size_t bad_j = (res.failed_dof >= 0 && res.failed_dof < 6)
      ? static_cast<size_t>(res.failed_dof) : 0;
    if (res.status == otg::SolveStatus::SEARCH_FAILED) {
      err_code = "ERR_PVA_SEARCH_FAILED";
      std::ostringstream oss;
      oss << "MovePVA numerical position-bounded OTG search failed on joint" << (bad_j + 1);
      if (seg.explicit_dt.has_value()) {
        oss << std::fixed << std::setprecision(4)
            << " at explicit dt=" << round_1ms(*seg.explicit_dt) << " s";
      }
      oss << " (could not find a boundary-constrained profile within search/depth budget)";
      err_msg = oss.str();
    } else if (seg.explicit_dt.has_value()) {
      const double T_target = round_1ms(*seg.explicit_dt);
      const double t_ref = std::isfinite(res.next_feasible_duration)
        ? res.next_feasible_duration
        : (bad_j < res.independent_min_durations.size() ? res.independent_min_durations[bad_j] : T_target);
      diagnose_dt_failure(
        bad_j, seg.q0[bad_j], seg.v0[bad_j], seg.a0[bad_j],
        seg.q1[bad_j], seg.v1[bad_j], seg.a1[bad_j],
        lims[bad_j], T_target, t_ref, err_code, err_msg);
    } else {
      err_code = "ERR_PVA_INFEASIBLE";
      err_msg = "MovePVA segment could not synchronize all 6 joints within limits (joint" +
        std::to_string(bad_j + 1) + ")";
    }
    return false;
  }

  for (size_t j = 0; j < 6; ++j) {
    out_sol.dofs[j] = res.profiles[j];
    out_sol.ind_min_durations[j] = res.independent_min_durations[j];
  }
  out_sol.duration = res.duration;
  return true;
}

// Fast 1-DOF native bounded minimum-time oracle (returns +inf if infeasible).
double eval_1dof_min_dur(
  double q0, double v0, double a0,
  double q1, double v1, double a1,
  const DoFLimit & lim)
{
  const otg::AxisLimits hard = to_hard_axis_limits(lim);
  if (!otg::validate_axis_state({q0, v0, a0}, hard, true, false) ||
      !otg::validate_axis_state({q1, v1, a1}, hard, false, true))
  {
    return std::numeric_limits<double>::infinity();
  }
  const otg::AxisLimits work = build_working_axis_limits(v0, a0, v1, a1, lim);
  otg::AxisProfile prof;
  if (!otg::solve_axis_bounded_min_time({q0, v0, a0}, {q1, v1, a1}, work, prof)) {
    return std::numeric_limits<double>::infinity();
  }
  return prof.duration;
}

// Fast 1-DOF oracle respecting optional explicit dt (for feasibility & Stage B2/B3).
double eval_1dof_seg_cost(
  double q0, double v0, double a0,
  double q1, double v1, double a1,
  const DoFLimit & lim,
  std::optional<double> explicit_dt)
{
  const otg::AxisLimits hard = to_hard_axis_limits(lim);
  if (!otg::validate_axis_state({q0, v0, a0}, hard, true, false) ||
      !otg::validate_axis_state({q1, v1, a1}, hard, false, true))
  {
    return std::numeric_limits<double>::infinity();
  }
  const otg::AxisLimits work = build_working_axis_limits(v0, a0, v1, a1, lim);
  if (explicit_dt.has_value()) {
    const double T_target = round_1ms(*explicit_dt);
    otg::AxisProfile prof;
    double next_t = 0.0;
    if (!otg::solve_axis_bounded_at_duration({q0, v0, a0}, {q1, v1, a1}, work, T_target, prof, next_t)) {
      return std::numeric_limits<double>::infinity();
    }
    return T_target;
  }
  otg::AxisProfile prof;
  if (!otg::solve_axis_bounded_min_time({q0, v0, a0}, {q1, v1, a1}, work, prof)) {
    return std::numeric_limits<double>::infinity();
  }
  return prof.duration;
}

// Exact 6-DOF continuous synchronized duration for segment m (blocked-interval aware).
double eval_6dof_sync_dur(
  const std::array<double, 6> & q0, const std::array<double, 6> & v0, const std::array<double, 6> & a0,
  const std::array<double, 6> & q1, const std::array<double, 6> & v1, const std::array<double, 6> & a1,
  const std::array<DoFLimit, 6> & lims,
  std::optional<double> explicit_dt,
  double * out_sum_ind = nullptr)
{
  otg::MultiDoFInput inp;
  inp.current.resize(6);
  inp.target.resize(6);
  inp.limits.resize(6);
  inp.control_interface = otg::ControlInterface::POSITION;
  inp.synchronization = otg::Synchronization::TIME;
  inp.discretization = otg::DurationDiscretization::CONTINUOUS;
  inp.dt_step = kDt;
  inp.max_duration = kMaxSegmentDurationSec;
  if (explicit_dt.has_value()) {
    inp.exact_duration = round_1ms(*explicit_dt);
  }
  for (size_t j = 0; j < 6; ++j) {
    const otg::AxisLimits hard = to_hard_axis_limits(lims[j]);
    if (!otg::validate_axis_state({q0[j], v0[j], a0[j]}, hard, true, false) ||
        !otg::validate_axis_state({q1[j], v1[j], a1[j]}, hard, false, true))
    {
      return std::numeric_limits<double>::infinity();
    }
    inp.current[j] = {q0[j], v0[j], a0[j]};
    inp.target[j] = {q1[j], v1[j], a1[j]};
    inp.limits[j] = build_working_axis_limits(v0[j], a0[j], v1[j], a1[j], lims[j]);
  }
  const otg::MultiDoFResult r = otg::solve_multidof(inp);
  if (!r.ok) {
    return std::numeric_limits<double>::infinity();
  }
  if (out_sum_ind != nullptr) {
    double s = 0.0;
    for (double d : r.independent_min_durations) {
      s += d;
    }
    *out_sum_ind = s;
  }
  return r.duration;
}

// Fits a natural C^2 cubic spline with clamped zero endpoint velocities (v0 = vM = 0)
// over knot durations h[0..M-1] for one DoF to produce physically consistent initial (v_k, a_k).
void fit_clamped_cubic_seed_1dof(
  const std::vector<double> & q,
  const std::vector<double> & h,
  std::vector<double> & out_v,
  std::vector<double> & out_a)
{
  const size_t M = h.size();
  out_v.assign(M + 1, 0.0);
  out_a.assign(M + 1, 0.0);
  if (M < 2) {
    return;
  }
  // Solve tridiagonal system for nodal second derivatives c[0..M] with clamped v[0] = v[M] = 0:
  //   (h[0]/3) c[0] + (h[0]/6) c[1] = (q[1] - q[0])/h[0]
  //   (h[k-1]/6) c[k-1] + ((h[k-1]+h[k])/3) c[k] + (h[k]/6) c[k+1] = (q[k+1]-q[k])/h[k] - (q[k]-q[k-1])/h[k-1]
  //   (h[M-1]/6) c[M-1] + (h[M-1]/3) c[M] = -(q[M] - q[M-1])/h[M-1]
  const size_t n = M + 1;
  std::vector<double> dl(n, 0.0), dm(n, 0.0), du(n, 0.0), rhs(n, 0.0);
  dm[0] = h[0] / 3.0;
  du[0] = h[0] / 6.0;
  rhs[0] = (q[1] - q[0]) / h[0];
  for (size_t k = 1; k < M; ++k) {
    dl[k] = h[k - 1] / 6.0;
    dm[k] = (h[k - 1] + h[k]) / 3.0;
    du[k] = h[k] / 6.0;
    rhs[k] = (q[k + 1] - q[k]) / h[k] - (q[k] - q[k - 1]) / h[k - 1];
  }
  dl[M] = h[M - 1] / 6.0;
  dm[M] = h[M - 1] / 3.0;
  rhs[M] = -(q[M] - q[M - 1]) / h[M - 1];

  // Thomas algorithm.
  for (size_t k = 1; k < n; ++k) {
    const double w = dl[k] / std::max(1e-12, dm[k - 1]);
    dm[k] -= w * du[k - 1];
    rhs[k] -= w * rhs[k - 1];
  }
  out_a[M] = rhs[M] / std::max(1e-12, dm[M]);
  for (size_t k = M; k-- > 0;) {
    out_a[k] = (rhs[k] - du[k] * out_a[k + 1]) / std::max(1e-12, dm[k]);
  }
  out_v[0] = 0.0;
  out_v[M] = 0.0;
  for (size_t k = 1; k < M; ++k) {
    out_v[k] = (q[k + 1] - q[k]) / h[k] - h[k] * (2.0 * out_a[k] + out_a[k + 1]) / 6.0;
  }
}

// Computes the exact closed interval [out_a_lo, out_a_hi] of free accelerations `a` that make
// (q, v_fixed, a) simultaneously forward- and reverse-time braking-feasible within hard limits
// (optionally tightened by headroom <= 1.0 on v/a/j caps).
// Exploits monotonicity: both U(a) = min_upper_excursion(q, |v|, a) and
// L(a) = max_lower_excursion(q, -|v|, a) are monotonically non-decreasing in `a`.
bool compute_feasible_acc_range_for_fixed_v(
  double q, double v, const DoFLimit & lim, double headroom,
  double & out_a_lo, double & out_a_hi,
  std::string * out_code = nullptr, std::string * out_reason = nullptr)
{
  const otg::AxisLimits hard = to_hard_axis_limits(lim);
  if (!std::isfinite(q) || !std::isfinite(v)) {
    if (out_code) *out_code = "ERR_NONFINITE_SAMPLE";
    if (out_reason) *out_reason = "non-finite waypoint position or velocity";
    return false;
  }
  if (q < lim.q_min - kPosTol || q > lim.q_max + kPosTol) {
    if (out_code) *out_code = "ERR_JOINT_POS_LIMIT";
    if (out_reason) {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(5)
          << "position=" << q << " rad outside [" << lim.q_min << ", " << lim.q_max << "] rad";
      *out_reason = oss.str();
    }
    return false;
  }
  if (std::abs(v) > lim.v_lim + 1e-9) {
    if (out_code) *out_code = "ERR_PVA_VEL_LIMIT";
    if (out_reason) {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(5)
          << "|v|=" << std::abs(v) << " rad/s exceeds scaled limit=" << lim.v_lim << " rad/s";
      *out_reason = oss.str();
    }
    return false;
  }

  const double v_cap_use = std::max(std::abs(v), lim.v_lim * headroom);
  const double a_cap_lim = lim.a_lim * headroom;
  const double j_use = lim.j_lim * headroom;
  const double a_vel_max = std::sqrt(std::max(0.0, 2.0 * j_use * std::max(0.0, v_cap_use - std::abs(v))));
  const double a_cap = std::min(a_cap_lim, a_vel_max);

  const double v_abs = std::abs(v);
  auto upper_ok = [&](double a) -> bool {
    return otg::min_upper_excursion(q, v_abs, a, -lim.a_lim, lim.j_lim) <= lim.q_max + kPosTol;
  };
  auto lower_ok = [&](double a) -> bool {
    return otg::max_lower_excursion(q, -v_abs, a, lim.a_lim, lim.j_lim) >= lim.q_min - kPosTol;
  };

  if (!upper_ok(-a_cap)) {
    if (out_code) *out_code = "ERR_JOINT_POS_LIMIT";
    if (out_reason) {
      const double q_hi = otg::min_upper_excursion(q, v_abs, -a_cap, -lim.a_lim, lim.j_lim);
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(5)
          << "even at maximum inward acceleration a=" << (-a_cap)
          << " rad/s^2, braking excursion reaches " << q_hi
          << " rad > upper limit " << lim.q_max << " rad";
      *out_reason = oss.str();
    }
    return false;
  }
  if (!lower_ok(a_cap)) {
    if (out_code) *out_code = "ERR_JOINT_POS_LIMIT";
    if (out_reason) {
      const double q_lo = otg::max_lower_excursion(q, -v_abs, a_cap, lim.a_lim, lim.j_lim);
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(5)
          << "even at maximum inward acceleration a=" << a_cap
          << " rad/s^2, braking excursion reaches " << q_lo
          << " rad < lower limit " << lim.q_min << " rad";
      *out_reason = oss.str();
    }
    return false;
  }

  // Upper feasible bound a_hi: largest a in [-a_cap, +a_cap] with upper_ok(a).
  double a_hi = a_cap;
  if (!upper_ok(a_cap)) {
    double lo = -a_cap, hi = a_cap;
    for (int it = 0; it < 50; ++it) {
      const double mid = 0.5 * (lo + hi);
      if (upper_ok(mid)) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    a_hi = lo;
  }

  // Lower feasible bound a_lo: smallest a in [-a_cap, +a_cap] with lower_ok(a).
  double a_lo = -a_cap;
  if (!lower_ok(-a_cap)) {
    double lo = -a_cap, hi = a_cap;
    for (int it = 0; it < 50; ++it) {
      const double mid = 0.5 * (lo + hi);
      if (lower_ok(mid)) {
        hi = mid;
      } else {
        lo = mid;
      }
    }
    a_lo = hi;
  }

  if (a_lo > a_hi + 1e-12) {
    if (out_code) *out_code = "ERR_JOINT_POS_LIMIT";
    if (out_reason) {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(5)
          << "no free acceleration satisfies both lower and upper braking bounds at q=" << q
          << " rad, v=" << v << " rad/s";
      *out_reason = oss.str();
    }
    return false;
  }
  if (a_lo > a_hi) {
    const double mid = 0.5 * (a_lo + a_hi);
    a_lo = a_hi = mid;
  }
  const double a_mid = 0.5 * (a_lo + a_hi);
  if (!otg::validate_axis_state({q, v, a_mid}, hard, true, true)) {
    if (out_code && out_reason) {
      validate_dof_state_diag(q, v, a_mid, lim, true, true, *out_code, *out_reason);
    }
    return false;
  }
  out_a_lo = a_lo;
  out_a_hi = a_hi;
  return true;
}

// Computes the exact symmetric feasible interval [-out_v_max, +out_v_max] of free velocities `v`
// that make (q, v, a_fixed) simultaneously forward- and reverse-time braking-feasible.
bool compute_feasible_vel_range_for_fixed_a(
  double q, double a, const DoFLimit & lim, double headroom,
  double & out_v_max,
  std::string * out_code = nullptr, std::string * out_reason = nullptr)
{
  const otg::AxisLimits hard = to_hard_axis_limits(lim);
  if (!std::isfinite(q) || !std::isfinite(a)) {
    if (out_code) *out_code = "ERR_NONFINITE_SAMPLE";
    if (out_reason) *out_reason = "non-finite waypoint position or acceleration";
    return false;
  }
  if (q < lim.q_min - kPosTol || q > lim.q_max + kPosTol) {
    if (out_code) *out_code = "ERR_JOINT_POS_LIMIT";
    if (out_reason) {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(5)
          << "position=" << q << " rad outside [" << lim.q_min << ", " << lim.q_max << "] rad";
      *out_reason = oss.str();
    }
    return false;
  }
  if (std::abs(a) > lim.a_lim + 1e-9) {
    if (out_code) *out_code = "ERR_PVA_ACC_LIMIT";
    if (out_reason) {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(5)
          << "|a|=" << std::abs(a) << " rad/s^2 exceeds scaled limit=" << lim.a_lim << " rad/s^2";
      *out_reason = oss.str();
    }
    return false;
  }

  // Because U(|v|, a) increases and L(-|v|, a) decreases monotonically with |v|, a feasible
  // velocity exists iff v = 0 is feasible under the hard limits.
  if (!otg::validate_axis_state({q, 0.0, a}, hard, true, true)) {
    if (out_code && out_reason) {
      validate_dof_state_diag(q, 0.0, a, lim, true, true, *out_code, *out_reason);
    }
    return false;
  }

  const double dv_a = (a * a) / (2.0 * (lim.j_lim * headroom));
  const double v_cap = std::max(0.0, lim.v_lim * headroom - dv_a);
  if (otg::validate_axis_state({q, v_cap, a}, hard, true, true)) {
    out_v_max = v_cap;
    return true;
  }
  double lo = 0.0, hi = v_cap;
  for (int it = 0; it < 50; ++it) {
    const double mid = 0.5 * (lo + hi);
    if (otg::validate_axis_state({q, mid, a}, hard, true, true)) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  out_v_max = lo;
  return true;
}

void project_node_respecting_masks(
  double q, bool v_fixed, bool a_fixed,
  double & v, double & a, const DoFLimit & lim)
{
  const otg::AxisLimits al = to_hard_axis_limits(lim);
  if (!v_fixed && !a_fixed) {
    otg::project_state_feasible(q, v, a, al, kHeadroom);
    return;
  }
  if (!v_fixed && a_fixed) {
    // Keep pinned a fixed; clamp v into the exact feasible symmetric velocity interval.
    double v_max = 0.0;
    if (!compute_feasible_vel_range_for_fixed_a(q, a, lim, kHeadroom, v_max)) {
      if (!compute_feasible_vel_range_for_fixed_a(q, a, lim, 1.0, v_max)) {
        v = 0.0;
        return;
      }
    }
    v = std::clamp(v, -v_max, v_max);
    return;
  }
  if (v_fixed && !a_fixed) {
    // Keep pinned v fixed; clamp a into the exact feasible acceleration interval [a_lo, a_hi].
    // Do NOT assume scaling toward zero leads toward feasibility: near a position boundary,
    // a = 0 can be infeasible while an inward nonzero acceleration is strictly feasible.
    double a_lo = 0.0, a_hi = 0.0;
    if (!compute_feasible_acc_range_for_fixed_v(q, v, lim, kHeadroom, a_lo, a_hi)) {
      if (!compute_feasible_acc_range_for_fixed_v(q, v, lim, 1.0, a_lo, a_hi)) {
        return;
      }
    }
    a = std::clamp(a, a_lo, a_hi);
  }
}

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
  const std::array<lite6_kinematics::JointLimit, 6> & limits,
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
  const std::array<lite6_kinematics::JointLimit, 6> & limits,
  double vel_scale,
  double acc_scale,
  double jerk_scale,
  MultiWaypointResult & out_result,
  SourceLocation & err_loc,
  std::string & err_code,
  std::string & err_msg,
  const std::function<bool()> & is_cancelled)
{
  out_result = MultiWaypointResult{};
  auto check_cancel = [&](const SourceLocation & loc) -> bool {
    if (is_cancelled && is_cancelled()) {
      err_loc = loc;
      err_code = "ERR_CANCELLED";
      err_msg = "Motion cancelled by supervisor";
      return true;
    }
    return false;
  };
  if (waypoints.empty()) {
    err_code = "ERR_EMPTY_PVA";
    err_msg = "MovePVA requires at least one waypoint";
    return false;
  }
  if (check_cancel(waypoints.front().loc)) {
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

    for (size_t j = 0; j < 6; ++j) {
      if (!std::isfinite(wp.joint_pos[j]) ||
          wp.joint_pos[j] < lims[j].q_min - lite6_kinematics::tolerance::kJointPosition ||
          wp.joint_pos[j] > lims[j].q_max + lite6_kinematics::tolerance::kJointPosition)
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
      for (size_t j = 0; j < 6; ++j) {
        std::string code, reason;
        bool ok = true;
        if (v_fixed[k + 1] && a_fixed[k + 1]) {
          ok = validate_dof_state_diag(
            q_nodes[k + 1][j], v_nodes[k + 1][j], a_nodes[k + 1][j], lims[j],
            /*check_forward=*/true, /*check_reverse=*/true, code, reason);
        } else if (v_fixed[k + 1] && !a_fixed[k + 1]) {
          double a_lo = 0.0, a_hi = 0.0;
          ok = compute_feasible_acc_range_for_fixed_v(
            q_nodes[k + 1][j], v_nodes[k + 1][j], lims[j], 1.0,
            a_lo, a_hi, &code, &reason);
        } else if (!v_fixed[k + 1] && a_fixed[k + 1]) {
          double v_max = 0.0;
          ok = compute_feasible_vel_range_for_fixed_a(
            q_nodes[k + 1][j], a_nodes[k + 1][j], lims[j], 1.0,
            v_max, &code, &reason);
        }
        if (!ok) {
          err_loc = wp.loc;
          err_code = code;
          err_msg = "MovePVA waypoint " + std::to_string(k + 1) +
            " joint" + std::to_string(j + 1) + ": " + reason;
          return false;
        }
      }
    }
  }

  bool has_any_free = false;
  for (size_t k = 1; k < M; ++k) {
    if (!v_fixed[k] || !a_fixed[k]) {
      has_any_free = true;
      break;
    }
  }

  // 2. Layer B: Deterministic 4-stage multi-waypoint (v_k, a_k) optimization.
  if (has_any_free) {
    // Exact 6-DOF synchronized total objective (with tiny individual-duration tie-breaker).
    auto eval_total_sync = [&](
      const std::vector<std::array<double, 6>> & vn,
      const std::vector<std::array<double, 6>> & an,
      std::vector<double> * out_seg_t = nullptr) -> double
    {
      if (out_seg_t != nullptr) {
        out_seg_t->assign(M, 0.0);
      }
      double total = 0.0;
      double sum_ind = 0.0;
      for (size_t m = 0; m < M; ++m) {
        double si = 0.0;
        const double tm = eval_6dof_sync_dur(
          q_nodes[m], vn[m], an[m], q_nodes[m + 1], vn[m + 1], an[m + 1],
          lims, seg_dt[m], &si);
        if (!std::isfinite(tm)) {
          return std::numeric_limits<double>::infinity();
        }
        if (out_seg_t != nullptr) {
          (*out_seg_t)[m] = tm;
        }
        total += tm;
        sum_ind += si;
      }
      return total + 1e-4 * sum_ind;
    };

    // Estimate segment durations from rest-to-rest 6-DOF solves (or explicit dt).
    std::vector<double> est_dt(M, 0.2);
    for (size_t m = 0; m < M; ++m) {
      if (seg_dt[m].has_value()) {
        est_dt[m] = std::max(0.01, round_1ms(*seg_dt[m]));
      } else {
        double t_est = 0.02;
        for (size_t j = 0; j < 6; ++j) {
          const double tj = eval_1dof_min_dur(
            q_nodes[m][j], 0.0, 0.0, q_nodes[m + 1][j], 0.0, 0.0, lims[j]);
          if (std::isfinite(tj)) {
            t_est = std::max(t_est, tj);
          }
        }
        est_dt[m] = t_est;
      }
    }

    // ---- Stage B1: Multi-seed deterministic initialization ----
    // Seed 1: Natural C^2 cubic spline (v_k, a_k).
    auto v_spline = v_nodes;
    auto a_spline = a_nodes;
    for (size_t j = 0; j < 6; ++j) {
      std::vector<double> qj(M + 1), vj, aj;
      for (size_t k = 0; k <= M; ++k) {
        qj[k] = q_nodes[k][j];
      }
      fit_clamped_cubic_seed_1dof(qj, est_dt, vj, aj);
      for (size_t k = 1; k < M; ++k) {
        if (!v_fixed[k]) {
          v_spline[k][j] = vj[k];
        }
        if (!a_fixed[k]) {
          a_spline[k][j] = aj[k];
        }
        project_node_respecting_masks(
          q_nodes[k][j], v_fixed[k], a_fixed[k], v_spline[k][j], a_spline[k][j], lims[j]);
      }
    }

    // Seed 2: Direction-preserving time-weighted Catmull-Rom velocity + projected free acceleration.
    auto v_catmull = v_nodes;
    auto a_catmull = a_nodes;
    for (size_t k = 1; k < M; ++k) {
      if (!a_fixed[k]) {
        a_catmull[k].fill(0.0);
      }
      const double tm = est_dt[k - 1];
      const double tp = est_dt[k];
      for (size_t j = 0; j < 6; ++j) {
        if (!v_fixed[k]) {
          const double sm = (q_nodes[k][j] - q_nodes[k - 1][j]) / tm;
          const double sp = (q_nodes[k + 1][j] - q_nodes[k][j]) / tp;
          v_catmull[k][j] = (sm * sp > 0.0) ? ((tp * sm + tm * sp) / (tm + tp)) : 0.0;
        }
        if (!v_fixed[k] || !a_fixed[k]) {
          project_node_respecting_masks(
            q_nodes[k][j], v_fixed[k], a_fixed[k], v_catmull[k][j], a_catmull[k][j], lims[j]);
        }
      }
    }

    // Seed 3: Zero free states projected into the exact feasible range.
    auto v_zero = v_nodes;
    auto a_zero = a_nodes;
    for (size_t k = 1; k < M; ++k) {
      if (!v_fixed[k]) {
        v_zero[k].fill(0.0);
      }
      if (!a_fixed[k]) {
        a_zero[k].fill(0.0);
      }
      if (!v_fixed[k] || !a_fixed[k]) {
        for (size_t j = 0; j < 6; ++j) {
          project_node_respecting_masks(
            q_nodes[k][j], v_fixed[k], a_fixed[k], v_zero[k][j], a_zero[k][j], lims[j]);
        }
      }
    }

    if (check_cancel(waypoints.front().loc)) {
      return false;
    }

    double best_sync_obj = eval_total_sync(v_spline, a_spline);
    v_nodes = v_spline;
    a_nodes = a_spline;

    const double cat_obj = eval_total_sync(v_catmull, a_catmull);
    if (cat_obj < best_sync_obj) {
      best_sync_obj = cat_obj;
      v_nodes = v_catmull;
      a_nodes = a_catmull;
    }
    const double zero_obj = eval_total_sync(v_zero, a_zero);
    if (zero_obj < best_sync_obj) {
      best_sync_obj = zero_obj;
      v_nodes = v_zero;
      a_nodes = a_zero;
    }

    // Phase-I deterministic feasibility restoration when tight explicit dt makes all seeds infeasible.
    if (!std::isfinite(best_sync_obj)) {
      auto violation_cost = [&](
        const std::vector<std::array<double, 6>> & vn,
        const std::vector<std::array<double, 6>> & an) -> double
      {
        double vio = 0.0;
        for (size_t m = 0; m < M; ++m) {
          const double target = seg_dt[m].has_value() ? round_1ms(*seg_dt[m]) : kMaxSegmentDurationSec;
          for (size_t j = 0; j < 6; ++j) {
            const double tj = eval_1dof_min_dur(
              q_nodes[m][j], vn[m][j], an[m][j],
              q_nodes[m + 1][j], vn[m + 1][j], an[m + 1][j], lims[j]);
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
      a_nodes = a_catmull;
      double cur_vio = violation_cost(v_nodes, a_nodes);
      const double spl_vio = violation_cost(v_spline, a_spline);
      if (spl_vio < cur_vio) {
        cur_vio = spl_vio;
        v_nodes = v_spline;
        a_nodes = a_spline;
      }
      const double zr_vio = violation_cost(v_zero, a_zero);
      if (zr_vio < cur_vio) {
        cur_vio = zr_vio;
        v_nodes = v_zero;
        a_nodes = a_zero;
      }

      if (std::isfinite(cur_vio) && cur_vio > 0.0) {
        for (int sweep = 0; sweep < 10 && cur_vio > 1e-6; ++sweep) {
          if (check_cancel(waypoints.front().loc)) {
            return false;
          }
          const double scale = std::pow(0.55, sweep);
          for (size_t k = 1; k < M; ++k) {
            for (size_t j = 0; j < 6; ++j) {
              if (!v_fixed[k]) {
                const double dv = 0.4 * scale * lims[j].v_lim * kHeadroom;
                for (double sgn : {1.0, -1.0}) {
                  const double ov = v_nodes[k][j];
                  const double oa = a_nodes[k][j];
                  v_nodes[k][j] = ov + sgn * dv;
                  project_node_respecting_masks(
                    q_nodes[k][j], v_fixed[k], a_fixed[k], v_nodes[k][j], a_nodes[k][j], lims[j]);
                  const double nv = violation_cost(v_nodes, a_nodes);
                  if (nv < cur_vio - 1e-9) {
                    cur_vio = nv;
                    break;
                  }
                  v_nodes[k][j] = ov;
                  a_nodes[k][j] = oa;
                }
              }
              if (!a_fixed[k]) {
                const double da = 0.4 * scale * lims[j].a_lim * kHeadroom;
                for (double sgn : {1.0, -1.0}) {
                  const double ov = v_nodes[k][j];
                  const double oa = a_nodes[k][j];
                  a_nodes[k][j] = oa + sgn * da;
                  project_node_respecting_masks(
                    q_nodes[k][j], v_fixed[k], a_fixed[k], v_nodes[k][j], a_nodes[k][j], lims[j]);
                  const double nv = violation_cost(v_nodes, a_nodes);
                  if (nv < cur_vio - 1e-9) {
                    cur_vio = nv;
                    break;
                  }
                  v_nodes[k][j] = ov;
                  a_nodes[k][j] = oa;
                }
              }
            }
          }
        }
        best_sync_obj = eval_total_sync(v_nodes, a_nodes);
      }
    }

    // ---- Stage B2 & B3: Coupled 6-DOF Smooth-Max Projected L-BFGS Continuation ----
    if (std::isfinite(best_sync_obj)) {
      // Index mapping of all free scalar coordinates (normalized by v_lim / a_lim).
      struct FreeVar
      {
        size_t node_k = 0;
        size_t dof_j = 0;
        bool is_acc = false;
        double scale = 1.0;
      };
      std::vector<FreeVar> vars;
      vars.reserve(12 * (M - 1));
      for (size_t k = 1; k < M; ++k) {
        for (size_t j = 0; j < 6; ++j) {
          if (!v_fixed[k]) {
            vars.push_back({k, j, false, a_fixed[k] ? lims[j].v_lim : (lims[j].v_lim * kHeadroom)});
          }
          if (!a_fixed[k]) {
            vars.push_back({k, j, true, v_fixed[k] ? lims[j].a_lim : (lims[j].a_lim * kHeadroom)});
          }
        }
      }
      const size_t D = vars.size();

      auto pack_x = [&](
        const std::vector<std::array<double, 6>> & vn,
        const std::vector<std::array<double, 6>> & an) -> std::vector<double>
      {
        std::vector<double> x(D, 0.0);
        for (size_t idx = 0; idx < D; ++idx) {
          const auto & fv = vars[idx];
          x[idx] = (fv.is_acc ? an[fv.node_k][fv.dof_j] : vn[fv.node_k][fv.dof_j]) / fv.scale;
        }
        return x;
      };

      auto unpack_and_project = [&](
        const std::vector<double> & x,
        std::vector<std::array<double, 6>> & vn,
        std::vector<std::array<double, 6>> & an)
      {
        vn = v_nodes;
        an = a_nodes;
        for (size_t idx = 0; idx < D; ++idx) {
          const auto & fv = vars[idx];
          const double val = std::clamp(x[idx], -1.0, 1.0) * fv.scale;
          if (fv.is_acc) {
            an[fv.node_k][fv.dof_j] = val;
          } else {
            vn[fv.node_k][fv.dof_j] = val;
          }
        }
        for (size_t k = 1; k < M; ++k) {
          if (v_fixed[k] && a_fixed[k]) {
            continue;
          }
          for (size_t j = 0; j < 6; ++j) {
            project_node_respecting_masks(
              q_nodes[k][j], v_fixed[k], a_fixed[k], vn[k][j], an[k][j], lims[j]);
          }
        }
      };

      // Smooth-max LogSumExp objective on 1-DOF exact bounded durations T_mat[m][j].
      // For explicit dt segments, checks feasibility and adds exact target duration + small tie-breaker.
      auto eval_smooth_obj = [&](
        const std::vector<std::array<double, 6>> & vn,
        const std::vector<std::array<double, 6>> & an,
        double beta,
        std::vector<std::array<double, 6>> & out_tmat) -> double
      {
        out_tmat.assign(M, std::array<double, 6>{});
        double total = 0.0;
        for (size_t m = 0; m < M; ++m) {
          double seg_max = 0.0;
          double sum_j = 0.0;
          for (size_t j = 0; j < 6; ++j) {
            const double tj = eval_1dof_seg_cost(
              q_nodes[m][j], vn[m][j], an[m][j],
              q_nodes[m + 1][j], vn[m + 1][j], an[m + 1][j],
              lims[j], seg_dt[m]);
            if (!std::isfinite(tj)) {
              return std::numeric_limits<double>::infinity();
            }
            const double t_ind = seg_dt[m].has_value()
              ? eval_1dof_min_dur(
                  q_nodes[m][j], vn[m][j], an[m][j],
                  q_nodes[m + 1][j], vn[m + 1][j], an[m + 1][j], lims[j])
              : tj;
            out_tmat[m][j] = t_ind;
            seg_max = std::max(seg_max, t_ind);
            sum_j += t_ind;
          }
          if (seg_dt[m].has_value()) {
            total += round_1ms(*seg_dt[m]) + 1e-4 * sum_j;
          } else {
            double exp_sum = 0.0;
            for (size_t j = 0; j < 6; ++j) {
              exp_sum += std::exp(beta * (out_tmat[m][j] - seg_max));
            }
            const double smax = seg_max + std::log(exp_sum) / beta;
            total += smax + 1e-4 * sum_j;
          }
        }
        return total;
      };

      // Stage B2: Per-DoF independent chain pre-optimization (fast 1-DOF L-BFGS warm start).
      // Only attempted if no explicit dt constraints exist (so per-DoF solves stay unconstrained in time).
      bool any_explicit_dt = false;
      for (const auto & d : seg_dt) {
        if (d.has_value()) {
          any_explicit_dt = true;
          break;
        }
      }
      if (!any_explicit_dt && M >= 2) {
        auto v_chain = v_nodes;
        auto a_chain = a_nodes;
        for (size_t j = 0; j < 6; ++j) {
          std::vector<size_t> j_vars;
          for (size_t idx = 0; idx < D; ++idx) {
            if (vars[idx].dof_j == j) {
              j_vars.push_back(idx);
            }
          }
          if (j_vars.empty()) {
            continue;
          }
          const size_t dj = j_vars.size();
          std::vector<double> xj(dj, 0.0);
          for (size_t p = 0; p < dj; ++p) {
            const auto & fv = vars[j_vars[p]];
            xj[p] = (fv.is_acc ? a_chain[fv.node_k][j] : v_chain[fv.node_k][j]) / fv.scale;
          }
          auto eval_chain_j = [&](const std::vector<double> & xv,
                                  std::vector<double> & vj_out, std::vector<double> & aj_out,
                                  std::vector<double> & seg_out) -> double
          {
            vj_out.resize(M + 1);
            aj_out.resize(M + 1);
            for (size_t k = 0; k <= M; ++k) {
              vj_out[k] = v_chain[k][j];
              aj_out[k] = a_chain[k][j];
            }
            for (size_t p = 0; p < dj; ++p) {
              const auto & fv = vars[j_vars[p]];
              const double val = std::clamp(xv[p], -1.0, 1.0) * fv.scale;
              if (fv.is_acc) {
                aj_out[fv.node_k] = val;
              } else {
                vj_out[fv.node_k] = val;
              }
            }
            for (size_t k = 1; k < M; ++k) {
              project_node_respecting_masks(
                q_nodes[k][j], v_fixed[k], a_fixed[k], vj_out[k], aj_out[k], lims[j]);
            }
            seg_out.assign(M, 0.0);
            double tot = 0.0;
            for (size_t m = 0; m < M; ++m) {
              const double tm = eval_1dof_min_dur(
                q_nodes[m][j], vj_out[m], aj_out[m],
                q_nodes[m + 1][j], vj_out[m + 1], aj_out[m + 1], lims[j]);
              if (!std::isfinite(tm)) {
                return std::numeric_limits<double>::infinity();
              }
              seg_out[m] = tm;
              tot += tm;
            }
            return tot;
          };

          std::vector<double> vj_c, aj_c, seg_c;
          double fj = eval_chain_j(xj, vj_c, aj_c, seg_c);
          if (!std::isfinite(fj)) {
            continue;
          }
          auto eval_grad_j = [&](
            const std::vector<double> & vj_in,
            const std::vector<double> & aj_in,
            const std::vector<double> & seg_in,
            std::vector<double> & g_out)
          {
            g_out.assign(dj, 0.0);
            constexpr double kFD = 1e-4;
            for (size_t p = 0; p < dj; ++p) {
              const auto & fv = vars[j_vars[p]];
              const size_t k = fv.node_k;
              const double base = fv.is_acc ? aj_in[k] : vj_in[k];
              const double h = kFD * fv.scale;
              auto eval_pair = [&](double val) -> double {
                const double vt = fv.is_acc ? vj_in[k] : val;
                const double at = fv.is_acc ? val : aj_in[k];
                const double t1 = eval_1dof_min_dur(
                  q_nodes[k - 1][j], vj_in[k - 1], aj_in[k - 1], q_nodes[k][j], vt, at, lims[j]);
                if (!std::isfinite(t1)) {
                  return std::numeric_limits<double>::infinity();
                }
                const double t2 = eval_1dof_min_dur(
                  q_nodes[k][j], vt, at, q_nodes[k + 1][j], vj_in[k + 1], aj_in[k + 1], lims[j]);
                return std::isfinite(t2) ? (t1 + t2) : std::numeric_limits<double>::infinity();
              };
              const double fp = eval_pair(base + h);
              const double fm = eval_pair(base - h);
              const double f0 = seg_in[k - 1] + seg_in[k];
              if (std::isfinite(fp) && std::isfinite(fm)) {
                g_out[p] = ((fp - fm) / (2.0 * h)) * fv.scale;
              } else if (std::isfinite(fp)) {
                g_out[p] = ((fp - f0) / h) * fv.scale;
              } else if (std::isfinite(fm)) {
                g_out[p] = ((f0 - fm) / h) * fv.scale;
              }
            }
          };

          std::vector<std::vector<double>> s_hist, y_hist;
          std::vector<double> rho_hist;
          std::vector<double> g;
          eval_grad_j(vj_c, aj_c, seg_c, g);
          for (int it = 0; it < 10; ++it) {
            if (check_cancel(waypoints.front().loc)) {
              return false;
            }
            double gnorm = 0.0;
            for (double gv : g) {
              gnorm = std::max(gnorm, std::abs(gv));
            }
            if (gnorm < 1e-5) {
              break;
            }

            // Two-loop L-BFGS direction.
            std::vector<double> dir = g;
            const size_t nh = s_hist.size();
            std::vector<double> alphas(nh, 0.0);
            for (size_t r = nh; r-- > 0;) {
              double dot = 0.0;
              for (size_t p = 0; p < dj; ++p) {
                dot += s_hist[r][p] * dir[p];
              }
              alphas[r] = rho_hist[r] * dot;
              for (size_t p = 0; p < dj; ++p) {
                dir[p] -= alphas[r] * y_hist[r][p];
              }
            }
            double gamma = 1.0 / std::max(1.0, gnorm);
            if (nh > 0) {
              double sy = 0.0, yy = 0.0;
              for (size_t p = 0; p < dj; ++p) {
                sy += s_hist.back()[p] * y_hist.back()[p];
                yy += y_hist.back()[p] * y_hist.back()[p];
              }
              if (yy > 1e-16 && sy > 1e-16) {
                gamma = sy / yy;
              }
            }
            for (size_t p = 0; p < dj; ++p) {
              dir[p] *= -gamma;
            }
            for (size_t r = 0; r < nh; ++r) {
              double dot = 0.0;
              for (size_t p = 0; p < dj; ++p) {
                dot += y_hist[r][p] * dir[p];
              }
              const double beta_r = rho_hist[r] * dot;
              for (size_t p = 0; p < dj; ++p) {
                dir[p] += s_hist[r][p] * (alphas[r] - beta_r);
              }
            }

            double ddotg = 0.0;
            for (size_t p = 0; p < dj; ++p) {
              ddotg += dir[p] * g[p];
            }
            if (!(ddotg < 0.0)) {
              for (size_t p = 0; p < dj; ++p) {
                dir[p] = -g[p] / std::max(1.0, gnorm);
              }
            }

            double step = 1.0;
            bool acc = false;
            std::vector<double> x_new(dj), vj_n, aj_n, seg_n;
            double f_new = fj;
            for (int ls = 0; ls < 8; ++ls) {
              for (size_t p = 0; p < dj; ++p) {
                x_new[p] = std::clamp(xj[p] + step * dir[p], -1.0, 1.0);
              }
              f_new = eval_chain_j(x_new, vj_n, aj_n, seg_n);
              if (f_new < fj - 1e-7) {
                acc = true;
                break;
              }
              step *= 0.4;
            }
            if (!acc) {
              break;
            }
            // Re-pack projected coordinates into x_new and compute matching gradient g_new.
            for (size_t p = 0; p < dj; ++p) {
              const auto & fv = vars[j_vars[p]];
              x_new[p] = (fv.is_acc ? aj_n[fv.node_k] : vj_n[fv.node_k]) / fv.scale;
            }
            std::vector<double> g_new;
            eval_grad_j(vj_n, aj_n, seg_n, g_new);
            std::vector<double> sv(dj), yv(dj);
            double sy = 0.0;
            for (size_t p = 0; p < dj; ++p) {
              sv[p] = x_new[p] - xj[p];
              yv[p] = g_new[p] - g[p];
              sy += sv[p] * yv[p];
            }
            if (sy > 1e-12) {
              if (s_hist.size() >= 5) {
                s_hist.erase(s_hist.begin());
                y_hist.erase(y_hist.begin());
                rho_hist.erase(rho_hist.begin());
              }
              s_hist.push_back(std::move(sv));
              y_hist.push_back(std::move(yv));
              rho_hist.push_back(1.0 / sy);
            }
            g.swap(g_new);
            xj.swap(x_new);
            vj_c.swap(vj_n);
            aj_c.swap(aj_n);
            seg_c.swap(seg_n);
            const double rel = (fj - f_new) / std::max(fj, 1e-6);
            fj = f_new;
            if (rel < 1e-4) {
              break;
            }
          }
          for (size_t k = 1; k < M; ++k) {
            v_chain[k][j] = vj_c[k];
            a_chain[k][j] = aj_c[k];
          }
        }
        const double chain_sync = eval_total_sync(v_chain, a_chain);
        if (chain_sync < best_sync_obj) {
          best_sync_obj = chain_sync;
          v_nodes = std::move(v_chain);
          a_nodes = std::move(a_chain);
        }
      }

      // Stage B3: Coupled 6-DOF Smooth-Max Continuation via Projected L-BFGS.
      // Sparse gradient: perturbing (k, j) evaluates only 2 scalar 1-DOF calls!
      auto v_lbfgs = v_nodes;
      auto a_lbfgs = a_nodes;
      std::vector<double> x = pack_x(v_lbfgs, a_lbfgs);
      constexpr std::array<double, 2> kBetas{{25.0, 150.0}};

      for (double beta : kBetas) {
        std::vector<std::array<double, 6>> tmat;
        double f_cur = eval_smooth_obj(v_lbfgs, a_lbfgs, beta, tmat);
        if (!std::isfinite(f_cur)) {
          break;
        }
        std::vector<std::vector<double>> s_hist, y_hist;
        std::vector<double> rho_hist;
        std::vector<double> g_prev;

        auto compute_sparse_grad = [&](
          const std::vector<std::array<double, 6>> & vn,
          const std::vector<std::array<double, 6>> & an,
          const std::vector<std::array<double, 6>> & tm,
          std::vector<double> & g_out)
        {
          g_out.assign(D, 0.0);
          constexpr double kFD = 1e-4;
          auto seg_contrib = [&](size_t m, size_t j_mod, double t_mod) -> double {
            if (seg_dt[m].has_value()) {
              return 1e-4 * t_mod;
            }
            double mx = t_mod;
            for (size_t d = 0; d < 6; ++d) {
              if (d != j_mod) {
                mx = std::max(mx, tm[m][d]);
              }
            }
            double es = std::exp(beta * (t_mod - mx));
            double sum_j = t_mod;
            for (size_t d = 0; d < 6; ++d) {
              if (d != j_mod) {
                es += std::exp(beta * (tm[m][d] - mx));
                sum_j += tm[m][d];
              }
            }
            return mx + std::log(es) / beta + 1e-4 * sum_j;
          };

          for (size_t idx = 0; idx < D; ++idx) {
            const auto & fv = vars[idx];
            const size_t k = fv.node_k;
            const size_t j = fv.dof_j;
            const double base = fv.is_acc ? an[k][j] : vn[k][j];
            const double h = kFD * fv.scale;

            auto eval_local = [&](double val) -> double {
              const double vt = fv.is_acc ? vn[k][j] : val;
              const double at = fv.is_acc ? val : an[k][j];
              const double c1 = eval_1dof_seg_cost(
                q_nodes[k - 1][j], vn[k - 1][j], an[k - 1][j],
                q_nodes[k][j], vt, at, lims[j], seg_dt[k - 1]);
              if (!std::isfinite(c1)) {
                return std::numeric_limits<double>::infinity();
              }
              const double c2 = eval_1dof_seg_cost(
                q_nodes[k][j], vt, at,
                q_nodes[k + 1][j], vn[k + 1][j], an[k + 1][j], lims[j], seg_dt[k]);
              if (!std::isfinite(c2)) {
                return std::numeric_limits<double>::infinity();
              }
              const double t1 = seg_dt[k - 1].has_value()
                ? eval_1dof_min_dur(q_nodes[k - 1][j], vn[k - 1][j], an[k - 1][j], q_nodes[k][j], vt, at, lims[j])
                : c1;
              const double t2 = seg_dt[k].has_value()
                ? eval_1dof_min_dur(q_nodes[k][j], vt, at, q_nodes[k + 1][j], vn[k + 1][j], an[k + 1][j], lims[j])
                : c2;
              return seg_contrib(k - 1, j, t1) + seg_contrib(k, j, t2);
            };

            const double fp = eval_local(base + h);
            const double fm = eval_local(base - h);
            const double f0 = seg_contrib(k - 1, j, tm[k - 1][j]) + seg_contrib(k, j, tm[k][j]);
            if (std::isfinite(fp) && std::isfinite(fm)) {
              g_out[idx] = ((fp - fm) / (2.0 * h)) * fv.scale;
            } else if (std::isfinite(fp)) {
              g_out[idx] = ((fp - f0) / h) * fv.scale;
            } else if (std::isfinite(fm)) {
              g_out[idx] = ((f0 - fm) / h) * fv.scale;
            }
          }
        };

        std::vector<double> g(D, 0.0);
        compute_sparse_grad(v_lbfgs, a_lbfgs, tmat, g);

        for (int it = 0; it < 16; ++it) {
          if (check_cancel(waypoints.front().loc)) {
            return false;
          }
          double gnorm = 0.0;
          for (double gv : g) {
            gnorm = std::max(gnorm, std::abs(gv));
          }
          if (gnorm < 1e-5) {
            break;
          }

          // Two-loop L-BFGS recursion (memory m = 7).
          std::vector<double> dir = g;
          const size_t nh = s_hist.size();
          std::vector<double> alphas(nh, 0.0);
          for (size_t r = nh; r-- > 0;) {
            double dot = 0.0;
            for (size_t idx = 0; idx < D; ++idx) {
              dot += s_hist[r][idx] * dir[idx];
            }
            alphas[r] = rho_hist[r] * dot;
            for (size_t idx = 0; idx < D; ++idx) {
              dir[idx] -= alphas[r] * y_hist[r][idx];
            }
          }
          double gamma = 0.5 / std::max(0.5, gnorm);
          if (nh > 0) {
            double sy = 0.0, yy = 0.0;
            for (size_t idx = 0; idx < D; ++idx) {
              sy += s_hist.back()[idx] * y_hist.back()[idx];
              yy += y_hist.back()[idx] * y_hist.back()[idx];
            }
            if (yy > 1e-16 && sy > 1e-16) {
              gamma = std::clamp(sy / yy, 1e-4, 10.0);
            }
          }
          for (size_t idx = 0; idx < D; ++idx) {
            dir[idx] *= -gamma;
          }
          for (size_t r = 0; r < nh; ++r) {
            double dot = 0.0;
            for (size_t idx = 0; idx < D; ++idx) {
              dot += y_hist[r][idx] * dir[idx];
            }
            const double beta_r = rho_hist[r] * dot;
            for (size_t idx = 0; idx < D; ++idx) {
              dir[idx] += s_hist[r][idx] * (alphas[r] - beta_r);
            }
          }

          double ddotg = 0.0;
          double max_dir = 0.0;
          for (size_t idx = 0; idx < D; ++idx) {
            ddotg += dir[idx] * g[idx];
            max_dir = std::max(max_dir, std::abs(dir[idx]));
          }
          if (!(ddotg < 0.0)) {
            s_hist.clear();
            y_hist.clear();
            rho_hist.clear();
            max_dir = 0.0;
            for (size_t idx = 0; idx < D; ++idx) {
              dir[idx] = -g[idx] / std::max(1.0, gnorm);
              max_dir = std::max(max_dir, std::abs(dir[idx]));
            }
          }
          if (max_dir > 0.6) {
            const double cap = 0.6 / max_dir;
            for (double & dv : dir) {
              dv *= cap;
            }
          }

          double step = 1.0;
          bool accepted = false;
          std::vector<double> x_try(D);
          std::vector<std::array<double, 6>> v_try, a_try;
          std::vector<std::array<double, 6>> tm_try;
          double f_try = f_cur;

          for (int ls = 0; ls < 9; ++ls) {
            for (size_t idx = 0; idx < D; ++idx) {
              x_try[idx] = std::clamp(x[idx] + step * dir[idx], -1.0, 1.0);
            }
            unpack_and_project(x_try, v_try, a_try);
            f_try = eval_smooth_obj(v_try, a_try, beta, tm_try);
            if (f_try < f_cur - 1e-7) {
              accepted = true;
              break;
            }
            step *= 0.4;
          }
          if (!accepted) {
            break;
          }

          const std::vector<double> x_new = pack_x(v_try, a_try);
          std::vector<double> g_new(D, 0.0);
          compute_sparse_grad(v_try, a_try, tm_try, g_new);

          std::vector<double> sv(D), yv(D);
          double sy = 0.0;
          for (size_t idx = 0; idx < D; ++idx) {
            sv[idx] = x_new[idx] - x[idx];
            yv[idx] = g_new[idx] - g[idx];
            sy += sv[idx] * yv[idx];
          }
          if (sy > 1e-12) {
            if (s_hist.size() >= 7) {
              s_hist.erase(s_hist.begin());
              y_hist.erase(y_hist.begin());
              rho_hist.erase(rho_hist.begin());
            }
            s_hist.push_back(std::move(sv));
            y_hist.push_back(std::move(yv));
            rho_hist.push_back(1.0 / sy);
          }

          const double rel = (f_cur - f_try) / std::max(f_cur, 1e-6);
          x = x_new;
          v_lbfgs.swap(v_try);
          a_lbfgs.swap(a_try);
          tmat.swap(tm_try);
          g.swap(g_new);
          f_cur = f_try;
          if (rel < 1e-5) {
            break;
          }
        }
      }

      const double lbfgs_sync = eval_total_sync(v_lbfgs, a_lbfgs);
      if (lbfgs_sync < best_sync_obj) {
        best_sync_obj = lbfgs_sync;
        v_nodes = std::move(v_lbfgs);
        a_nodes = std::move(a_lbfgs);
      }

      // ---- Stage B4: Deterministic Exact 6-DOF Synchronized Polish (Blocked-Interval Aware) ----
      std::vector<double> seg_sync(M, 0.0);
      best_sync_obj = eval_total_sync(v_nodes, a_nodes, &seg_sync);

      std::vector<std::array<double, 6>> step_v(M + 1), step_a(M + 1);
      for (size_t k = 1; k < M; ++k) {
        for (size_t j = 0; j < 6; ++j) {
          step_v[k][j] = 0.15 * lims[j].v_lim * kHeadroom;
          step_a[k][j] = 0.15 * lims[j].a_lim * kHeadroom;
        }
      }

      for (int sweep = 0; sweep < 5; ++sweep) {
        if (check_cancel(waypoints.front().loc)) {
          return false;
        }
        const auto v_prev = v_nodes;
        const auto a_prev = a_nodes;
        const double obj_before = best_sync_obj;
        bool any_improved = false;

        for (size_t k = 1; k < M; ++k) {
          if (check_cancel(waypoints[k].loc)) {
            return false;
          }
          if (v_fixed[k] && a_fixed[k]) {
            continue;
          }
          // Evaluate exact synchronized pair cost for segments (k-1, k).
          auto pair_sync_cost = [&](
            const std::array<double, 6> & vk, const std::array<double, 6> & ak,
            double & out_t1, double & out_t2) -> double
          {
            double s1 = 0.0, s2 = 0.0;
            const double t1 = eval_6dof_sync_dur(
              q_nodes[k - 1], v_nodes[k - 1], a_nodes[k - 1],
              q_nodes[k], vk, ak, lims, seg_dt[k - 1], &s1);
            if (!std::isfinite(t1)) {
              return std::numeric_limits<double>::infinity();
            }
            const double t2 = eval_6dof_sync_dur(
              q_nodes[k], vk, ak,
              q_nodes[k + 1], v_nodes[k + 1], a_nodes[k + 1], lims, seg_dt[k], &s2);
            if (!std::isfinite(t2)) {
              return std::numeric_limits<double>::infinity();
            }
            out_t1 = t1;
            out_t2 = t2;
            return (t1 + t2) + 1e-4 * (s1 + s2);
          };

          double cur_t1 = 0.0, cur_t2 = 0.0;
          double cur_pair = pair_sync_cost(v_nodes[k], a_nodes[k], cur_t1, cur_t2);

          for (size_t j = 0; j < 6; ++j) {
            auto try_kj = [&](double vt, double at) -> bool {
              project_node_respecting_masks(
                q_nodes[k][j], v_fixed[k], a_fixed[k], vt, at, lims[j]);
              auto vk = v_nodes[k];
              auto ak = a_nodes[k];
              vk[j] = vt;
              ak[j] = at;
              double t1 = 0.0, t2 = 0.0;
              const double c = pair_sync_cost(vk, ak, t1, t2);
              if (c < cur_pair - 1e-7) {
                v_nodes[k][j] = vt;
                a_nodes[k][j] = at;
                seg_sync[k - 1] = t1;
                seg_sync[k] = t2;
                cur_pair = c;
                any_improved = true;
                return true;
              }
              return false;
            };

            if (!v_fixed[k]) {
              const double sv = step_v[k][j];
              if (try_kj(v_nodes[k][j] + sv, a_nodes[k][j])) {
                try_kj(v_nodes[k][j] + 0.5 * sv, a_nodes[k][j]);
              } else if (try_kj(v_nodes[k][j] - sv, a_nodes[k][j])) {
                try_kj(v_nodes[k][j] - 0.5 * sv, a_nodes[k][j]);
              } else {
                step_v[k][j] *= 0.5;
              }
            }
            if (!a_fixed[k]) {
              const double sa = step_a[k][j];
              if (try_kj(v_nodes[k][j], a_nodes[k][j] + sa)) {
                try_kj(v_nodes[k][j], a_nodes[k][j] + 0.5 * sa);
              } else if (try_kj(v_nodes[k][j], a_nodes[k][j] - sa)) {
                try_kj(v_nodes[k][j], a_nodes[k][j] - 0.5 * sa);
              } else {
                step_a[k][j] *= 0.5;
              }
            }
          }
        }

        if (any_improved) {
          // Powell-style extrapolated multi-waypoint step on exact synchronized objective.
          for (double alpha : {1.0, 0.5}) {
            auto v_try = v_nodes;
            auto a_try = a_nodes;
            for (size_t k = 1; k < M; ++k) {
              for (size_t j = 0; j < 6; ++j) {
                if (!v_fixed[k]) {
                  v_try[k][j] = v_nodes[k][j] + alpha * (v_nodes[k][j] - v_prev[k][j]);
                }
                if (!a_fixed[k]) {
                  a_try[k][j] = a_nodes[k][j] + alpha * (a_nodes[k][j] - a_prev[k][j]);
                }
                project_node_respecting_masks(
                  q_nodes[k][j], v_fixed[k], a_fixed[k], v_try[k][j], a_try[k][j], lims[j]);
              }
            }
            std::vector<double> seg_try;
            const double o_try = eval_total_sync(v_try, a_try, &seg_try);
            if (o_try < best_sync_obj - 1e-6) {
              v_nodes.swap(v_try);
              a_nodes.swap(a_try);
              seg_sync.swap(seg_try);
              best_sync_obj = o_try;
              break;
            }
          }
          best_sync_obj = eval_total_sync(v_nodes, a_nodes, &seg_sync);
        }

        const double rel_imp = (obj_before - best_sync_obj) / std::max(obj_before, 1e-6);
        if (!any_improved && sweep >= 1) {
          break;
        }
        if (rel_imp < 1e-5 && sweep >= 2) {
          break;
        }
      }
    }
  }

  // 3. Final 1 ms synchronized Layer-A solve for all M segments and 1 kHz sampling.
  std::vector<SegmentSolution6DoF> seg_sols(M);
  double total_dur = 0.0;
  for (size_t m = 0; m < M; ++m) {
    if (check_cancel(waypoints[m].loc)) {
      return false;
    }
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
