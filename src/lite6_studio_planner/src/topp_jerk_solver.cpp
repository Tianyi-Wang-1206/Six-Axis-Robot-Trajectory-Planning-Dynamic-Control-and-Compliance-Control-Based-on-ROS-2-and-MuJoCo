#include "lite6_studio_planner/topp_jerk_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace lite6_studio_planner
{
namespace
{

// Exact analytical piecewise-linear TOPP-RA velocity profile smoothed by a zero-overshoot
// boxcar jerk filter of duration T_fir. Because boxcar integration has unit DC gain and
// preserves non-negativity, u(t) monotonically increases from 0 to 1 with C^1 velocity
// and continuous C^0 acceleration (zero vel/acc at both endpoints).
struct SmoothedTOPPProfile
{
  std::vector<double> t;  // TOPP timestamps [0 .. T_topp], strictly increasing
  std::vector<double> v;  // TOPP velocity du/dt_topp at each timestamp, v[0]=v[N]=0
  std::vector<double> U;  // Exact antiderivative int_0^t v(s) ds, U[0]=0, U[N]=1
  std::vector<double> W;  // Exact antiderivative int_0^t U(s) ds
  double T_topp = 0.0;
  double T_fir = 0.02;

  double raw_duration() const { return T_topp + T_fir; }

  void precompute_integrals()
  {
    const size_t n = t.size();
    U.assign(n, 0.0);
    W.assign(n, 0.0);
    for (size_t k = 0; k + 1 < n; ++k) {
      const double h = t[k + 1] - t[k];
      const double dv = v[k + 1] - v[k];
      const double m = (h > 1e-12) ? (dv / h) : 0.0;
      U[k + 1] = U[k] + v[k] * h + 0.5 * m * h * h;
      W[k + 1] = W[k] + U[k] * h + 0.5 * v[k] * h * h + (1.0 / 6.0) * m * h * h * h;
    }
    // Renormalize U and W slightly if numerical accumulation shifted U.back() by ~1e-15
    if (U.back() > 1e-12 && std::abs(U.back() - 1.0) > 1e-12) {
      const double corr = 1.0 / U.back();
      for (size_t k = 0; k < n; ++k) {
        v[k] *= corr;
        U[k] *= corr;
        W[k] *= corr;
      }
    }
  }

  void eval_raw(double tau, double & V_val, double & dV_val, double & U_val, double & W_val) const
  {
    if (tau <= 0.0) {
      V_val = 0.0;
      dV_val = 0.0;
      U_val = 0.0;
      W_val = 0.0;
      return;
    }
    if (tau >= T_topp) {
      const double dt = tau - T_topp;
      V_val = 0.0;
      dV_val = 0.0;
      U_val = U.back();
      W_val = W.back() + U.back() * dt;
      return;
    }

    auto it = std::upper_bound(t.begin(), t.end(), tau);
    size_t k = (it == t.begin()) ? 0 : static_cast<size_t>(std::distance(t.begin(), it) - 1);
    if (k + 1 >= t.size()) {
      k = t.size() - 2;
    }
    const double h = t[k + 1] - t[k];
    const double delta = std::clamp(tau - t[k], 0.0, h);
    const double m = (h > 1e-12) ? ((v[k + 1] - v[k]) / h) : 0.0;

    V_val = v[k] + m * delta;
    dV_val = m;
    U_val = U[k] + v[k] * delta + 0.5 * m * delta * delta;
    W_val = W[k] + U[k] * delta + 0.5 * v[k] * delta * delta + (1.0 / 6.0) * m * delta * delta * delta;
  }

  void eval_smoothed(
    double t_eval, double & u, double & u_d, double & u_dd, double & u_ddd) const
  {
    const double T_raw = raw_duration();
    if (t_eval <= 0.0) {
      u = 0.0;
      u_d = 0.0;
      u_dd = 0.0;
      u_ddd = 0.0;
      return;
    }
    if (t_eval >= T_raw) {
      u = 1.0;
      u_d = 0.0;
      u_dd = 0.0;
      u_ddd = 0.0;
      return;
    }

    double V1, dV1, U1, W1;
    double V0, dV0, U0, W0;
    eval_raw(t_eval, V1, dV1, U1, W1);
    eval_raw(t_eval - T_fir, V0, dV0, U0, W0);

    const double invF = 1.0 / T_fir;
    u = std::clamp((W1 - W0) * invF, 0.0, 1.0);
    u_d = std::max(0.0, (U1 - U0) * invF);
    u_dd = (V1 - V0) * invF;
    u_ddd = (dV1 - dV0) * invF;
  }
};

}  // namespace

bool TOPPJerkSolver::solve(
  const C3JointSpline & joint_spline,
  const CompositeCartesianPath * cart_path,
  const std::array<lite6_planner::JointLimit, 6> & joint_limits,
  const lite6_planner::CartesianLimit & cart_limits,
  double default_vel_scale,
  double default_acc_scale,
  double default_jerk_scale,
  const SourceLocation & default_loc,
  SampledTrajectory1kHz & out_traj,
  std::string & err_msg)
{
  out_traj.samples.clear();
  out_traj.duration_sec = 0.0;

  // Handle degenerate zero-displacement paths cleanly without division by zero.
  if (joint_spline.max_q1().lpNorm<Eigen::Infinity>() < 1e-10) {
    Vec6 q0, q1, q2, q3;
    joint_spline.eval(0.0, q0, q1, q2, q3);
    out_traj.duration_sec = 0.002;
    for (int tick = 0; tick <= 2; ++tick) {
      SampledTrajectory1kHz::Sample smp;
      smp.time_sec = static_cast<double>(tick) * 0.001;
      smp.path_u = (tick == 0) ? 0.0 : (tick == 2 ? 1.0 : 0.5);
      for (size_t i = 0; i < 6; ++i) {
        smp.q[i] = q0[static_cast<int>(i)];
        smp.dq[i] = 0.0;
        smp.ddq[i] = 0.0;
      }
      smp.loc = (cart_path != nullptr) ? cart_path->eval_normalized(smp.path_u).loc : default_loc;
      out_traj.samples.push_back(smp);
    }
    return true;
  }

  constexpr int kGridN = 400;
  const double du = 1.0 / static_cast<double>(kGridN);

  std::vector<double> u_dot_max(kGridN + 1, 100.0);
  std::vector<double> u_ddot_max(kGridN + 1, 500.0);

  double t_j_char = 0.0;
  // Stage 1: Evaluate local kinematic + curvature + jerk bounds along u in [0, 1].
  for (int k = 0; k <= kGridN; ++k) {
    const double u = static_cast<double>(k) * du;
    Vec6 q, q1, q2, q3;
    joint_spline.eval(u, q, q1, q2, q3);

    double vs = default_vel_scale;
    double as = default_acc_scale;
    double js = default_jerk_scale;

    double v_bound = 100.0;
    double a_bound = 500.0;

    if (cart_path != nullptr) {
      const CartesianSample cs = cart_path->eval_normalized(u);
      vs = cs.vel_scale;
      as = cs.acc_scale;
      js = cs.jerk_scale;

      const double ps_norm = cs.p_s.norm();
      const double pss_norm = cs.p_ss.norm();
      const double ws_norm = cs.w_s.norm();
      const double wss_norm = cs.w_ss.norm();

      if (ps_norm > 1e-9) {
        v_bound = std::min(v_bound, (vs * cart_limits.max_trans_vel) / ps_norm);
        a_bound = std::min(a_bound, (0.95 * as * cart_limits.max_trans_acc) / ps_norm);
      }
      if (pss_norm > 1e-9) {
        v_bound = std::min(v_bound, std::sqrt((0.85 * as * cart_limits.max_trans_acc) / pss_norm));
      }
      if (ws_norm > 1e-9) {
        v_bound = std::min(v_bound, (vs * cart_limits.max_rot_vel) / ws_norm);
        a_bound = std::min(a_bound, (0.95 * as * cart_limits.max_rot_acc) / ws_norm);
      }
      if (wss_norm > 1e-9) {
        v_bound = std::min(v_bound, std::sqrt((0.85 * as * cart_limits.max_rot_acc) / wss_norm));
      }
    }

    for (size_t i = 0; i < 6; ++i) {
      const double d1 = std::abs(q1[static_cast<int>(i)]);
      const double d2 = std::abs(q2[static_cast<int>(i)]);
      const double d3 = std::abs(q3[static_cast<int>(i)]);
      const auto & lim = joint_limits[i];

      if (d1 > 1e-9) {
        v_bound = std::min(v_bound, (vs * lim.max_vel) / d1);
        a_bound = std::min(a_bound, (0.95 * as * lim.max_acc) / d1);
        t_j_char = std::max(t_j_char, (as * lim.max_acc) / std::max(1e-6, js * lim.max_jerk));
      }
      if (d2 > 1e-9) {
        v_bound = std::min(v_bound, std::sqrt((0.80 * as * lim.max_acc) / d2));
      }
      if (d3 > 1e-9) {
        v_bound = std::min(v_bound, std::cbrt((0.65 * js * lim.max_jerk) / d3));
      }
    }

    u_dot_max[static_cast<size_t>(k)] = std::max(1e-3, v_bound);
    u_ddot_max[static_cast<size_t>(k)] = std::max(1e-3, a_bound);
  }

  // Stage 2: TOPP-RA Backward & Forward Reachability Pass on s_dot(u) = v_prof(u)
  // with pinned zero velocity at both endpoints.
  std::vector<double> v_prof = u_dot_max;
  v_prof.front() = 0.0;
  v_prof.back() = 0.0;

  for (int k = kGridN - 1; k >= 0; --k) {
    const double v_next = v_prof[static_cast<size_t>(k + 1)];
    const double a_allow =
      std::min(u_ddot_max[static_cast<size_t>(k)], u_ddot_max[static_cast<size_t>(k + 1)]);
    const double v_reach = std::sqrt(v_next * v_next + 2.0 * du * a_allow);
    v_prof[static_cast<size_t>(k)] = std::min(v_prof[static_cast<size_t>(k)], v_reach);
  }
  for (int k = 0; k < kGridN; ++k) {
    const double v_curr = v_prof[static_cast<size_t>(k)];
    const double a_allow =
      std::min(u_ddot_max[static_cast<size_t>(k)], u_ddot_max[static_cast<size_t>(k + 1)]);
    const double v_reach = std::sqrt(v_curr * v_curr + 2.0 * du * a_allow);
    v_prof[static_cast<size_t>(k + 1)] = std::min(v_prof[static_cast<size_t>(k + 1)], v_reach);
  }

  // Stage 3: Exact piecewise-linear TOPP-RA time-mapping + FIR boxcar jerk smoothing.
  SmoothedTOPPProfile profile;
  profile.t.resize(kGridN + 1, 0.0);
  profile.v = v_prof;
  for (int k = 0; k < kGridN; ++k) {
    const double vm =
      std::max(1e-4, 0.5 * (v_prof[static_cast<size_t>(k)] + v_prof[static_cast<size_t>(k + 1)]));
    profile.t[static_cast<size_t>(k + 1)] = profile.t[static_cast<size_t>(k)] + du / vm;
  }
  profile.T_topp = profile.t.back();
  profile.T_fir = std::clamp(std::max(t_j_char, 0.025), 0.015, std::max(0.025, 0.6 * profile.T_topp));
  profile.precompute_integrals();

  double base_T = std::max(0.004, profile.raw_duration());

  // Stage 4: Closed-loop bottleneck time-dilation so that EVERY joint & Cartesian limit
  // (including 1 kHz finite-difference jerk) hits ~99.0% of the tightest constraint
  // without ever violating a limit.
  auto compute_bottleneck_dilation = [&](double T_eval) -> double {
    const double time_scale = profile.raw_duration() / T_eval;
    const int n_steps = std::max(24, static_cast<int>(std::ceil(T_eval / 0.001)));
    const double dt = T_eval / static_cast<double>(n_steps);

    double max_vel_ratio = 0.0;
    double max_acc_ratio = 0.0;
    double max_jrk_ratio = 0.0;

    std::array<double, 6> prev_acc{};
    for (int k = 0; k <= n_steps; ++k) {
      const double t = static_cast<double>(k) * dt;
      double u = 0.0, du_dt = 0.0, d2u_dt2 = 0.0, d3u_dt3 = 0.0;
      profile.eval_smoothed(t * time_scale, u, du_dt, d2u_dt2, d3u_dt3);
      du_dt *= time_scale;
      d2u_dt2 *= (time_scale * time_scale);
      d3u_dt3 *= (time_scale * time_scale * time_scale);

      Vec6 q, q1, q2, q3;
      joint_spline.eval(u, q, q1, q2, q3);

      const Vec6 dq = q1 * du_dt;
      const Vec6 ddq = q2 * (du_dt * du_dt) + q1 * d2u_dt2;
      const Vec6 jrk =
        q3 * (du_dt * du_dt * du_dt) + 3.0 * q2 * du_dt * d2u_dt2 + q1 * d3u_dt3;

      double vs_local = default_vel_scale;
      double as_local = default_acc_scale;
      double js_local = default_jerk_scale;

      if (cart_path != nullptr) {
        const CartesianSample cs = cart_path->eval_normalized(u);
        vs_local = cs.vel_scale;
        as_local = cs.acc_scale;
        js_local = cs.jerk_scale;

        const Eigen::Vector3d v_trans = cs.p_s * du_dt;
        const Eigen::Vector3d a_trans = cs.p_ss * (du_dt * du_dt) + cs.p_s * d2u_dt2;
        const Eigen::Vector3d v_rot = cs.w_s * du_dt;
        const Eigen::Vector3d a_rot = cs.w_ss * (du_dt * du_dt) + cs.w_s * d2u_dt2;

        max_vel_ratio = std::max(
          max_vel_ratio, v_trans.norm() / (cart_limits.max_trans_vel * vs_local));
        max_acc_ratio = std::max(
          max_acc_ratio, a_trans.norm() / (cart_limits.max_trans_acc * as_local));
        max_vel_ratio = std::max(
          max_vel_ratio, v_rot.norm() / (cart_limits.max_rot_vel * vs_local));
        max_acc_ratio = std::max(
          max_acc_ratio, a_rot.norm() / (cart_limits.max_rot_acc * as_local));
      }

      for (size_t i = 0; i < 6; ++i) {
        const auto & lim = joint_limits[i];
        max_vel_ratio = std::max(
          max_vel_ratio, std::abs(dq[static_cast<int>(i)]) / (lim.max_vel * vs_local));
        max_acc_ratio = std::max(
          max_acc_ratio, std::abs(ddq[static_cast<int>(i)]) / (lim.max_acc * as_local));
        max_jrk_ratio = std::max(
          max_jrk_ratio, std::abs(jrk[static_cast<int>(i)]) / (lim.max_jerk * js_local));
        if (k > 0) {
          const double disc_jrk =
            std::abs(ddq[static_cast<int>(i)] - prev_acc[i]) / dt;
          max_jrk_ratio = std::max(
            max_jrk_ratio, disc_jrk / (lim.max_jerk * js_local));
        }
        prev_acc[i] = ddq[static_cast<int>(i)];
      }
    }

    const double max_all = std::max({max_vel_ratio, max_acc_ratio, max_jrk_ratio});
    if (max_all < 1e-12) {
      return 1.0;
    }
    return std::max({
      max_vel_ratio / 0.99,
      std::sqrt(max_acc_ratio / 0.99),
      std::cbrt(max_jrk_ratio / 0.99)});
  };

  for (int pass = 0; pass < 4; ++pass) {
    const double dilation = compute_bottleneck_dilation(base_T);
    if (!std::isfinite(dilation) || dilation <= 0.0) {
      err_msg = "Non-finite time-dilation factor in TOPP-Jerk solver";
      return false;
    }
    base_T = std::max(0.002, base_T * std::max(0.1, dilation));
    if (dilation <= 1.0 && dilation >= 0.985) {
      break;
    }
  }

  // Round final duration up to an exact integer multiple of 1 ms (0.001 s).
  const int total_ticks = std::max(2, static_cast<int>(std::ceil(base_T * 1000.0)));
  const double final_T = static_cast<double>(total_ticks) * 0.001;
  if (final_T > 300.0) {
    err_msg = "Computed trajectory duration=" + std::to_string(final_T) + " s exceeds 300 s limit";
    return false;
  }

  const double time_scale = profile.raw_duration() / final_T;
  out_traj.duration_sec = final_T;
  out_traj.samples.reserve(static_cast<size_t>(total_ticks + 1));

  for (int tick = 0; tick <= total_ticks; ++tick) {
    const double t = static_cast<double>(tick) * 0.001;
    double u = 0.0, du_dt = 0.0, d2u_dt2 = 0.0, d3u_dt3 = 0.0;
    profile.eval_smoothed(t * time_scale, u, du_dt, d2u_dt2, d3u_dt3);
    du_dt *= time_scale;
    d2u_dt2 *= (time_scale * time_scale);
    if (tick == 0) {
      u = 0.0;
      du_dt = 0.0;
      d2u_dt2 = 0.0;
    } else if (tick == total_ticks) {
      u = 1.0;
      du_dt = 0.0;
      d2u_dt2 = 0.0;
    }

    Vec6 q, q1, q2, q3;
    joint_spline.eval(u, q, q1, q2, q3);
    const Vec6 dq = q1 * du_dt;
    const Vec6 ddq = q2 * (du_dt * du_dt) + q1 * d2u_dt2;

    SampledTrajectory1kHz::Sample smp;
    smp.time_sec = t;
    smp.path_u = u;
    for (size_t i = 0; i < 6; ++i) {
      smp.q[i] = q[static_cast<int>(i)];
      smp.dq[i] = (tick == 0 || tick == total_ticks) ? 0.0 : dq[static_cast<int>(i)];
      smp.ddq[i] = (tick == 0 || tick == total_ticks) ? 0.0 : ddq[static_cast<int>(i)];
    }
    smp.loc = (cart_path != nullptr) ? cart_path->eval_normalized(u).loc : default_loc;
    out_traj.samples.push_back(smp);
  }

  return true;
}

}  // namespace lite6_studio_planner
