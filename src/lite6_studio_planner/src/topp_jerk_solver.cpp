#include "lite6_studio_planner/topp_jerk_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace lite6_studio_planner
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
// Sides of the inscribed polygon approximating Cartesian norm constraints.
constexpr int kPolygonSides = 16;
// TOPP-RA plans against this fraction of the acceleration limits; the 1 kHz check
// accepts up to kCheckMargin, leaving headroom for the boxcar smoothing.
constexpr double kAccMargin = 0.97;
constexpr double kCheckMargin = 0.995;
// Heuristic jerk-aware speed cap: |q'''| * sdot^3 <= kJerkCapShare * j_max.
constexpr double kJerkCapShare = 0.6;
constexpr int kFineSamples = 2048;
constexpr int kMinGrid = 64;
constexpr int kMaxGrid = 6000;
constexpr int kMaxRefinePasses = 24;
constexpr int kMaxDilationPasses = 8;
constexpr double kMaxDurationSec = 300.0;

// Grid resolution targets per stage.
constexpr double kGridJointStep = 0.003;       // rad
constexpr double kGridJointCurvature = 0.006;  // rad (chord sag scale)
constexpr double kGridTransStep = 0.002;       // m
constexpr double kGridRotStep = 0.005;         // rad

struct PathPoint
{
  Vec6 q = Vec6::Zero(), q1 = Vec6::Zero(), q2 = Vec6::Zero(), q3 = Vec6::Zero();
  double vs = 0.5, as = 0.5, js = 1.0;
  bool cart = false;
  Eigen::Vector3d p1 = Eigen::Vector3d::Zero(), p2 = Eigen::Vector3d::Zero();
  Eigen::Vector3d w1 = Eigen::Vector3d::Zero(), w2 = Eigen::Vector3d::Zero();
  SourceLocation loc;
};

class PathSampler
{
public:
  PathSampler(
    const JointPath & path, const CompositeCartesianPath * cart,
    double vs, double as, double js, const SourceLocation & loc)
  : path_(path), cart_(cart), vs_(vs), as_(as), js_(js), loc_(loc) {}

  PathPoint eval(double u) const
  {
    PathPoint pp;
    path_.eval(u, pp.q, pp.q1, pp.q2, pp.q3);
    if (cart_ != nullptr) {
      const CartesianSample cs = cart_->eval_normalized(u);
      pp.cart = true;
      pp.p1 = cs.p_s;
      pp.p2 = cs.p_ss;
      pp.w1 = cs.w_s;
      pp.w2 = cs.w_ss;
      pp.vs = cs.vel_scale;
      pp.as = cs.acc_scale;
      pp.js = cs.jerk_scale;
      pp.loc = cs.loc;
      return pp;
    }
    MotionOptions opts;
    SourceLocation loc;
    if (path_.options_at(u, opts, loc)) {
      pp.vs = opts.vel_scale;
      pp.as = opts.acc_scale;
      pp.js = opts.jerk_scale;
      pp.loc = loc;
    } else {
      pp.vs = vs_;
      pp.as = as_;
      pp.js = js_;
      pp.loc = loc_;
    }
    return pp;
  }

private:
  const JointPath & path_;
  const CompositeCartesianPath * cart_;
  double vs_, as_, js_;
  SourceLocation loc_;
};

// a * sdd + b * x <= c, with x = sdot^2 and sdd = d2u/dt2.
struct LinCon
{
  double a, b, c;
};

// Inner polygon of |d1 * sdd + d2 * x| <= limit (vectors span at most a plane).
void add_norm_polygon(
  const Eigen::Vector3d & d1, const Eigen::Vector3d & d2, double limit,
  std::vector<LinCon> & out)
{
  const double n1 = d1.norm();
  if (n1 < 1e-12) {
    const double n2 = d2.norm();
    if (n2 > 1e-12) {
      out.push_back({0.0, n2, limit});
    }
    return;
  }
  const Eigen::Vector3d e1 = d1 / n1;
  const double b1 = d2.dot(e1);
  const double b2 = (d2 - b1 * e1).norm();
  const double c = limit * std::cos(kPi / kPolygonSides);
  for (int k = 0; k < kPolygonSides; ++k) {
    const double phi = 2.0 * kPi * k / kPolygonSides;
    const double cp = std::cos(phi);
    const double sp = std::sin(phi);
    out.push_back({cp * n1, cp * b1 + sp * b2, c});
  }
}

struct Stage
{
  double x_cap = 0.0;
  std::vector<LinCon> cons;
};

Stage build_stage(
  const PathPoint & pp, double f_v, double f_a,
  const std::array<lite6_planner::JointLimit, 6> & jl,
  const lite6_planner::CartesianLimit & cl)
{
  Stage st;
  double cap = 1e12;
  const double fv2 = f_v * f_v;
  for (int j = 0; j < 6; ++j) {
    const auto & lim = jl[static_cast<size_t>(j)];
    const double d1 = std::abs(pp.q1[j]);
    const double d3 = std::abs(pp.q3[j]);
    if (d1 > 1e-12) {
      const double v = pp.vs * lim.max_vel / d1;
      cap = std::min(cap, v * v * fv2);
    }
    if (d3 > 1e-12) {
      cap = std::min(cap, std::pow(kJerkCapShare * pp.js * lim.max_jerk / d3, 2.0 / 3.0) * fv2);
    }
    const double acc = pp.as * lim.max_acc * kAccMargin * f_a;
    if (d1 > 1e-12 || std::abs(pp.q2[j]) > 1e-12) {
      st.cons.push_back({pp.q1[j], pp.q2[j], acc});
      st.cons.push_back({-pp.q1[j], -pp.q2[j], acc});
    }
  }
  if (pp.cart) {
    const double np = pp.p1.norm();
    const double nw = pp.w1.norm();
    if (np > 1e-12) {
      const double v = pp.vs * cl.max_trans_vel / np;
      cap = std::min(cap, v * v * fv2);
    }
    if (nw > 1e-12) {
      const double v = pp.vs * cl.max_rot_vel / nw;
      cap = std::min(cap, v * v * fv2);
    }
    add_norm_polygon(pp.p1, pp.p2, pp.as * cl.max_trans_acc * kAccMargin * f_a, st.cons);
    add_norm_polygon(pp.w1, pp.w2, pp.as * cl.max_rot_acc * kAccMargin * f_a, st.cons);
  }
  st.x_cap = cap;
  return st;
}

// Exact projection onto x of {(sdd, x) : stage constraints, extra constraints,
// lo_in <= x <= hi_in} by Fourier-Motzkin elimination of sdd.
bool project_x(
  const std::vector<LinCon> & cons, const LinCon * extra, size_t n_extra,
  double lo_in, double hi_in, double & lo, double & hi)
{
  lo = lo_in;
  hi = hi_in;
  // Bounds sdd <= p + r x (upper) or sdd >= p + r x (lower).
  static thread_local std::vector<std::pair<double, double>> uppers, lowers;
  uppers.clear();
  lowers.clear();
  auto add = [&](const LinCon & k) {
    if (k.a > 1e-14) {
      uppers.emplace_back(k.c / k.a, -k.b / k.a);
    } else if (k.a < -1e-14) {
      lowers.emplace_back(k.c / k.a, -k.b / k.a);
    } else if (k.b > 1e-14) {
      hi = std::min(hi, k.c / k.b);
    } else if (k.b < -1e-14) {
      lo = std::max(lo, k.c / k.b);
    }
  };
  for (const auto & k : cons) {
    add(k);
  }
  for (size_t i = 0; i < n_extra; ++i) {
    add(extra[i]);
  }
  for (const auto & l : lowers) {
    for (const auto & u : uppers) {
      const double coef = l.second - u.second;
      const double rhs = u.first - l.first;
      if (coef > 1e-14) {
        hi = std::min(hi, rhs / coef);
      } else if (coef < -1e-14) {
        lo = std::max(lo, rhs / coef);
      }
    }
  }
  const double tol = 1e-9 * (1.0 + std::abs(hi));
  if (lo > hi + tol) {
    return false;
  }
  hi = std::max(hi, lo);
  return true;
}

// Admissible sdd interval at a fixed x.
void sdd_range(
  const std::vector<LinCon> & cons, double x, double & s_lo, double & s_hi)
{
  s_lo = -std::numeric_limits<double>::infinity();
  s_hi = std::numeric_limits<double>::infinity();
  for (const auto & k : cons) {
    if (k.a > 1e-14) {
      s_hi = std::min(s_hi, (k.c - k.b * x) / k.a);
    } else if (k.a < -1e-14) {
      s_lo = std::max(s_lo, (k.c - k.b * x) / k.a);
    }
  }
}

// TOPP-RA: returns x_i = sdot^2 on the grid (rest at both ends).
bool topp_ra(
  const std::vector<double> & grid, const std::vector<Stage> & stages,
  std::vector<double> & x, std::string & err)
{
  const size_t N = grid.size() - 1;
  std::vector<double> k_lo(N + 1, 0.0), k_hi(N + 1, 0.0);
  for (size_t ii = N; ii-- > 0;) {
    const double delta = grid[ii + 1] - grid[ii];
    const LinCon extra[2] = {
      {2.0 * delta, 1.0, k_hi[ii + 1]},
      {-2.0 * delta, -1.0, -k_lo[ii + 1]}};
    if (!project_x(stages[ii].cons, extra, 2, 0.0, stages[ii].x_cap, k_lo[ii], k_hi[ii])) {
      err = "TOPP-RA controllable set is empty at path parameter u=" + std::to_string(grid[ii]);
      return false;
    }
  }
  if (k_lo[0] > 1e-9) {
    err = "TOPP-RA: path cannot start from rest";
    return false;
  }
  x.assign(N + 1, 0.0);
  for (size_t i = 0; i < N; ++i) {
    const double delta = grid[i + 1] - grid[i];
    double s_lo = 0.0, s_hi = 0.0;
    sdd_range(stages[i].cons, x[i], s_lo, s_hi);
    s_hi = std::min(s_hi, (k_hi[i + 1] - x[i]) / (2.0 * delta));
    s_lo = std::max(s_lo, (k_lo[i + 1] - x[i]) / (2.0 * delta));
    const double sdd = std::max(s_lo, s_hi);
    x[i + 1] = std::clamp(x[i] + 2.0 * delta * sdd, k_lo[i + 1], k_hi[i + 1]);
    x[i + 1] = std::max(0.0, x[i + 1]);
  }
  x[N] = 0.0;
  return true;
}

// Piecewise-linear path speed v(t) from TOPP-RA (constant sdd per stage), smoothed by a
// boxcar of length T_fir: finite jerk, unit DC gain, monotone u(t) from 0 to 1.
struct SmoothedProfile
{
  std::vector<double> t;  // raw TOPP timestamps
  std::vector<double> v;  // du/dt at t
  std::vector<double> U;  // int_0^t v
  std::vector<double> W;  // int_0^t U
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
      const double m = (h > 1e-15) ? ((v[k + 1] - v[k]) / h) : 0.0;
      U[k + 1] = U[k] + v[k] * h + 0.5 * m * h * h;
      W[k + 1] = W[k] + U[k] * h + 0.5 * v[k] * h * h + (1.0 / 6.0) * m * h * h * h;
    }
    if (U.back() > 1e-12 && std::abs(U.back() - 1.0) > 1e-12) {
      const double corr = 1.0 / U.back();
      for (size_t k = 0; k < n; ++k) {
        v[k] *= corr;
        U[k] *= corr;
        W[k] *= corr;
      }
    }
  }

  void eval_raw(double tau, double & V, double & dV, double & Uv, double & Wv) const
  {
    if (tau <= 0.0) {
      V = dV = Uv = Wv = 0.0;
      return;
    }
    if (tau >= T_topp) {
      V = dV = 0.0;
      Uv = U.back();
      Wv = W.back() + U.back() * (tau - T_topp);
      return;
    }
    auto it = std::upper_bound(t.begin(), t.end(), tau);
    size_t k = (it == t.begin()) ? 0 : static_cast<size_t>(std::distance(t.begin(), it) - 1);
    k = std::min(k, t.size() - 2);
    const double h = t[k + 1] - t[k];
    const double d = std::clamp(tau - t[k], 0.0, h);
    const double m = (h > 1e-15) ? ((v[k + 1] - v[k]) / h) : 0.0;
    V = v[k] + m * d;
    dV = m;
    Uv = U[k] + v[k] * d + 0.5 * m * d * d;
    Wv = W[k] + U[k] * d + 0.5 * v[k] * d * d + (1.0 / 6.0) * m * d * d * d;
  }

  void eval_smoothed(double te, double & u, double & ud, double & udd, double & uddd) const
  {
    if (te <= 0.0) {
      u = ud = udd = uddd = 0.0;
      return;
    }
    if (te >= raw_duration()) {
      u = 1.0;
      ud = udd = uddd = 0.0;
      return;
    }
    double V1, dV1, U1, W1, V0, dV0, U0, W0;
    eval_raw(te, V1, dV1, U1, W1);
    eval_raw(te - T_fir, V0, dV0, U0, W0);
    const double inv = 1.0 / T_fir;
    u = std::clamp((W1 - W0) * inv, 0.0, 1.0);
    ud = std::max(0.0, (U1 - U0) * inv);
    udd = (V1 - V0) * inv;
    uddd = (dV1 - dV0) * inv;
  }

  double raw_position(double tau) const
  {
    double V, dV, Uv, Wv;
    eval_raw(tau, V, dV, Uv, Wv);
    return Uv;
  }
};

struct Violation
{
  double tau_raw;
  double rv, ra, rj;
};

struct EvalResult
{
  std::vector<Violation> violations;
  double max_rv = 0.0, max_ra = 0.0, max_rj = 0.0;
};

// Samples the smoothed profile at 1 kHz over final_T; reports limit ratios and,
// when out is non-null, emits the samples.
EvalResult evaluate(
  const SmoothedProfile & prof, double final_T, const PathSampler & sampler,
  const std::array<lite6_planner::JointLimit, 6> & jl,
  const lite6_planner::CartesianLimit & cl,
  SampledTrajectory1kHz * out)
{
  EvalResult res;
  const int ticks = std::max(2, static_cast<int>(std::llround(final_T * 1000.0)));
  const double ts = prof.raw_duration() / final_T;
  Vec6 prev_ddq = Vec6::Zero();
  if (out != nullptr) {
    out->samples.clear();
    out->samples.reserve(static_cast<size_t>(ticks + 1));
    out->duration_sec = static_cast<double>(ticks) * 0.001;
  }
  for (int tick = 0; tick <= ticks; ++tick) {
    const double t = static_cast<double>(tick) * 0.001;
    const double tau = std::min(t * ts, prof.raw_duration());
    double u = 0.0, ud = 0.0, udd = 0.0, uddd = 0.0;
    prof.eval_smoothed(tau, u, ud, udd, uddd);
    ud *= ts;
    udd *= ts * ts;
    uddd *= ts * ts * ts;
    if (tick == 0) {
      u = 0.0;
      ud = udd = uddd = 0.0;
    } else if (tick == ticks) {
      u = 1.0;
      ud = udd = uddd = 0.0;
    }
    const PathPoint pp = sampler.eval(u);
    const Vec6 dq = pp.q1 * ud;
    const Vec6 ddq = pp.q2 * (ud * ud) + pp.q1 * udd;
    const Vec6 jrk = pp.q3 * (ud * ud * ud) + 3.0 * pp.q2 * (ud * udd) + pp.q1 * uddd;

    double rv = 0.0, ra = 0.0, rj = 0.0;
    for (int j = 0; j < 6; ++j) {
      const auto & lim = jl[static_cast<size_t>(j)];
      rv = std::max(rv, std::abs(dq[j]) / (pp.vs * lim.max_vel * kCheckMargin));
      ra = std::max(ra, std::abs(ddq[j]) / (pp.as * lim.max_acc * kCheckMargin));
      double jerk = std::abs(jrk[j]);
      if (tick > 0) {
        jerk = std::max(jerk, std::abs(ddq[j] - prev_ddq[j]) / 0.001);
      }
      rj = std::max(rj, jerk / (pp.js * lim.max_jerk * kCheckMargin));
    }
    if (pp.cart) {
      rv = std::max(rv, (pp.p1 * ud).norm() / (pp.vs * cl.max_trans_vel * kCheckMargin));
      rv = std::max(rv, (pp.w1 * ud).norm() / (pp.vs * cl.max_rot_vel * kCheckMargin));
      ra = std::max(ra, (pp.p2 * (ud * ud) + pp.p1 * udd).norm() /
        (pp.as * cl.max_trans_acc * kCheckMargin));
      ra = std::max(ra, (pp.w2 * (ud * ud) + pp.w1 * udd).norm() /
        (pp.as * cl.max_rot_acc * kCheckMargin));
    }
    prev_ddq = ddq;
    res.max_rv = std::max(res.max_rv, rv);
    res.max_ra = std::max(res.max_ra, ra);
    res.max_rj = std::max(res.max_rj, rj);
    if (rv > 1.0 || ra > 1.0 || rj > 1.0) {
      res.violations.push_back({tau, rv, ra, rj});
    }
    if (out != nullptr) {
      SampledTrajectory1kHz::Sample smp;
      smp.time_sec = t;
      smp.path_u = u;
      for (size_t i = 0; i < 6; ++i) {
        const int k = static_cast<int>(i);
        smp.q[i] = pp.q[k];
        smp.dq[i] = (tick == 0 || tick == ticks) ? 0.0 : dq[k];
        smp.ddq[i] = (tick == 0 || tick == ticks) ? 0.0 : ddq[k];
      }
      smp.loc = pp.loc;
      out->samples.push_back(smp);
    }
  }
  return res;
}

}  // namespace

bool TOPPJerkSolver::solve(
  const JointPath & path,
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
  const PathSampler sampler(
    path, cart_path, default_vel_scale, default_acc_scale, default_jerk_scale, default_loc);

  // Fine scan: motion detection, grid density and boxcar length.
  std::vector<double> fine_u(kFineSamples + 1), density(kFineSamples + 1);
  double max_motion = 0.0;
  double t_ratio = 0.0;
  for (int k = 0; k <= kFineSamples; ++k) {
    const double u = static_cast<double>(k) / kFineSamples;
    const PathPoint pp = sampler.eval(u);
    fine_u[static_cast<size_t>(k)] = u;
    double rho = 0.0;
    for (int j = 0; j < 6; ++j) {
      const double d1 = std::abs(pp.q1[j]);
      rho = std::max(rho, d1 / kGridJointStep);
      rho = std::max(rho, std::sqrt(std::abs(pp.q2[j]) / kGridJointCurvature));
      max_motion = std::max(max_motion, d1);
      if (d1 > 1e-9) {
        const auto & lim = joint_limits[static_cast<size_t>(j)];
        t_ratio = std::max(t_ratio, (pp.as * lim.max_acc) / (pp.js * lim.max_jerk));
      }
    }
    if (pp.cart) {
      rho = std::max(rho, pp.p1.norm() / kGridTransStep);
      rho = std::max(rho, pp.w1.norm() / kGridRotStep);
      max_motion = std::max({max_motion, pp.p1.norm(), pp.w1.norm()});
    }
    density[static_cast<size_t>(k)] = rho;
  }

  if (max_motion < 1e-10) {
    // Degenerate zero-displacement path.
    const PathPoint p0 = sampler.eval(0.0);
    out_traj.duration_sec = 0.002;
    for (int tick = 0; tick <= 2; ++tick) {
      SampledTrajectory1kHz::Sample smp;
      smp.time_sec = static_cast<double>(tick) * 0.001;
      smp.path_u = 0.5 * tick;
      for (size_t i = 0; i < 6; ++i) {
        smp.q[i] = p0.q[static_cast<int>(i)];
      }
      smp.loc = sampler.eval(smp.path_u).loc;
      out_traj.samples.push_back(smp);
    }
    return true;
  }

  // Adaptive grid: equal increments of the integrated density.
  std::vector<double> cum(kFineSamples + 1, 0.0);
  for (int k = 0; k < kFineSamples; ++k) {
    const size_t i = static_cast<size_t>(k);
    cum[i + 1] = cum[i] + 0.5 * (density[i] + density[i + 1]) / kFineSamples;
  }
  const int N = std::clamp(static_cast<int>(std::ceil(cum.back())), kMinGrid, kMaxGrid);
  std::vector<double> grid(static_cast<size_t>(N) + 1, 0.0);
  {
    size_t k = 0;
    for (int i = 1; i < N; ++i) {
      const double target = cum.back() * i / N;
      while (k + 1 < cum.size() && cum[k + 1] < target) {
        ++k;
      }
      const double span = cum[k + 1] - cum[k];
      const double frac = span > 1e-15 ? (target - cum[k]) / span : 0.0;
      grid[static_cast<size_t>(i)] = fine_u[k] + frac * (fine_u[k + 1] - fine_u[k]);
    }
    grid.back() = 1.0;
    for (size_t i = 1; i < grid.size(); ++i) {
      grid[i] = std::max(grid[i], grid[i - 1] + 1e-9);
    }
    grid.back() = std::max(grid.back(), 1.0);
    const double scale = 1.0 / grid.back();
    for (auto & g : grid) {
      g *= scale;
    }
  }

  std::vector<PathPoint> points(grid.size());
  for (size_t i = 0; i < grid.size(); ++i) {
    points[i] = sampler.eval(grid[i]);
  }
  std::vector<double> f_v(grid.size(), 1.0), f_a(grid.size(), 1.0);
  std::vector<Stage> stages(grid.size());

  SmoothedProfile prof;
  double final_T = 0.0;
  EvalResult res;
  bool accepted = false;

  for (int pass = 0; pass < kMaxRefinePasses && !accepted; ++pass) {
    for (size_t i = 0; i < grid.size(); ++i) {
      stages[i] = build_stage(points[i], f_v[i], f_a[i], joint_limits, cart_limits);
    }
    std::vector<double> x;
    if (!topp_ra(grid, stages, x, err_msg)) {
      return false;
    }

    prof = SmoothedProfile{};
    prof.t.assign(grid.size(), 0.0);
    prof.v.assign(grid.size(), 0.0);
    for (size_t i = 0; i < grid.size(); ++i) {
      prof.v[i] = std::sqrt(std::max(0.0, x[i]));
    }
    for (size_t i = 0; i + 1 < grid.size(); ++i) {
      const double vsum = prof.v[i] + prof.v[i + 1];
      if (vsum < 1e-12) {
        err_msg = "TOPP-RA profile stalls at path parameter u=" + std::to_string(grid[i]);
        return false;
      }
      prof.t[i + 1] = prof.t[i] + 2.0 * (grid[i + 1] - grid[i]) / vsum;
    }
    prof.T_topp = prof.t.back();
    if (!(prof.T_topp > 0.0) || prof.T_topp > kMaxDurationSec) {
      err_msg = "TOPP-RA duration=" + std::to_string(prof.T_topp) + " s is out of range";
      return false;
    }
    // Boxcar length bounds the jerk of a full +a -> -a acceleration reversal.
    prof.T_fir = std::clamp(2.0 * t_ratio, 0.01, std::max(0.02, 0.6 * prof.T_topp));
    prof.precompute_integrals();

    final_T = std::ceil(prof.raw_duration() * 1000.0 - 1e-9) * 0.001;
    res = evaluate(prof, final_T, sampler, joint_limits, cart_limits, nullptr);
    if (res.violations.empty()) {
      accepted = true;
      break;
    }

    // Local tightening around each violation: the smoothed sample at raw time tau
    // depends on the raw profile over [tau - T_fir, tau].
    std::vector<double> req_v(grid.size(), 1.0), req_a(grid.size(), 1.0);
    for (const auto & vio : res.violations) {
      double g_v = 1.0, g_a = 1.0;
      if (vio.rv > 1.0) {
        g_v = std::min(g_v, 0.98 / vio.rv);
      }
      if (vio.ra > 1.0) {
        g_a = std::min(g_a, 0.97 / vio.ra);
        g_v = std::min(g_v, std::sqrt(0.98 / vio.ra));
      }
      if (vio.rj > 1.0) {
        g_a = std::min(g_a, 0.97 / vio.rj);
        g_v = std::min(g_v, std::cbrt(0.98 / vio.rj));
      }
      // Damped update: a single extreme ratio must not over-slow a whole region;
      // remaining excess is removed by the following passes.
      g_v = std::max(g_v, 0.5);
      g_a = std::max(g_a, 0.25);
      const double u_lo = prof.raw_position(vio.tau_raw - prof.T_fir);
      const double u_hi = prof.raw_position(vio.tau_raw);
      auto lo_it = std::lower_bound(grid.begin(), grid.end(), u_lo);
      auto hi_it = std::upper_bound(grid.begin(), grid.end(), u_hi);
      const size_t i0 = static_cast<size_t>(
        std::max<std::ptrdiff_t>(0, std::distance(grid.begin(), lo_it) - 2));
      const size_t i1 = std::min(grid.size() - 1, static_cast<size_t>(
        std::distance(grid.begin(), hi_it) + 1));
      for (size_t i = i0; i <= i1; ++i) {
        req_v[i] = std::min(req_v[i], g_v);
        req_a[i] = std::min(req_a[i], g_a);
      }
    }
    for (size_t i = 0; i < grid.size(); ++i) {
      f_v[i] = std::max(1e-3, f_v[i] * req_v[i]);
      f_a[i] = std::max(1e-3, f_a[i] * req_a[i]);
    }
  }

  // Last resort: uniform time dilation of the smoothed profile (exact scaling law).
  for (int pass = 0; !accepted && pass < kMaxDilationPasses; ++pass) {
    const double dil = 1.002 * std::max({1.0, res.max_rv, std::sqrt(res.max_ra), std::cbrt(res.max_rj)});
    final_T = std::ceil(final_T * dil * 1000.0 - 1e-9) * 0.001;
    if (final_T > kMaxDurationSec) {
      break;
    }
    res = evaluate(prof, final_T, sampler, joint_limits, cart_limits, nullptr);
    accepted = res.violations.empty();
  }
  if (!accepted) {
    err_msg = "TOPP-RA could not satisfy velocity/acceleration/jerk limits (max ratios v=" +
      std::to_string(res.max_rv) + ", a=" + std::to_string(res.max_ra) +
      ", j=" + std::to_string(res.max_rj) + ")";
    return false;
  }
  if (final_T > kMaxDurationSec) {
    err_msg = "Computed trajectory duration=" + std::to_string(final_T) + " s exceeds 300 s limit";
    return false;
  }

  evaluate(prof, final_T, sampler, joint_limits, cart_limits, &out_traj);
  return true;
}

}  // namespace lite6_studio_planner
