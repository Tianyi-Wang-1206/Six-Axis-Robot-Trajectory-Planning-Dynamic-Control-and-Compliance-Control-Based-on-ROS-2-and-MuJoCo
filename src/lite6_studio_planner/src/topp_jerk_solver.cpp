#include "lite6_studio_planner/topp_jerk_solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

namespace lite6_studio_planner
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
// Sides of the inscribed polygon used in the 2nd-order TOPP-RA warm-start pass.
constexpr int kPolygonSidesRA = 16;
// Small numerical headroom (0.4 %) inside hard physical limits so 1 kHz sampling and
// floating-point evaluation stay strictly within limits without distorting optimality.
constexpr double kLimitMargin = 0.996;
constexpr double kCheckMargin = 0.999;

constexpr int kFineSamples = 1024;
constexpr int kMinFinePerPiece = 12;
constexpr int kMinGridPerPiece = 5;
constexpr int kMinGrid = 64;
constexpr int kMaxInitGrid = 300;
constexpr int kMaxRefinedGrid = 1400;
constexpr int kMaxCollocationRefines = 8;
constexpr double kMaxDurationSec = 300.0;

// Initial adaptive-grid resolution targets per stage.
constexpr double kGridJointStep = 0.020;       // rad
constexpr double kGridJointCurvature = 0.016;  // rad
constexpr double kGridTransStep = 0.012;       // m
constexpr double kGridTransCurvature = 0.012;  // m
constexpr double kGridRotStep = 0.030;         // rad
constexpr double kGridRotCurvature = 0.035;    // rad

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
    const JointPath & path, const CartesianPath * cart,
    double vs, double as, double js, const SourceLocation & loc)
  : path_(path), cart_(cart), vs_(vs), as_(as), js_(js), loc_(loc) {}

  PathPoint eval(double u) const
  {
    u = std::clamp(u, 0.0, 1.0);
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
  const CartesianPath * cart_;
  double vs_, as_, js_;
  SourceLocation loc_;
};

// ---------------- 2nd-order TOPP-RA (exact reachability warm-start) ----------------

struct LinCon
{
  double a, b, c;
};

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
  const double c = limit * std::cos(kPi / kPolygonSidesRA);
  for (int k = 0; k < kPolygonSidesRA; ++k) {
    const double phi = 2.0 * kPi * k / kPolygonSidesRA;
    out.push_back({std::cos(phi) * n1, std::cos(phi) * b1 + std::sin(phi) * b2, c});
  }
}

struct RAStage
{
  double x_cap = 0.0;
  std::vector<LinCon> cons;
};

double compute_v_max(
  const PathPoint & pp,
  const std::array<lite6_planner::JointLimit, 6> & jl,
  const lite6_planner::CartesianLimit & cl,
  double margin)
{
  double v_cap = 1e6;
  for (int j = 0; j < 6; ++j) {
    const double d1 = std::abs(pp.q1[j]);
    if (d1 > 1e-12) {
      v_cap = std::min(v_cap, pp.vs * jl[static_cast<size_t>(j)].max_vel * margin / d1);
    }
  }
  if (pp.cart) {
    const double np = pp.p1.norm();
    const double nw = pp.w1.norm();
    if (np > 1e-12) {
      v_cap = std::min(v_cap, pp.vs * cl.max_trans_vel * margin / np);
    }
    if (nw > 1e-12) {
      v_cap = std::min(v_cap, pp.vs * cl.max_rot_vel * margin / nw);
    }
  }
  return v_cap;
}

RAStage build_ra_stage(
  const PathPoint & pp,
  const std::array<lite6_planner::JointLimit, 6> & jl,
  const lite6_planner::CartesianLimit & cl,
  double margin)
{
  RAStage st;
  const double vmax = compute_v_max(pp, jl, cl, margin);
  st.x_cap = vmax * vmax;
  for (int j = 0; j < 6; ++j) {
    const double acc = pp.as * jl[static_cast<size_t>(j)].max_acc * margin;
    if (std::abs(pp.q1[j]) > 1e-12 || std::abs(pp.q2[j]) > 1e-12) {
      st.cons.push_back({pp.q1[j], pp.q2[j], acc});
      st.cons.push_back({-pp.q1[j], -pp.q2[j], acc});
    }
  }
  if (pp.cart) {
    add_norm_polygon(pp.p1, pp.p2, pp.as * cl.max_trans_acc * margin, st.cons);
    add_norm_polygon(pp.w1, pp.w2, pp.as * cl.max_rot_acc * margin, st.cons);
  }
  return st;
}

bool project_x(
  const std::vector<LinCon> & cons, const LinCon * extra, size_t n_extra,
  double lo_in, double hi_in, double & lo, double & hi)
{
  lo = lo_in;
  hi = hi_in;
  static thread_local std::vector<std::pair<double, double>> uppers, lowers;
  uppers.clear();
  lowers.clear();
  bool infeasible = false;
  auto add = [&](const LinCon & k) {
    if (k.a > 1e-14) {
      uppers.emplace_back(k.c / k.a, -k.b / k.a);
    } else if (k.a < -1e-14) {
      lowers.emplace_back(k.c / k.a, -k.b / k.a);
    } else if (k.b > 1e-14) {
      hi = std::min(hi, k.c / k.b);
    } else if (k.b < -1e-14) {
      lo = std::max(lo, k.c / k.b);
    } else if (k.c < -1e-12) {
      infeasible = true;
    }
  };
  for (const auto & k : cons) {
    add(k);
  }
  for (size_t i = 0; i < n_extra; ++i) {
    add(extra[i]);
  }
  if (infeasible) {
    return false;
  }
  for (const auto & l : lowers) {
    for (const auto & u : uppers) {
      const double coef = l.second - u.second;
      const double rhs = u.first - l.first;
      if (coef > 1e-14) {
        hi = std::min(hi, rhs / coef);
      } else if (coef < -1e-14) {
        lo = std::max(lo, rhs / coef);
      } else if (rhs < -1e-12 * (1.0 + std::abs(u.first) + std::abs(l.first))) {
        return false;
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

void sdd_range(const std::vector<LinCon> & cons, double x, double & s_lo, double & s_hi)
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

bool run_topp_ra_2nd(
  const std::vector<double> & grid, const std::vector<RAStage> & stages,
  std::vector<double> & x, double & T_ra, std::string & err)
{
  const size_t N = grid.size() - 1;
  double s_lo_end = -std::numeric_limits<double>::infinity();
  double s_hi_end = std::numeric_limits<double>::infinity();
  sdd_range(stages[N].cons, 0.0, s_lo_end, s_hi_end);

  std::vector<double> k_lo(N + 1, 0.0), k_hi(N + 1, 0.0);
  for (size_t ii = N; ii-- > 0;) {
    const double delta = grid[ii + 1] - grid[ii];
    LinCon extra[4];
    size_t n_extra = 2;
    extra[0] = {2.0 * delta, 1.0, k_hi[ii + 1]};
    extra[1] = {-2.0 * delta, -1.0, -k_lo[ii + 1]};
    if (ii + 1 == N) {
      if (std::isfinite(s_hi_end)) {
        extra[n_extra++] = {1.0, 0.0, s_hi_end};
      }
      if (std::isfinite(s_lo_end)) {
        extra[n_extra++] = {-1.0, 0.0, -s_lo_end};
      }
    }
    if (!project_x(stages[ii].cons, extra, n_extra, 0.0, stages[ii].x_cap, k_lo[ii], k_hi[ii])) {
      err = "2nd-order TOPP-RA controllable set is empty at path parameter u=" +
        std::to_string(grid[ii]);
      return false;
    }
  }
  if (k_lo[0] > 1e-9) {
    err = "2nd-order TOPP-RA: path cannot start from rest";
    return false;
  }
  x.assign(N + 1, 0.0);
  T_ra = 0.0;
  for (size_t i = 0; i < N; ++i) {
    const double delta = grid[i + 1] - grid[i];
    double s_lo = 0.0, s_hi = 0.0;
    sdd_range(stages[i].cons, x[i], s_lo, s_hi);
    if (i + 1 == N) {
      s_lo = std::max(s_lo, s_lo_end);
      s_hi = std::min(s_hi, s_hi_end);
    }
    s_hi = std::min(s_hi, (k_hi[i + 1] - x[i]) / (2.0 * delta));
    s_lo = std::max(s_lo, (k_lo[i + 1] - x[i]) / (2.0 * delta));
    const double sdd = std::max(s_lo, s_hi);
    x[i + 1] = std::max(0.0, std::clamp(x[i] + 2.0 * delta * sdd, k_lo[i + 1], k_hi[i + 1]));
    const double vsum = std::sqrt(x[i]) + std::sqrt(x[i + 1]);
    if (vsum < 1e-12) {
      err = "2nd-order TOPP-RA profile stalls at path parameter u=" + std::to_string(grid[i]);
      return false;
    }
    T_ra += 2.0 * delta / vsum;
  }
  x[N] = 0.0;
  return true;
}

// ---------------- Banded Symmetric Quasi-Definite LDL^T (Half-bandwidth B = 6) ----------------
// Interleaving stage unknowns as [v_i, a_i, dt_i, y1_i, y2_i] gives half-bandwidth 6.
// By Vanderbei's SQD theorem, a matrix with positive-definite primal block and negative-
// definite dual block factors without pivoting in O(N * B^2) with certified inertia.

class BandedSQD
{
public:
  static constexpr int kBW = 6;

  void init(int n)
  {
    n_ = n;
    data_.assign(static_cast<size_t>(n) * (kBW + 1), 0.0);
    L_.assign(static_cast<size_t>(n) * (kBW + 1), 0.0);
    D_.assign(static_cast<size_t>(n), 0.0);
  }

  void clear()
  {
    std::fill(data_.begin(), data_.end(), 0.0);
  }

  inline void add(int r, int c, double val)
  {
    if (r < c) {
      std::swap(r, c);
    }
    data_[static_cast<size_t>(c) * (kBW + 1) + static_cast<size_t>(r - c)] += val;
  }

  bool factorize(const std::vector<uint8_t> & is_primal, double delta_w, double delta_c)
  {
    constexpr double kMinPiv = 1e-13;
    L_ = data_;
    for (int j = 0; j < n_; ++j) {
      L_[static_cast<size_t>(j) * (kBW + 1)] += is_primal[static_cast<size_t>(j)] ? delta_w : -delta_c;
    }
    for (int j = 0; j < n_; ++j) {
      const size_t base_j = static_cast<size_t>(j) * (kBW + 1);
      double dj = L_[base_j];
      const int k_start = std::max(0, j - kBW);
      for (int k = k_start; k < j; ++k) {
        const double l_jk = L_[static_cast<size_t>(k) * (kBW + 1) + static_cast<size_t>(j - k)];
        dj -= l_jk * l_jk * D_[static_cast<size_t>(k)];
      }
      if (!std::isfinite(dj)) {
        return false;
      }
      const bool primal = (is_primal[static_cast<size_t>(j)] != 0);
      if (primal ? (dj <= kMinPiv) : (dj >= -kMinPiv)) {
        return false;
      }
      D_[static_cast<size_t>(j)] = dj;
      const double inv_dj = 1.0 / dj;
      const int i_end = std::min(n_ - 1, j + kBW);
      for (int i = j + 1; i <= i_end; ++i) {
        double sum = L_[base_j + static_cast<size_t>(i - j)];
        const int kk_start = std::max(0, i - kBW);
        for (int k = kk_start; k < j; ++k) {
          const size_t base_k = static_cast<size_t>(k) * (kBW + 1);
          sum -= L_[base_k + static_cast<size_t>(i - k)] *
            L_[base_k + static_cast<size_t>(j - k)] * D_[static_cast<size_t>(k)];
        }
        L_[base_j + static_cast<size_t>(i - j)] = sum * inv_dj;
      }
    }
    return true;
  }

  void solve(std::vector<double> & b) const
  {
    for (int j = 0; j < n_; ++j) {
      const double bj = b[static_cast<size_t>(j)];
      const size_t base_j = static_cast<size_t>(j) * (kBW + 1);
      const int i_end = std::min(n_ - 1, j + kBW);
      for (int i = j + 1; i <= i_end; ++i) {
        b[static_cast<size_t>(i)] -= L_[base_j + static_cast<size_t>(i - j)] * bj;
      }
    }
    for (int j = 0; j < n_; ++j) {
      b[static_cast<size_t>(j)] /= D_[static_cast<size_t>(j)];
    }
    for (int j = n_ - 1; j >= 0; --j) {
      double xj = b[static_cast<size_t>(j)];
      const size_t base_j = static_cast<size_t>(j) * (kBW + 1);
      const int i_end = std::min(n_ - 1, j + kBW);
      for (int i = j + 1; i <= i_end; ++i) {
        xj -= L_[base_j + static_cast<size_t>(i - j)] * b[static_cast<size_t>(i)];
      }
      b[static_cast<size_t>(j)] = xj;
    }
  }

private:
  int n_ = 0;
  std::vector<double> data_, L_, D_;
};

// ---------------- Exact Piecewise-Constant-Jerk Cubic Timing Profile ----------------

struct CubicTiming
{
  std::vector<double> u;
  std::vector<double> t;
  std::vector<double> v;
  std::vector<double> a;
  std::vector<double> dt;
  std::vector<double> j;

  double duration() const { return t.empty() ? 0.0 : t.back(); }

  void rebuild_from_state(
    const std::vector<double> & grid,
    const std::vector<double> & v_in,
    const std::vector<double> & a_in,
    const std::vector<double> & dt_in)
  {
    const size_t N = dt_in.size();
    u = grid;
    v = v_in;
    a = a_in;
    dt = dt_in;
    t.assign(N + 1, 0.0);
    j.assign(N, 0.0);
    for (size_t i = 0; i < N; ++i) {
      t[i + 1] = t[i] + dt[i];
      j[i] = (a[i + 1] - a[i]) / dt[i];
    }
  }

  void eval_at_time(
    double ts, double & u_out, double & v_out, double & a_out, double & j_out) const
  {
    if (ts <= 0.0) {
      u_out = 0.0;
      v_out = 0.0;
      a_out = 0.0;
      j_out = j.empty() ? 0.0 : j.front();
      return;
    }
    if (ts >= t.back()) {
      u_out = 1.0;
      v_out = 0.0;
      a_out = 0.0;
      j_out = 0.0;
      return;
    }
    auto it = std::upper_bound(t.begin(), t.end(), ts);
    size_t i = (it == t.begin()) ? 0 : static_cast<size_t>(std::distance(t.begin(), it) - 1);
    i = std::min(i, dt.size() - 1);
    const double d = std::clamp(ts - t[i], 0.0, dt[i]);
    const double d2 = d * d;
    const double d3 = d2 * d;
    u_out = std::clamp(
      u[i] + v[i] * d + 0.5 * a[i] * d2 + (1.0 / 6.0) * j[i] * d3, u[i], u[i + 1]);
    v_out = std::max(0.0, v[i] + a[i] * d + 0.5 * j[i] * d2);
    a_out = a[i] + j[i] * d;
    j_out = j[i];
  }
};

// ---------------- Precomputed Collocation Constants (Zero-Allocation IPM) ----------------

struct JointAccCon
{
  double c1 = 0.0;  // q1 / alim
  double c2 = 0.0;  // q2 / alim
};

struct CartAccCon
{
  bool active = false;
  Eigen::Vector3d w1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d w2 = Eigen::Vector3d::Zero();
  double w11 = 0.0;
  double w12 = 0.0;
  double w22 = 0.0;
};

struct JointJerkCon
{
  double c1 = 0.0;  // q1 / (jlim * tau_s)
  double c2 = 0.0;  // 3 * q2 / (jlim * tau_s)
  double c3 = 0.0;  // q3 / (jlim * tau_s)
};

struct NodeData
{
  double u = 0.0;
  PathPoint p_left;
  PathPoint p_right;
  PathPoint p_node;
  double v_max = 1e6;
};

struct MidData
{
  double u = 0.0;
  PathPoint p_mid;
  double v_max = 1e6;
};

// Precomputed normalized constraint parameters for a fixed grid and time scale tau_s.
struct ProblemCache
{
  size_t N = 0;
  std::vector<double> h;
  std::vector<double> inv_h;
  std::vector<double> v_max;
  std::vector<double> v_mid_max;
  std::vector<double> dt_min;

  struct NodeAccList
  {
    int n_joint = 0;
    JointAccCon joint[6];
    CartAccCon trans;
    CartAccCon rot;
  };
  struct IntervalJerkList
  {
    int n_left = 0;
    JointJerkCon left[6];
    int n_right = 0;
    JointJerkCon right[6];
  };

  std::vector<NodeAccList> node_acc;       // size N + 1 (interior 1..N-1 used)
  std::vector<NodeAccList> mid_acc;        // size N
  std::vector<IntervalJerkList> int_jerk;  // size N
  size_t n_ineq = 0;
};

ProblemCache build_problem_cache(
  const std::vector<NodeData> & nodes,
  const std::vector<MidData> & mids,
  const std::array<lite6_planner::JointLimit, 6> & jl,
  const lite6_planner::CartesianLimit & cl,
  double tau_s)
{
  ProblemCache pc;
  const size_t N = nodes.size() - 1;
  pc.N = N;
  pc.h.resize(N);
  pc.inv_h.resize(N);
  pc.v_max.resize(N + 1);
  pc.v_mid_max.resize(N);
  pc.dt_min.resize(N);
  pc.node_acc.resize(N + 1);
  pc.mid_acc.resize(N);
  pc.int_jerk.resize(N);

  for (size_t i = 0; i <= N; ++i) {
    pc.v_max[i] = nodes[i].v_max;
  }
  for (size_t i = 0; i < N; ++i) {
    pc.h[i] = nodes[i + 1].u - nodes[i].u;
    pc.inv_h[i] = 1.0 / pc.h[i];
    pc.v_mid_max[i] = mids[i].v_max;
    const double vmax_i = std::max(nodes[i].v_max, nodes[i + 1].v_max);
    pc.dt_min[i] = std::max(1e-7, 0.2 * pc.h[i] / std::max(vmax_i, 1e-6));
  }

  auto fill_acc = [&](const PathPoint & pp, ProblemCache::NodeAccList & dst) {
    dst.n_joint = 0;
    for (int k = 0; k < 6; ++k) {
      if (std::abs(pp.q1[k]) < 1e-12 && std::abs(pp.q2[k]) < 1e-12) {
        continue;
      }
      const double alim = pp.as * jl[static_cast<size_t>(k)].max_acc * kLimitMargin;
      dst.joint[dst.n_joint++] = {pp.q1[k] / alim, pp.q2[k] / alim};
    }
    if (pp.cart) {
      auto fill_cart = [](
        const Eigen::Vector3d & d1, const Eigen::Vector3d & d2, double alim, CartAccCon & c)
      {
        if (d1.norm() < 1e-12 && d2.norm() < 1e-12) {
          c.active = false;
          return;
        }
        c.active = true;
        c.w1 = d1 / alim;
        c.w2 = d2 / alim;
        c.w11 = c.w1.squaredNorm();
        c.w12 = c.w1.dot(c.w2);
        c.w22 = c.w2.squaredNorm();
      };
      fill_cart(pp.p1, pp.p2, pp.as * cl.max_trans_acc * kLimitMargin, dst.trans);
      fill_cart(pp.w1, pp.w2, pp.as * cl.max_rot_acc * kLimitMargin, dst.rot);
    }
  };

  size_t count = 2 * (N - 1) + N;  // simple node bounds (0 <= v_i <= v_max, dt_i >= dt_min)
  for (size_t i = 1; i < N; ++i) {
    fill_acc(nodes[i].p_node, pc.node_acc[i]);
    count += static_cast<size_t>(2 * pc.node_acc[i].n_joint);
    if (pc.node_acc[i].trans.active) {
      ++count;
    }
    if (pc.node_acc[i].rot.active) {
      ++count;
    }
  }
  for (size_t i = 0; i < N; ++i) {
    fill_acc(mids[i].p_mid, pc.mid_acc[i]);
    count += 2;  // midpoint velocity bounds
    count += static_cast<size_t>(2 * pc.mid_acc[i].n_joint);
    if (pc.mid_acc[i].trans.active) {
      ++count;
    }
    if (pc.mid_acc[i].rot.active) {
      ++count;
    }

    auto fill_jerk = [&](const PathPoint & pp, JointJerkCon out[6], int & n_out) {
      n_out = 0;
      for (int k = 0; k < 6; ++k) {
        if (std::abs(pp.q1[k]) < 1e-12 && std::abs(pp.q2[k]) < 1e-12 &&
            std::abs(pp.q3[k]) < 1e-12)
        {
          continue;
        }
        const double jlim = pp.js * jl[static_cast<size_t>(k)].max_jerk * kLimitMargin;
        const double invD = 1.0 / (jlim * tau_s);
        out[n_out++] = {pp.q1[k] * invD, 3.0 * pp.q2[k] * invD, pp.q3[k] * invD};
      }
    };
    fill_jerk(nodes[i].p_right, pc.int_jerk[i].left, pc.int_jerk[i].n_left);
    fill_jerk(nodes[i + 1].p_left, pc.int_jerk[i].right, pc.int_jerk[i].n_right);
    count += static_cast<size_t>(2 * (pc.int_jerk[i].n_left + pc.int_jerk[i].n_right));
  }
  pc.n_ineq = count;
  return pc;
}

inline int kv(size_t i) { return 3 + 5 * static_cast<int>(i - 1); }
inline int ka(size_t i) { return 3 + 5 * static_cast<int>(i - 1) + 1; }
inline int kt(size_t i) { return (i == 0) ? 0 : (3 + 5 * static_cast<int>(i - 1) + 2); }
inline int ky1(size_t i) { return (i == 0) ? 1 : (3 + 5 * static_cast<int>(i - 1) + 3); }
inline int ky2(size_t i) { return (i == 0) ? 2 : (3 + 5 * static_cast<int>(i - 1) + 4); }

// Single canonical zero-allocation inequality visitor guaranteeing identical ordering
// across value evaluation, KKT assembly, step recovery, and second-order corrections.
template <bool NeedGrad, bool NeedHess, typename Fn>
void for_each_ineq(
  const ProblemCache & pc,
  double V_s, double A_s, double tau_s,
  const std::vector<double> & v,
  const std::vector<double> & a,
  const std::vector<double> & dt,
  Fn && fn)
{
  const size_t N = pc.N;
  const double inv_Vs = 1.0 / V_s;
  const double inv_ts = 1.0 / tau_s;
  const double h_vm_at = 0.125 * A_s * tau_s;

  size_t m = 0;
  int idx[5];
  double grad[5];
  int hr[15], hc[15];
  double hv[15];

  // 1. Node simple bounds: -v_i/V_s <= 0, (v_i - v_max)/V_s <= 0, (dt_min - dt_i)/tau_s <= 0
  for (size_t i = 1; i < N; ++i) {
    idx[0] = kv(i);
    grad[0] = -1.0;
    fn(m++, -v[i] * inv_Vs, 1, idx, grad, 0, hr, hc, hv);
    grad[0] = 1.0;
    fn(m++, (v[i] - pc.v_max[i]) * inv_Vs, 1, idx, grad, 0, hr, hc, hv);
  }
  for (size_t i = 0; i < N; ++i) {
    idx[0] = kt(i);
    grad[0] = -1.0;
    fn(m++, (pc.dt_min[i] - dt[i]) * inv_ts, 1, idx, grad, 0, hr, hc, hv);
  }

  // 2. Node joint & Cartesian acceleration (interior nodes i = 1..N-1)
  for (size_t i = 1; i < N; ++i) {
    const auto & na = pc.node_acc[i];
    const int iv = kv(i), ia = ka(i);
    const double vi = v[i], ai = a[i], vi2 = vi * vi;
    idx[0] = iv;
    idx[1] = ia;
    for (int k = 0; k < na.n_joint; ++k) {
      const auto & jc = na.joint[k];
      const double f = jc.c1 * ai + jc.c2 * vi2;
      const double gv = NeedGrad ? (2.0 * jc.c2 * vi * V_s) : 0.0;
      const double ga = NeedGrad ? (jc.c1 * A_s) : 0.0;
      const double hvv = NeedHess ? (2.0 * jc.c2 * V_s * V_s) : 0.0;
      const int nh = (NeedHess && std::abs(hvv) > 1e-16) ? 1 : 0;
      hr[0] = iv;
      hc[0] = iv;

      grad[0] = gv;
      grad[1] = ga;
      hv[0] = hvv;
      fn(m++, f - 1.0, 2, idx, grad, nh, hr, hc, hv);

      grad[0] = -gv;
      grad[1] = -ga;
      hv[0] = -hvv;
      fn(m++, -f - 1.0, 2, idx, grad, nh, hr, hc, hv);
    }
    auto visit_cart_node = [&](const CartAccCon & cc) {
      if (!cc.active) {
        return;
      }
      const Eigen::Vector3d w = cc.w1 * ai + cc.w2 * vi2;
      const double val = 0.5 * (w.squaredNorm() - 1.0);
      int nh = 0;
      if constexpr (NeedGrad) {
        const double w2w = cc.w2.dot(w);
        grad[0] = 2.0 * vi * w2w * V_s;
        grad[1] = cc.w1.dot(w) * A_s;
        if constexpr (NeedHess) {
          nh = 3;
          hr[0] = iv; hc[0] = iv; hv[0] = (2.0 * w2w + 4.0 * vi2 * cc.w22) * V_s * V_s;
          hr[1] = iv; hc[1] = ia; hv[1] = (2.0 * vi * cc.w12) * V_s * A_s;
          hr[2] = ia; hc[2] = ia; hv[2] = cc.w11 * A_s * A_s;
        }
      }
      fn(m++, val, 2, idx, grad, nh, hr, hc, hv);
    };
    visit_cart_node(na.trans);
    visit_cart_node(na.rot);
  }

  // 3. Hermite-Simpson interval midpoint velocity & acceleration (i = 0..N-1)
  for (size_t i = 0; i < N; ++i) {
    const double vm = 0.5 * (v[i] + v[i + 1]) + 0.125 * (a[i] - a[i + 1]) * dt[i];
    const double am = 0.5 * (a[i] + a[i + 1]);
    const double vm2 = vm * vm;
    const auto & ma = pc.mid_acc[i];

    int nl = 0;
    double dvm[5] = {}, dam[5] = {};
    int pos_ai = -1, pos_ai1 = -1, pos_t = -1;
    if constexpr (NeedGrad) {
      if (i > 0) {
        idx[nl] = kv(i); dvm[nl] = 0.5 * V_s; dam[nl] = 0.0; ++nl;
        pos_ai = nl;
        idx[nl] = ka(i); dvm[nl] = 0.125 * dt[i] * A_s; dam[nl] = 0.5 * A_s; ++nl;
      }
      pos_t = nl;
      idx[nl] = kt(i); dvm[nl] = 0.125 * (a[i] - a[i + 1]) * tau_s; dam[nl] = 0.0; ++nl;
      if (i + 1 < N) {
        idx[nl] = kv(i + 1); dvm[nl] = 0.5 * V_s; dam[nl] = 0.0; ++nl;
        pos_ai1 = nl;
        idx[nl] = ka(i + 1); dvm[nl] = -0.125 * dt[i] * A_s; dam[nl] = 0.5 * A_s; ++nl;
      }
    }

    // 3a. Midpoint velocity bounds
    {
      int nh = 0;
      if constexpr (NeedGrad) {
        for (int q = 0; q < nl; ++q) {
          grad[q] = -dvm[q] * inv_Vs;
        }
        if constexpr (NeedHess) {
          if (pos_ai >= 0) {
            hr[nh] = idx[pos_ai]; hc[nh] = idx[pos_t]; hv[nh] = -h_vm_at * inv_Vs; ++nh;
          }
          if (pos_ai1 >= 0) {
            hr[nh] = idx[pos_ai1]; hc[nh] = idx[pos_t]; hv[nh] = h_vm_at * inv_Vs; ++nh;
          }
        }
      }
      fn(m++, -vm * inv_Vs, nl, idx, grad, nh, hr, hc, hv);

      if constexpr (NeedGrad) {
        for (int q = 0; q < nl; ++q) {
          grad[q] = -grad[q];
        }
        if constexpr (NeedHess) {
          for (int q = 0; q < nh; ++q) {
            hv[q] = -hv[q];
          }
        }
      }
      fn(m++, (vm - pc.v_mid_max[i]) * inv_Vs, nl, idx, grad, nh, hr, hc, hv);
    }

    // 3b. Midpoint joint acceleration
    for (int k = 0; k < ma.n_joint; ++k) {
      const auto & jc = ma.joint[k];
      const double f = jc.c1 * am + jc.c2 * vm2;
      int nh = 0;
      if constexpr (NeedGrad) {
        const double c2_2v = 2.0 * jc.c2 * vm;
        for (int q = 0; q < nl; ++q) {
          grad[q] = jc.c1 * dam[q] + c2_2v * dvm[q];
        }
        if constexpr (NeedHess) {
          if (std::abs(jc.c2) > 1e-15) {
            for (int p = 0; p < nl; ++p) {
              for (int q = p; q < nl; ++q) {
                double val_h = 2.0 * jc.c2 * dvm[p] * dvm[q];
                if ((p == pos_ai && q == pos_t) || (p == pos_t && q == pos_ai)) {
                  val_h += c2_2v * h_vm_at;
                }
                if ((p == pos_ai1 && q == pos_t) || (p == pos_t && q == pos_ai1)) {
                  val_h -= c2_2v * h_vm_at;
                }
                if (std::abs(val_h) > 1e-16) {
                  hr[nh] = idx[p]; hc[nh] = idx[q]; hv[nh] = val_h; ++nh;
                }
              }
            }
          }
        }
      }
      fn(m++, f - 1.0, nl, idx, grad, nh, hr, hc, hv);

      if constexpr (NeedGrad) {
        for (int q = 0; q < nl; ++q) {
          grad[q] = -grad[q];
        }
        if constexpr (NeedHess) {
          for (int q = 0; q < nh; ++q) {
            hv[q] = -hv[q];
          }
        }
      }
      fn(m++, -f - 1.0, nl, idx, grad, nh, hr, hc, hv);
    }

    // 3c. Midpoint Cartesian acceleration
    auto visit_cart_mid = [&](const CartAccCon & cc) {
      if (!cc.active) {
        return;
      }
      const Eigen::Vector3d w = cc.w1 * am + cc.w2 * vm2;
      const double val = 0.5 * (w.squaredNorm() - 1.0);
      int nh = 0;
      if constexpr (NeedGrad) {
        const double w1w = cc.w1.dot(w);
        const double w2w = cc.w2.dot(w);
        const double g_v = 2.0 * vm * w2w;
        for (int q = 0; q < nl; ++q) {
          grad[q] = w1w * dam[q] + g_v * dvm[q];
        }
        if constexpr (NeedHess) {
          const double h_vv = 2.0 * w2w + 4.0 * vm2 * cc.w22;
          const double h_av = 2.0 * vm * cc.w12;
          for (int p = 0; p < nl; ++p) {
            for (int q = p; q < nl; ++q) {
              double val_h = cc.w11 * dam[p] * dam[q] +
                h_av * (dam[p] * dvm[q] + dvm[p] * dam[q]) +
                h_vv * dvm[p] * dvm[q];
              if ((p == pos_ai && q == pos_t) || (p == pos_t && q == pos_ai)) {
                val_h += g_v * h_vm_at;
              }
              if ((p == pos_ai1 && q == pos_t) || (p == pos_t && q == pos_ai1)) {
                val_h -= g_v * h_vm_at;
              }
              if (std::abs(val_h) > 1e-16) {
                hr[nh] = idx[p]; hc[nh] = idx[q]; hv[nh] = val_h; ++nh;
              }
            }
          }
        }
      }
      fn(m++, val, nl, idx, grad, nh, hr, hc, hv);
    };
    visit_cart_mid(ma.trans);
    visit_cart_mid(ma.rot);
  }

  // 4. Endpoint joint jerk constraints at both ends of every interval i = 0..N-1
  for (size_t i = 0; i < N; ++i) {
    const int it = kt(i);
    const double dti = dt[i];
    const double da_i = a[i + 1] - a[i];
    const double dt_bar_i = dti * inv_ts;
    const bool has_i = (i > 0);
    const bool has_i1 = (i + 1 < N);

    auto visit_jerk_end = [&](const JointJerkCon list[6], int n_list, size_t p) {
      const double vp = v[p], ap = a[p];
      const double vp2 = vp * vp, vp3 = vp2 * vp;
      const bool has_p = (p > 0 && p < N);
      int nv = 0;
      if constexpr (NeedGrad) {
        if (has_p) { idx[nv++] = kv(p); }
        if (has_i) { idx[nv++] = ka(i); }
        if (has_i1) { idx[nv++] = ka(i + 1); }
        idx[nv++] = it;
      }
      for (int k = 0; k < n_list; ++k) {
        const auto & jc = list[k];
        const double S_sc = jc.c2 * vp * ap + jc.c3 * vp3;
        const double term_sc = jc.c1 * da_i + S_sc * dti;
        const double g_pos = term_sc - dt_bar_i;
        const double g_neg = -term_sc - dt_bar_i;

        double dt_term = S_sc * tau_s;
        int nh = 0;
        if constexpr (NeedGrad) {
          int pos = 0;
          if (has_p) {
            grad[pos++] = (jc.c2 * ap + 3.0 * jc.c3 * vp2) * dti * V_s;
          }
          if (has_i) {
            grad[pos++] = (-jc.c1 + (p == i ? jc.c2 * vp * dti : 0.0)) * A_s;
          }
          if (has_i1) {
            grad[pos++] = (jc.c1 + (p == i + 1 ? jc.c2 * vp * dti : 0.0)) * A_s;
          }
          grad[pos] = dt_term - 1.0;

          if constexpr (NeedHess) {
            if (has_p) {
              hr[0] = kv(p); hc[0] = kv(p); hv[0] = 6.0 * jc.c3 * vp * dti * V_s * V_s;
              hr[1] = kv(p); hc[1] = ka(p); hv[1] = jc.c2 * dti * V_s * A_s;
              hr[2] = kv(p); hc[2] = it;    hv[2] = (jc.c2 * ap + 3.0 * jc.c3 * vp2) * V_s * tau_s;
              hr[3] = ka(p); hc[3] = it;    hv[3] = jc.c2 * vp * A_s * tau_s;
              nh = 4;
            }
          }
        }
        fn(m++, g_pos, nv, idx, grad, nh, hr, hc, hv);

        if constexpr (NeedGrad) {
          for (int q = 0; q < nv - 1; ++q) {
            grad[q] = -grad[q];
          }
          grad[nv - 1] = -dt_term - 1.0;
          if constexpr (NeedHess) {
            for (int q = 0; q < nh; ++q) {
              hv[q] = -hv[q];
            }
          }
        }
        fn(m++, g_neg, nv, idx, grad, nh, hr, hc, hv);
      }
    };
    visit_jerk_end(pc.int_jerk[i].left, pc.int_jerk[i].n_left, i);
    visit_jerk_end(pc.int_jerk[i].right, pc.int_jerk[i].n_right, i + 1);
  }
}

// ---------------- Smooth Initial Guess Generator (Warm-Start ONLY) ----------------

void build_initial_guess(
  const std::vector<double> & grid,
  const std::vector<double> & x_ra,
  const std::vector<NodeData> & nodes,
  const std::array<lite6_planner::JointLimit, 6> & jl,
  std::vector<double> & v_init,
  std::vector<double> & a_init,
  std::vector<double> & dt_init)
{
  const size_t N = grid.size() - 1;
  std::vector<double> t_raw(N + 1, 0.0), v_raw(N + 1, 0.0);
  for (size_t i = 0; i <= N; ++i) {
    v_raw[i] = std::sqrt(std::max(0.0, x_ra[i]));
  }
  for (size_t i = 0; i < N; ++i) {
    const double vsum = std::max(1e-9, v_raw[i] + v_raw[i + 1]);
    t_raw[i + 1] = t_raw[i] + 2.0 * (grid[i + 1] - grid[i]) / vsum;
  }
  const double T_raw = t_raw.back();

  double t_fir = 0.015;
  for (size_t i = 0; i < N; ++i) {
    const double delta = grid[i + 1] - grid[i];
    const double sdd = (x_ra[i + 1] - x_ra[i]) / (2.0 * delta);
    const Vec6 ddq = nodes[i].p_node.q2 * x_ra[i] + nodes[i].p_node.q1 * sdd;
    for (int j = 0; j < 6; ++j) {
      const double jlim = nodes[i].p_node.js * jl[static_cast<size_t>(j)].max_jerk;
      if (jlim > 1e-6) {
        t_fir = std::max(t_fir, std::abs(ddq[j]) / jlim);
      }
    }
  }
  t_fir = std::clamp(t_fir, 0.008, std::max(0.02, 0.5 * T_raw));

  std::vector<double> U(N + 1, 0.0), W(N + 1, 0.0);
  for (size_t k = 0; k < N; ++k) {
    const double h = t_raw[k + 1] - t_raw[k];
    const double m = (h > 1e-15) ? ((v_raw[k + 1] - v_raw[k]) / h) : 0.0;
    U[k + 1] = U[k] + v_raw[k] * h + 0.5 * m * h * h;
    W[k + 1] = W[k] + U[k] * h + 0.5 * v_raw[k] * h * h + (1.0 / 6.0) * m * h * h * h;
  }
  const double corr = (U.back() > 1e-12) ? (1.0 / U.back()) : 1.0;
  for (size_t k = 0; k <= N; ++k) {
    v_raw[k] *= corr;
    U[k] *= corr;
    W[k] *= corr;
  }

  auto eval_raw = [&](double tau, double & V, double & Uv, double & Wv) {
    if (tau <= 0.0) {
      V = Uv = Wv = 0.0;
      return;
    }
    if (tau >= T_raw) {
      V = 0.0;
      Uv = U.back();
      Wv = W.back() + U.back() * (tau - T_raw);
      return;
    }
    auto it = std::upper_bound(t_raw.begin(), t_raw.end(), tau);
    size_t k = (it == t_raw.begin()) ? 0 : static_cast<size_t>(std::distance(t_raw.begin(), it) - 1);
    k = std::min(k, N - 1);
    const double h = t_raw[k + 1] - t_raw[k];
    const double d = std::clamp(tau - t_raw[k], 0.0, h);
    const double m = (h > 1e-15) ? ((v_raw[k + 1] - v_raw[k]) / h) : 0.0;
    V = v_raw[k] + m * d;
    Uv = U[k] + v_raw[k] * d + 0.5 * m * d * d;
    Wv = W[k] + U[k] * d + 0.5 * v_raw[k] * d * d + (1.0 / 6.0) * m * d * d * d;
  };

  const double T_tot = T_raw + t_fir;
  auto eval_sm = [&](double te, double & u_s, double & v_s, double & a_s) {
    if (te <= 0.0) {
      u_s = v_s = a_s = 0.0;
      return;
    }
    if (te >= T_tot) {
      u_s = 1.0;
      v_s = a_s = 0.0;
      return;
    }
    double V1, U1, W1, V0, U0, W0;
    eval_raw(te, V1, U1, W1);
    eval_raw(te - t_fir, V0, U0, W0);
    const double inv = 1.0 / t_fir;
    u_s = std::clamp((W1 - W0) * inv, 0.0, 1.0);
    v_s = std::max(0.0, (U1 - U0) * inv);
    a_s = (V1 - V0) * inv;
  };

  std::vector<double> t_node(N + 1, 0.0);
  v_init.assign(N + 1, 0.0);
  a_init.assign(N + 1, 0.0);
  t_node[N] = T_tot;
  for (size_t i = 1; i < N; ++i) {
    double lo = t_node[i - 1], hi = T_tot;
    for (int it = 0; it < 42; ++it) {
      const double mid = 0.5 * (lo + hi);
      double um, vm, am;
      eval_sm(mid, um, vm, am);
      if (um < grid[i]) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    t_node[i] = 0.5 * (lo + hi);
    double uu;
    eval_sm(t_node[i], uu, v_init[i], a_init[i]);
    v_init[i] = std::clamp(v_init[i], 1e-4 * nodes[i].v_max, 0.98 * nodes[i].v_max);
  }
  dt_init.assign(N, 0.0);
  for (size_t i = 0; i < N; ++i) {
    dt_init[i] = std::max(1e-6, t_node[i + 1] - t_node[i]);
  }

  // If extreme jerk limits make the initial guess heavily infeasible, apply one single
  // scaling pass purely to place the starting iterate near the feasible manifold for IPM.
  double max_rj = 1.0;
  for (size_t i = 0; i < N; ++i) {
    const double ji = (a_init[i + 1] - a_init[i]) / dt_init[i];
    auto check_end = [&](const PathPoint & pp, double vp, double ap) {
      for (int k = 0; k < 6; ++k) {
        const double jlim = pp.js * jl[static_cast<size_t>(k)].max_jerk;
        if (jlim > 1e-9) {
          const double jrk = pp.q1[k] * ji + 3.0 * pp.q2[k] * vp * ap + pp.q3[k] * vp * vp * vp;
          max_rj = std::max(max_rj, std::abs(jrk) / jlim);
        }
      }
    };
    check_end(nodes[i].p_right, v_init[i], a_init[i]);
    check_end(nodes[i + 1].p_left, v_init[i + 1], a_init[i + 1]);
  }
  if (max_rj > 1.5) {
    const double gamma = std::min(10.0, std::cbrt(max_rj));
    const double inv_g = 1.0 / gamma;
    const double inv_g2 = inv_g * inv_g;
    for (size_t i = 1; i < N; ++i) {
      v_init[i] *= inv_g;
      a_init[i] *= inv_g2;
    }
    for (size_t i = 0; i < N; ++i) {
      dt_init[i] *= gamma;
    }
  }
}

// ---------------- Banded Primal-Dual Interior-Point NLP Solver ----------------

struct IPMResult
{
  bool converged = false;
  double eq_inf = 0.0;
  double ineq_inf = 0.0;
  double dual_inf = 0.0;
  double compl_inf = 0.0;
};

IPMResult solve_3rd_order_ipm(
  const std::vector<NodeData> & nodes,
  const std::vector<MidData> & mids,
  const std::array<lite6_planner::JointLimit, 6> & jl,
  const lite6_planner::CartesianLimit & cl,
  std::vector<double> & v,
  std::vector<double> & a,
  std::vector<double> & dt,
  std::vector<double> & y1,
  std::vector<double> & y2,
  double T_ref,
  bool warm_polish)
{
  const size_t N = nodes.size() - 1;
  double V_s = 0.0, A_s = 0.0;
  for (size_t i = 1; i < N; ++i) {
    V_s = std::max(V_s, v[i]);
    A_s = std::max(A_s, std::abs(a[i]));
  }
  const double T0 = std::max(T_ref, 1e-4);
  V_s = std::max(V_s, 1.0 / T0);
  A_s = std::max({A_s, V_s / T0, 1e-3});
  const double tau_s = std::max(T0 / static_cast<double>(N), 1e-5);

  const ProblemCache pc = build_problem_cache(nodes, mids, jl, cl, tau_s);
  const size_t n_ineq = pc.n_ineq;
  const int n_kkt = static_cast<int>(5 * N - 2);

  std::vector<uint8_t> is_primal(static_cast<size_t>(n_kkt), 1);
  for (size_t i = 0; i < N; ++i) {
    is_primal[static_cast<size_t>(ky1(i))] = 0;
    is_primal[static_cast<size_t>(ky2(i))] = 0;
  }

  v[0] = 0.0;
  a[0] = 0.0;
  v[N] = 0.0;
  a[N] = 0.0;
  for (size_t i = 1; i < N; ++i) {
    v[i] = std::clamp(v[i], 1e-5 * pc.v_max[i], 0.996 * pc.v_max[i]);
  }
  for (size_t i = 0; i < N; ++i) {
    dt[i] = std::max(dt[i], 1.5 * pc.dt_min[i]);
  }

  std::vector<double> g_vals(n_ineq, 0.0);
  std::vector<double> g_trial(n_ineq, 0.0);
  for_each_ineq<false, false>(
    pc, V_s, A_s, tau_s, v, a, dt,
    [&](size_t m, double val, int, const int *, const double *, int, const int *, const int *, const double *) {
      g_vals[m] = val;
    });

  double mu = warm_polish ? 5e-5 : 1e-3;
  std::vector<double> s(n_ineq, 1.0), lam(n_ineq, mu);
  for (size_t m = 0; m < n_ineq; ++m) {
    s[m] = std::max(warm_polish ? 0.01 : 0.05, -g_vals[m]);
    lam[m] = std::clamp(mu / s[m], 1e-6, 10.0);
  }
  if (y1.size() != N || y2.size() != N) {
    y1.assign(N, 0.0);
    y2.assign(N, 0.0);
  }

  BandedSQD kkt;
  kkt.init(n_kkt);
  std::vector<double> rhs(static_cast<size_t>(n_kkt), 0.0);
  std::vector<double> rhs_orig(static_cast<size_t>(n_kkt), 0.0);
  std::vector<double> grad_L(static_cast<size_t>(n_kkt), 0.0);
  std::vector<double> step(static_cast<size_t>(n_kkt), 0.0);
  std::vector<double> rhs_soc(static_cast<size_t>(n_kkt), 0.0);
  std::vector<double> e1(N, 0.0), e2(N, 0.0), e1t(N, 0.0), e2t(N, 0.0);
  std::vector<double> ds(n_ineq, 0.0), dlam(n_ineq, 0.0), ds_soc(n_ineq, 0.0);
  std::vector<double> vt(N + 1, 0.0), at(N + 1, 0.0), dtt(N, 0.0), st(n_ineq, 0.0);
  std::vector<double> vt2(N + 1, 0.0), at2(N + 1, 0.0), dtt2(N, 0.0), st2(n_ineq, 0.0);

  auto eval_eq = [&](
    const std::vector<double> & vv, const std::vector<double> & aa, const std::vector<double> & dtt_in,
    std::vector<double> & o1, std::vector<double> & o2)
  {
    const double inv_Vs = 1.0 / V_s;
    for (size_t i = 0; i < N; ++i) {
      o1[i] = (vv[i + 1] - vv[i] - 0.5 * (aa[i] + aa[i + 1]) * dtt_in[i]) * inv_Vs;
      o2[i] = (0.5 * (vv[i] + vv[i + 1]) * dtt_in[i] +
               (1.0 / 12.0) * (aa[i] - aa[i + 1]) * dtt_in[i] * dtt_in[i] - pc.h[i]) * pc.inv_h[i];
    }
  };

  auto eval_filter_pair = [&](
    const std::vector<double> & dtt_in, const std::vector<double> & ss,
    const std::vector<double> & ee1, const std::vector<double> & ee2,
    const std::vector<double> & gg, double mu_b,
    double & theta, double & phi)
  {
    double T = 0.0;
    for (double d : dtt_in) {
      T += d;
    }
    theta = 0.0;
    for (size_t i = 0; i < N; ++i) {
      theta += std::abs(ee1[i]) + std::abs(ee2[i]);
    }
    double log_s = 0.0;
    for (size_t m = 0; m < n_ineq; ++m) {
      log_s += std::log(ss[m]);
      theta += std::abs(gg[m] + ss[m]);
    }
    phi = T / tau_s - mu_b * log_s;
  };

  // IPOPT-style (theta, phi) filter for robust globalization without penalty blow-up.
  std::vector<std::pair<double, double>> filter;
  filter.reserve(32);
  constexpr double kGammaTheta = 1e-5;
  constexpr double kGammaPhi = 1e-5;
  auto filter_accepts = [&](double th, double ph) -> bool {
    if (!std::isfinite(th) || !std::isfinite(ph)) {
      return false;
    }
    for (const auto & entry : filter) {
      if (th > (1.0 - kGammaTheta) * entry.first && ph > entry.second - kGammaPhi * entry.first) {
        return false;
      }
    }
    return true;
  };

  double last_delta_w = 0.0;
  double prev_T = std::numeric_limits<double>::infinity();
  IPMResult res;
  const int max_iters = warm_polish ? 80 : 120;

  for (int iter = 0; iter < max_iters; ++iter) {
    eval_eq(v, a, dt, e1, e2);

    kkt.clear();
    std::fill(rhs.begin(), rhs.end(), 0.0);
    double T_cur = 0.0;
    for (size_t i = 0; i < N; ++i) {
      rhs[static_cast<size_t>(kt(i))] += 1.0;
      T_cur += dt[i];
    }

    double eq_inf = 0.0;
    for (size_t i = 0; i < N; ++i) {
      eq_inf = std::max({eq_inf, std::abs(e1[i]), std::abs(e2[i])});
      const int iy1 = ky1(i), iy2 = ky2(i), it = kt(i);
      rhs[static_cast<size_t>(iy1)] = e1[i];
      rhs[static_cast<size_t>(iy2)] = e2[i];
      const double y1i = y1[i], y2i = y2[i];

      const double de1_dt = -0.5 * (a[i] + a[i + 1]) * tau_s / V_s;
      kkt.add(iy1, it, de1_dt);
      rhs[static_cast<size_t>(it)] += y1i * de1_dt;
      if (i > 0) {
        const int iv = kv(i), ia = ka(i);
        const double de1_da = -0.5 * dt[i] * A_s / V_s;
        kkt.add(iy1, iv, -1.0);
        kkt.add(iy1, ia, de1_da);
        rhs[static_cast<size_t>(iv)] -= y1i;
        rhs[static_cast<size_t>(ia)] += y1i * de1_da;
        kkt.add(ia, it, y1i * (-0.5 * A_s * tau_s / V_s));
      }
      if (i + 1 < N) {
        const int iv1 = kv(i + 1), ia1 = ka(i + 1);
        const double de1_da1 = -0.5 * dt[i] * A_s / V_s;
        kkt.add(iy1, iv1, 1.0);
        kkt.add(iy1, ia1, de1_da1);
        rhs[static_cast<size_t>(iv1)] += y1i;
        rhs[static_cast<size_t>(ia1)] += y1i * de1_da1;
        kkt.add(ia1, it, y1i * (-0.5 * A_s * tau_s / V_s));
      }

      const double inv_h = pc.inv_h[i];
      const double de2_dt =
        (0.5 * (v[i] + v[i + 1]) + (1.0 / 6.0) * (a[i] - a[i + 1]) * dt[i]) * tau_s * inv_h;
      kkt.add(iy2, it, de2_dt);
      rhs[static_cast<size_t>(it)] += y2i * de2_dt;
      kkt.add(it, it, y2i * ((a[i] - a[i + 1]) / 6.0) * tau_s * tau_s * inv_h);
      if (i > 0) {
        const int iv = kv(i), ia = ka(i);
        const double de2_dv = 0.5 * dt[i] * V_s * inv_h;
        const double de2_da = (1.0 / 12.0) * dt[i] * dt[i] * A_s * inv_h;
        kkt.add(iy2, iv, de2_dv);
        kkt.add(iy2, ia, de2_da);
        rhs[static_cast<size_t>(iv)] += y2i * de2_dv;
        rhs[static_cast<size_t>(ia)] += y2i * de2_da;
        kkt.add(iv, it, y2i * (0.5 * V_s * tau_s * inv_h));
        kkt.add(ia, it, y2i * (dt[i] / 6.0) * A_s * tau_s * inv_h);
      }
      if (i + 1 < N) {
        const int iv1 = kv(i + 1), ia1 = ka(i + 1);
        const double de2_dv1 = 0.5 * dt[i] * V_s * inv_h;
        const double de2_da1 = -(1.0 / 12.0) * dt[i] * dt[i] * A_s * inv_h;
        kkt.add(iy2, iv1, de2_dv1);
        kkt.add(iy2, ia1, de2_da1);
        rhs[static_cast<size_t>(iv1)] += y2i * de2_dv1;
        rhs[static_cast<size_t>(ia1)] += y2i * de2_da1;
        kkt.add(iv1, it, y2i * (0.5 * V_s * tau_s * inv_h));
        kkt.add(ia1, it, y2i * (-dt[i] / 6.0) * A_s * tau_s * inv_h);
      }
    }

    // Evaluate all inequalities, compute KKT residuals, and condense into KKT matrix in one pass.
    grad_L = rhs;
    double ineq_inf = 0.0, prim_slack_inf = 0.0, compl_mu_inf = 0.0, compl_0_inf = 0.0;
    for_each_ineq<true, true>(
      pc, V_s, A_s, tau_s, v, a, dt,
      [&](size_t m, double val, int nv, const int * idx, const double * grad,
          int nh, const int * hr, const int * hc, const double * hv)
      {
        g_vals[m] = val;
        const double sm = s[m];
        const double lm = lam[m];
        const double rp = val + sm;
        ineq_inf = std::max(ineq_inf, std::max(0.0, val));
        prim_slack_inf = std::max(prim_slack_inf, std::abs(rp));
        compl_mu_inf = std::max(compl_mu_inf, std::abs(sm * lm - mu));
        compl_0_inf = std::max(compl_0_inf, std::abs(sm * lm));

        const double sigma_m = lm / sm;
        const double coef_rhs = (mu + lm * rp) / sm;
        for (int a_i = 0; a_i < nv; ++a_i) {
          const int ri = idx[a_i];
          const double gi = grad[a_i];
          grad_L[static_cast<size_t>(ri)] += lm * gi;
          rhs[static_cast<size_t>(ri)] += coef_rhs * gi;
          for (int b_i = a_i; b_i < nv; ++b_i) {
            kkt.add(ri, idx[b_i], sigma_m * gi * grad[b_i]);
          }
        }
        for (int hh = 0; hh < nh; ++hh) {
          kkt.add(hr[hh], hc[hh], lm * hv[hh]);
        }
      });

    double dual_inf = 0.0;
    for (int k = 0; k < n_kkt; ++k) {
      if (is_primal[static_cast<size_t>(k)]) {
        dual_inf = std::max(dual_inf, std::abs(grad_L[static_cast<size_t>(k)]));
      }
    }
    double lam_sum = 0.0;
    for (double l : lam) { lam_sum += l; }
    for (double y : y1) { lam_sum += std::abs(y); }
    for (double y : y2) { lam_sum += std::abs(y); }
    const double s_d = std::max(1.0, lam_sum / (100.0 * static_cast<double>(n_ineq + 2 * N)));
    const double d_sc = dual_inf / s_d;
    const double c0_sc = compl_0_inf / s_d;
    const double E_0 = std::max({eq_inf, ineq_inf, prim_slack_inf, d_sc, c0_sc});
    const double E_mu = std::max({eq_inf, prim_slack_inf, d_sc, compl_mu_inf / s_d});

    res.eq_inf = eq_inf;
    res.ineq_inf = ineq_inf;
    res.dual_inf = d_sc;
    res.compl_inf = c0_sc;

    const double dT_rel = std::abs(T_cur - prev_T) / std::max(T_cur, 1e-6);
    if ((E_0 < 2e-5 && eq_inf < 1e-6 && ineq_inf < 1e-5) ||
        (mu <= 1e-7 && eq_inf < 1e-7 && ineq_inf < 1e-5 && c0_sc < 1e-5 &&
         d_sc < 2e-4 && dT_rel < 5e-5))
    {
      res.converged = true;
      break;
    }
    prev_T = T_cur;

    // Monotone barrier update: when barrier subproblem is solved tightly, decrease mu and
    // refresh the RHS barrier term directly before factoring.
    if (E_mu <= 20.0 * mu && mu > 1e-8) {
      const double mu_new = std::max(1e-8, std::min(0.2 * mu, std::pow(mu, 1.4)));
      const double dmu = mu_new - mu;
      for_each_ineq<true, false>(
        pc, V_s, A_s, tau_s, v, a, dt,
        [&](size_t m, double, int nv, const int * idx, const double * grad,
            int, const int *, const int *, const double *)
        {
          const double coef = dmu / s[m];
          for (int a_i = 0; a_i < nv; ++a_i) {
            rhs[static_cast<size_t>(idx[a_i])] += coef * grad[a_i];
          }
        });
      mu = mu_new;
      filter.clear();
    }

    for (int k = 0; k < n_kkt; ++k) {
      rhs_orig[static_cast<size_t>(k)] = -rhs[static_cast<size_t>(k)];
    }

    // Inertia-corrected banded SQD factorization.
    const double delta_c = 1e-8 * std::pow(mu, 0.25);
    double delta_w = 0.0;
    bool factored = kkt.factorize(is_primal, delta_w, delta_c);
    if (!factored) {
      delta_w = (last_delta_w > 0.0) ? std::max(1e-7, last_delta_w / 3.0) : 1e-5;
      while (!factored && delta_w < 1e8) {
        factored = kkt.factorize(is_primal, delta_w, delta_c);
        if (!factored) {
          delta_w *= 8.0;
        }
      }
      if (factored) {
        last_delta_w = delta_w;
      }
    } else {
      last_delta_w *= 0.5;
    }
    if (!factored) {
      break;
    }

    step = rhs_orig;
    kkt.solve(step);

    // Recover inequality primal/dual steps (ds, dlam) and fraction-to-the-boundary step bounds.
    const double tau_ftb = std::max(0.99, 1.0 - mu);
    double alpha_s_max = 1.0, alpha_l_max = 1.0;
    for_each_ineq<true, false>(
      pc, V_s, A_s, tau_s, v, a, dt,
      [&](size_t m, double val, int nv, const int * idx, const double * grad,
          int, const int *, const int *, const double *)
      {
        double g_dz = 0.0;
        for (int a_i = 0; a_i < nv; ++a_i) {
          g_dz += grad[a_i] * step[static_cast<size_t>(idx[a_i])];
        }
        const double dsm = -(val + s[m]) - g_dz;
        const double dlm = (mu - s[m] * lam[m] - lam[m] * dsm) / s[m];
        ds[m] = dsm;
        dlam[m] = dlm;
        if (dsm < 0.0) {
          alpha_s_max = std::min(alpha_s_max, -tau_ftb * s[m] / dsm);
        }
        if (dlm < 0.0) {
          alpha_l_max = std::min(alpha_l_max, -tau_ftb * lam[m] / dlm);
        }
      });

    for (size_t i = 1; i < N; ++i) {
      const double dv_i = step[static_cast<size_t>(kv(i))] * V_s;
      if (dv_i < 0.0 && v[i] > 0.0) {
        alpha_s_max = std::min(alpha_s_max, -tau_ftb * v[i] / dv_i);
      }
    }
    for (size_t i = 0; i < N; ++i) {
      const double ddt_i = step[static_cast<size_t>(kt(i))] * tau_s;
      if (ddt_i < 0.0) {
        alpha_s_max = std::min(alpha_s_max, -tau_ftb * (dt[i] - 0.5 * pc.dt_min[i]) / ddt_i);
      }
    }
    alpha_s_max = std::clamp(alpha_s_max, 0.0, 1.0);

    double theta_cur = 0.0, phi_cur = 0.0;
    eval_filter_pair(dt, s, e1, e2, g_vals, mu, theta_cur, phi_cur);
    double d_phi = 0.0;
    for (size_t i = 0; i < N; ++i) {
      d_phi += step[static_cast<size_t>(kt(i))];
    }
    for (size_t m = 0; m < n_ineq; ++m) {
      d_phi -= (mu / s[m]) * ds[m];
    }

    // Filter backtracking line search with cheap 1-step Second-Order Correction (SOC).
    double alpha = alpha_s_max;
    bool accepted = false;
    bool add_to_filter = false;

    for (int ls = 0; ls < 16; ++ls) {
      bool pos_ok = true;
      for (size_t i = 1; i < N; ++i) {
        vt[i] = v[i] + alpha * step[static_cast<size_t>(kv(i))] * V_s;
        at[i] = a[i] + alpha * step[static_cast<size_t>(ka(i))] * A_s;
      }
      for (size_t i = 0; i < N; ++i) {
        dtt[i] = dt[i] + alpha * step[static_cast<size_t>(kt(i))] * tau_s;
        if (!(dtt[i] > 0.0)) {
          pos_ok = false;
        }
      }
      for (size_t m = 0; m < n_ineq; ++m) {
        st[m] = s[m] + alpha * ds[m];
        if (!(st[m] > 0.0)) {
          pos_ok = false;
        }
      }

      if (pos_ok) {
        eval_eq(vt, at, dtt, e1t, e2t);
        for_each_ineq<false, false>(
          pc, V_s, A_s, tau_s, vt, at, dtt,
          [&](size_t m, double val, int, const int *, const double *, int, const int *, const int *, const double *) {
            g_trial[m] = val;
          });
        double theta_t = 0.0, phi_t = 0.0;
        eval_filter_pair(dtt, st, e1t, e2t, g_trial, mu, theta_t, phi_t);

        if (filter_accepts(theta_t, phi_t)) {
          const bool switching = (theta_cur <= 1e-4 && d_phi < 0.0 &&
            alpha * (-d_phi) > 1e-9 * std::pow(std::max(theta_cur, 1e-12), 1.1));
          if (switching) {
            if (phi_t <= phi_cur + 1e-4 * alpha * d_phi) {
              accepted = true;
              add_to_filter = false;
              break;
            }
          } else if (theta_t <= (1.0 - kGammaTheta) * theta_cur ||
                     phi_t <= phi_cur - kGammaPhi * theta_cur)
          {
            accepted = true;
            add_to_filter = (theta_t > (1.0 - kGammaTheta) * theta_cur);
            break;
          }
        }

        // Second-Order Correction (reuses already-factored KKT, prevents Maratos rejections).
        if (ls == 0 && theta_t > theta_cur) {
          rhs_soc = rhs_orig;
          for (size_t i = 0; i < N; ++i) {
            rhs_soc[static_cast<size_t>(ky1(i))] -= e1t[i];
            rhs_soc[static_cast<size_t>(ky2(i))] -= e2t[i];
          }
          for_each_ineq<true, false>(
            pc, V_s, A_s, tau_s, v, a, dt,
            [&](size_t m, double, int nv, const int * idx, const double * grad,
                int, const int *, const int *, const double *)
            {
              const double r_soc = g_trial[m] + st[m];
              const double coef = lam[m] * r_soc / s[m];
              for (int a_i = 0; a_i < nv; ++a_i) {
                rhs_soc[static_cast<size_t>(idx[a_i])] -= coef * grad[a_i];
              }
            });
          kkt.solve(rhs_soc);

          double a_soc = 1.0;
          for_each_ineq<true, false>(
            pc, V_s, A_s, tau_s, v, a, dt,
            [&](size_t m, double val, int nv, const int * idx, const double * grad,
                int, const int *, const int *, const double *)
            {
              double g_dz = 0.0;
              for (int a_i = 0; a_i < nv; ++a_i) {
                g_dz += grad[a_i] * rhs_soc[static_cast<size_t>(idx[a_i])];
              }
              const double dsm = -(val + s[m] + (g_trial[m] + st[m])) - g_dz;
              ds_soc[m] = dsm;
              if (dsm < 0.0) {
                a_soc = std::min(a_soc, -tau_ftb * s[m] / dsm);
              }
            });
          for (size_t i = 0; i < N; ++i) {
            const double ddt_i = rhs_soc[static_cast<size_t>(kt(i))] * tau_s;
            if (ddt_i < 0.0) {
              a_soc = std::min(a_soc, -tau_ftb * (dt[i] - 0.5 * pc.dt_min[i]) / ddt_i);
            }
          }
          a_soc = std::clamp(a_soc, 0.0, 1.0);

          bool ok2 = true;
          for (size_t i = 1; i < N; ++i) {
            vt2[i] = v[i] + a_soc * rhs_soc[static_cast<size_t>(kv(i))] * V_s;
            at2[i] = a[i] + a_soc * rhs_soc[static_cast<size_t>(ka(i))] * A_s;
          }
          for (size_t i = 0; i < N; ++i) {
            dtt2[i] = dt[i] + a_soc * rhs_soc[static_cast<size_t>(kt(i))] * tau_s;
            if (!(dtt2[i] > 0.0)) {
              ok2 = false;
            }
          }
          for (size_t m = 0; m < n_ineq; ++m) {
            st2[m] = s[m] + a_soc * ds_soc[m];
            if (!(st2[m] > 0.0)) {
              ok2 = false;
            }
          }
          if (ok2) {
            eval_eq(vt2, at2, dtt2, e1t, e2t);
            for_each_ineq<false, false>(
              pc, V_s, A_s, tau_s, vt2, at2, dtt2,
              [&](size_t m, double val, int, const int *, const double *, int, const int *, const int *, const double *) {
                g_trial[m] = val;
              });
            double theta2 = 0.0, phi2 = 0.0;
            eval_filter_pair(dtt2, st2, e1t, e2t, g_trial, mu, theta2, phi2);
            if (filter_accepts(theta2, phi2) &&
                (theta2 <= (1.0 - kGammaTheta) * theta_cur ||
                 phi2 <= phi_cur + 1e-4 * a_soc * std::min(0.0, d_phi)))
            {
              vt.swap(vt2);
              at.swap(at2);
              dtt.swap(dtt2);
              st.swap(st2);
              step.swap(rhs_soc);
              ds.swap(ds_soc);
              alpha = a_soc;
              alpha_l_max = 1.0;
              for (size_t m = 0; m < n_ineq; ++m) {
                dlam[m] = (mu - s[m] * lam[m] - lam[m] * ds[m]) / s[m];
                if (dlam[m] < 0.0) {
                  alpha_l_max = std::min(alpha_l_max, -tau_ftb * lam[m] / dlam[m]);
                }
              }
              accepted = true;
              add_to_filter = (theta2 > (1.0 - kGammaTheta) * theta_cur);
              break;
            }
          }
        }
      }
      alpha *= 0.5;
    }

    if (!accepted) {
      if (eq_inf < 1e-5 && ineq_inf < 1e-4) {
        res.converged = true;
        break;
      }
      // Feasibility restoration fallback: reset filter and take a small damped primal step.
      filter.clear();
      alpha = std::min(alpha_s_max, 0.05);
      for (size_t i = 1; i < N; ++i) {
        vt[i] = v[i] + alpha * step[static_cast<size_t>(kv(i))] * V_s;
        at[i] = a[i] + alpha * step[static_cast<size_t>(ka(i))] * A_s;
      }
      for (size_t i = 0; i < N; ++i) {
        dtt[i] = std::max(0.5 * pc.dt_min[i], dt[i] + alpha * step[static_cast<size_t>(kt(i))] * tau_s);
      }
      for (size_t m = 0; m < n_ineq; ++m) {
        st[m] = std::max(1e-10, s[m] + alpha * ds[m]);
      }
    } else if (add_to_filter) {
      filter.emplace_back(theta_cur, phi_cur);
    }

    v.swap(vt);
    a.swap(at);
    dt.swap(dtt);
    s.swap(st);

    const double alpha_l = std::clamp(alpha_l_max, 0.0, 1.0);
    for (size_t i = 0; i < N; ++i) {
      y1[i] += alpha * step[static_cast<size_t>(ky1(i))];
      y2[i] += alpha * step[static_cast<size_t>(ky2(i))];
    }
    for (size_t m = 0; m < n_ineq; ++m) {
      lam[m] = std::max(1e-12, lam[m] + alpha_l * dlam[m]);
    }
  }

  if (!res.converged && res.eq_inf < 1e-4 && res.ineq_inf < 5e-3) {
    res.converged = true;
  }
  return res;
}

// ---------------- Grid Utilities & 1 kHz Evaluation ----------------

std::vector<double> collect_boundaries(const JointPath & path, const CartesianPath * cart_path)
{
  std::vector<double> b{0.0, 1.0};
  if (cart_path != nullptr) {
    for (double u : cart_path->piece_boundaries()) {
      if (u > 1e-8 && u < 1.0 - 1e-8) {
        b.push_back(u);
      }
    }
  }
  for (double u : path.piece_boundaries()) {
    if (u > 1e-8 && u < 1.0 - 1e-8) {
      b.push_back(u);
    }
  }
  std::sort(b.begin(), b.end());
  b.erase(
    std::unique(b.begin(), b.end(), [](double x, double y) { return std::abs(x - y) < 1e-7; }),
    b.end());
  b.front() = 0.0;
  b.back() = 1.0;
  return b;
}

double boundary_ramp_du(
  const PathPoint & pp,
  const std::array<lite6_planner::JointLimit, 6> & jl,
  const lite6_planner::CartesianLimit & cl)
{
  double J0 = 1e9, A0 = 1e9;
  const double V0 = compute_v_max(pp, jl, cl, 1.0);
  for (int k = 0; k < 6; ++k) {
    const double d1 = std::abs(pp.q1[k]);
    if (d1 > 1e-12) {
      J0 = std::min(J0, pp.js * jl[static_cast<size_t>(k)].max_jerk / d1);
      A0 = std::min(A0, pp.as * jl[static_cast<size_t>(k)].max_acc / d1);
    }
  }
  if (pp.cart) {
    if (pp.p1.norm() > 1e-12) {
      A0 = std::min(A0, pp.as * cl.max_trans_acc / pp.p1.norm());
    }
    if (pp.w1.norm() > 1e-12) {
      A0 = std::min(A0, pp.as * cl.max_rot_acc / pp.w1.norm());
    }
  }
  const double tj = std::min({
    A0 / std::max(J0, 1e-9),
    std::sqrt(V0 / std::max(J0, 1e-9)),
    std::cbrt(0.5 / std::max(J0, 1e-9))});
  return std::clamp((1.0 / 6.0) * J0 * tj * tj * tj, 1e-7, 0.05);
}

void build_grid_data(
  const std::vector<double> & grid,
  const PathSampler & sampler,
  const std::array<lite6_planner::JointLimit, 6> & jl,
  const lite6_planner::CartesianLimit & cl,
  std::vector<NodeData> & nodes,
  std::vector<MidData> & mids)
{
  const size_t M = grid.size();
  nodes.resize(M);
  for (size_t i = 0; i < M; ++i) {
    nodes[i].u = grid[i];
    const double h_l = (i > 0) ? (grid[i] - grid[i - 1]) : 1.0;
    const double h_r = (i + 1 < M) ? (grid[i + 1] - grid[i]) : 1.0;
    const double eps_l = std::min(1e-9, 1e-3 * h_l);
    const double eps_r = std::min(1e-9, 1e-3 * h_r);
    nodes[i].p_left = (i > 0) ? sampler.eval(grid[i] - eps_l) : sampler.eval(0.0);
    nodes[i].p_right = (i + 1 < M) ? sampler.eval(grid[i] + eps_r) : sampler.eval(1.0);
    nodes[i].p_node = sampler.eval(grid[i]);
    nodes[i].p_node.vs = std::min({nodes[i].p_node.vs, nodes[i].p_left.vs, nodes[i].p_right.vs});
    nodes[i].p_node.as = std::min({nodes[i].p_node.as, nodes[i].p_left.as, nodes[i].p_right.as});
    nodes[i].p_node.js = std::min({nodes[i].p_node.js, nodes[i].p_left.js, nodes[i].p_right.js});
    nodes[i].v_max = std::min({
      compute_v_max(nodes[i].p_node, jl, cl, kLimitMargin),
      compute_v_max(nodes[i].p_left, jl, cl, kLimitMargin),
      compute_v_max(nodes[i].p_right, jl, cl, kLimitMargin)});
  }
  mids.resize(M - 1);
  for (size_t i = 0; i + 1 < M; ++i) {
    mids[i].u = 0.5 * (grid[i] + grid[i + 1]);
    mids[i].p_mid = sampler.eval(mids[i].u);
    mids[i].v_max = compute_v_max(mids[i].p_mid, jl, cl, kLimitMargin);
  }
}

struct TickViolation
{
  double u = 0.0;
  double ratio = 0.0;
};

struct Verify1kHzResult
{
  std::vector<TickViolation> violations;
  double max_rv = 0.0;
  double max_ra = 0.0;
  double max_rj = 0.0;
};

Verify1kHzResult evaluate_1khz(
  const CubicTiming & timing,
  const PathSampler & sampler,
  const std::array<lite6_planner::JointLimit, 6> & jl,
  const lite6_planner::CartesianLimit & cl,
  SampledTrajectory1kHz * out)
{
  Verify1kHzResult res;
  const int ticks = std::max(2, static_cast<int>(std::ceil(timing.duration() * 1000.0 - 1e-9)));
  if (out != nullptr) {
    out->samples.clear();
    out->samples.reserve(static_cast<size_t>(ticks + 1));
    out->duration_sec = static_cast<double>(ticks) * 0.001;
  }

  // Also check continuous monotonicity inside every cubic interval.
  for (size_t i = 0; i < timing.dt.size(); ++i) {
    if (timing.a[i] < 0.0 && timing.a[i + 1] > 0.0) {
      const double dj = timing.a[i + 1] - timing.a[i];
      const double v_min = timing.v[i] - 0.5 * timing.a[i] * timing.a[i] * timing.dt[i] / dj;
      if (v_min < -1e-6) {
        res.violations.push_back({0.5 * (timing.u[i] + timing.u[i + 1]), 2.0});
      }
    }
  }

  Vec6 prev_ddq = Vec6::Zero();
  double prev_js = 1.0;

  for (int tick = 0; tick <= ticks; ++tick) {
    const double t_s = static_cast<double>(tick) * 0.001;
    double u_s = 0.0, v_s = 0.0, a_s = 0.0, j_s = 0.0;
    timing.eval_at_time(t_s, u_s, v_s, a_s, j_s);
    if (tick == 0) {
      u_s = 0.0;
      v_s = a_s = j_s = 0.0;
    } else if (tick == ticks) {
      u_s = 1.0;
      v_s = a_s = j_s = 0.0;
    }

    const PathPoint pp = sampler.eval(u_s);
    const Vec6 dq = pp.q1 * v_s;
    const Vec6 ddq = pp.q2 * (v_s * v_s) + pp.q1 * a_s;
    const Vec6 jrk = pp.q3 * (v_s * v_s * v_s) + 3.0 * pp.q2 * (v_s * a_s) + pp.q1 * j_s;

    double rv = 0.0, ra = 0.0, rj = 0.0;
    const double disc_js = (tick > 0) ? std::max(prev_js, pp.js) : pp.js;
    for (int k = 0; k < 6; ++k) {
      const auto & lim = jl[static_cast<size_t>(k)];
      rv = std::max(rv, std::abs(dq[k]) / (pp.vs * lim.max_vel * kCheckMargin));
      ra = std::max(ra, std::abs(ddq[k]) / (pp.as * lim.max_acc * kCheckMargin));
      rj = std::max(rj, std::abs(jrk[k]) / (pp.js * lim.max_jerk * kCheckMargin));
      if (tick > 0) {
        const double disc_jrk = std::abs(ddq[k] - prev_ddq[k]) / 0.001;
        rj = std::max(rj, disc_jrk / (disc_js * lim.max_jerk * kCheckMargin));
      }
    }
    if (pp.cart) {
      rv = std::max(rv, (pp.p1 * v_s).norm() / (pp.vs * cl.max_trans_vel * kCheckMargin));
      rv = std::max(rv, (pp.w1 * v_s).norm() / (pp.vs * cl.max_rot_vel * kCheckMargin));
      ra = std::max(
        ra, (pp.p2 * (v_s * v_s) + pp.p1 * a_s).norm() / (pp.as * cl.max_trans_acc * kCheckMargin));
      ra = std::max(
        ra, (pp.w2 * (v_s * v_s) + pp.w1 * a_s).norm() / (pp.as * cl.max_rot_acc * kCheckMargin));
    }
    prev_ddq = ddq;
    prev_js = pp.js;
    res.max_rv = std::max(res.max_rv, rv);
    res.max_ra = std::max(res.max_ra, ra);
    res.max_rj = std::max(res.max_rj, rj);
    const double worst = std::max({rv, ra, rj});
    if (worst > 1.0) {
      res.violations.push_back({u_s, worst});
    }

    if (out != nullptr) {
      SampledTrajectory1kHz::Sample smp;
      smp.time_sec = t_s;
      smp.path_u = u_s;
      for (size_t j = 0; j < 6; ++j) {
        const int jj = static_cast<int>(j);
        smp.q[j] = pp.q[jj];
        smp.dq[j] = (tick == 0 || tick == ticks) ? 0.0 : dq[jj];
        smp.ddq[j] = (tick == 0 || tick == ticks) ? 0.0 : ddq[jj];
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
  const CartesianPath * cart_path,
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

  // 1. Fine path scan for curvature-adaptive grid construction.
  const std::vector<double> boundaries = collect_boundaries(path, cart_path);
  std::vector<double> fine_u;
  fine_u.reserve(static_cast<size_t>(kFineSamples) + boundaries.size() * kMinFinePerPiece + 2);
  for (int k = 0; k <= kFineSamples; ++k) {
    fine_u.push_back(static_cast<double>(k) / kFineSamples);
  }
  for (size_t b = 0; b + 1 < boundaries.size(); ++b) {
    const double u0 = boundaries[b];
    const double span = boundaries[b + 1] - u0;
    for (int m = 0; m <= kMinFinePerPiece; ++m) {
      fine_u.push_back(u0 + span * static_cast<double>(m) / kMinFinePerPiece);
    }
  }
  std::sort(fine_u.begin(), fine_u.end());
  fine_u.erase(
    std::unique(fine_u.begin(), fine_u.end(), [](double x, double y) { return std::abs(x - y) < 1e-10; }),
    fine_u.end());
  fine_u.front() = 0.0;
  fine_u.back() = 1.0;

  std::vector<double> density(fine_u.size(), 0.0);
  double max_motion = 0.0;
  for (size_t k = 0; k < fine_u.size(); ++k) {
    const PathPoint pp = sampler.eval(fine_u[k]);
    double rho = 0.0;
    for (int j = 0; j < 6; ++j) {
      const double d1 = std::abs(pp.q1[j]);
      rho = std::max(rho, d1 / kGridJointStep);
      rho = std::max(rho, std::sqrt(std::abs(pp.q2[j]) / kGridJointCurvature));
      max_motion = std::max(max_motion, d1);
    }
    if (pp.cart) {
      rho = std::max(rho, pp.p1.norm() / kGridTransStep);
      rho = std::max(rho, std::sqrt(pp.p2.norm() / kGridTransCurvature));
      rho = std::max(rho, pp.w1.norm() / kGridRotStep);
      rho = std::max(rho, std::sqrt(pp.w2.norm() / kGridRotCurvature));
      max_motion = std::max({max_motion, pp.p1.norm(), pp.w1.norm()});
    }
    density[k] = rho;
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

  // Adaptive grid with pinned piece boundaries and graded rest-to-rest jerk-ramp intervals.
  std::vector<double> cum(fine_u.size(), 0.0);
  for (size_t i = 0; i + 1 < fine_u.size(); ++i) {
    cum[i + 1] = cum[i] + 0.5 * (density[i] + density[i + 1]) * (fine_u[i + 1] - fine_u[i]);
  }
  const int N_density = std::clamp(
    static_cast<int>(std::ceil(cum.back())), kMinGrid, kMaxInitGrid);
  std::vector<double> grid;
  grid.reserve(static_cast<size_t>(N_density) + boundaries.size() * kMinGridPerPiece + 32);
  grid.push_back(0.0);
  {
    size_t k = 0;
    for (int i = 1; i < N_density; ++i) {
      const double target = cum.back() * static_cast<double>(i) / static_cast<double>(N_density);
      while (k + 1 < cum.size() && cum[k + 1] < target) {
        ++k;
      }
      const double span = cum[k + 1] - cum[k];
      const double frac = (span > 1e-15) ? (target - cum[k]) / span : 0.0;
      grid.push_back(fine_u[k] + frac * (fine_u[k + 1] - fine_u[k]));
    }
  }
  for (size_t b = 0; b + 1 < boundaries.size(); ++b) {
    const double u0 = boundaries[b];
    const double span = boundaries[b + 1] - u0;
    for (int m = 0; m <= kMinGridPerPiece; ++m) {
      grid.push_back(u0 + span * static_cast<double>(m) / kMinGridPerPiece);
    }
  }
  grid.push_back(1.0);

  const double du_start = boundary_ramp_du(sampler.eval(0.0), joint_limits, cart_limits);
  const double du_end = boundary_ramp_du(sampler.eval(1.0), joint_limits, cart_limits);
  for (double d = 0.5 * du_start; d < 0.02; d *= 2.2) {
    grid.push_back(d);
  }
  for (double d = 0.5 * du_end; d < 0.02; d *= 2.2) {
    grid.push_back(1.0 - d);
  }

  std::sort(grid.begin(), grid.end());
  grid.erase(
    std::unique(grid.begin(), grid.end(), [](double x, double y) { return std::abs(x - y) < 1e-8; }),
    grid.end());
  grid.front() = 0.0;
  grid.back() = 1.0;

  std::vector<NodeData> nodes;
  std::vector<MidData> mids;
  build_grid_data(grid, sampler, joint_limits, cart_limits, nodes, mids);

  // 2. Exact 2nd-order TOPP-RA warm start.
  std::vector<RAStage> ra_stages(grid.size());
  for (size_t i = 0; i < grid.size(); ++i) {
    ra_stages[i] = build_ra_stage(nodes[i].p_node, joint_limits, cart_limits, kLimitMargin);
  }
  std::vector<double> x_ra;
  double T_ra = 0.0;
  if (!run_topp_ra_2nd(grid, ra_stages, x_ra, T_ra, err_msg)) {
    return false;
  }
  if (!(T_ra > 0.0) || T_ra > kMaxDurationSec) {
    err_msg = "2nd-order TOPP-RA lower-bound duration=" + std::to_string(T_ra) +
      " s is out of range (max " + std::to_string(kMaxDurationSec) + " s)";
    return false;
  }

  std::vector<double> v, a, dt, y1, y2;
  build_initial_guess(grid, x_ra, nodes, joint_limits, v, a, dt);

  // 3. Exact 3rd-order direct-transcription interior-point solve + adaptive collocation refinement.
  CubicTiming timing;
  Verify1kHzResult check_res;
  bool accepted = false;
  IPMResult last_ipm;

  for (int refine = 0; refine < kMaxCollocationRefines; ++refine) {
    last_ipm = solve_3rd_order_ipm(
      nodes, mids, joint_limits, cart_limits, v, a, dt, y1, y2, T_ra, refine > 0);
    if (!last_ipm.converged) {
      std::ostringstream oss;
      oss << "3rd-order TOPP interior-point solver failed to converge (eq_inf="
          << last_ipm.eq_inf << ", ineq_inf=" << last_ipm.ineq_inf
          << ", dual_inf=" << last_ipm.dual_inf << ")";
      err_msg = oss.str();
      return false;
    }

    timing.rebuild_from_state(grid, v, a, dt);
    if (!(timing.duration() > 0.0) || !std::isfinite(timing.duration())) {
      err_msg = "3rd-order TOPP produced invalid trajectory duration";
      return false;
    }
    if (timing.duration() > kMaxDurationSec) {
      err_msg = "Computed trajectory duration=" + std::to_string(timing.duration()) +
        " s exceeds 300 s limit";
      return false;
    }

    check_res = evaluate_1khz(timing, sampler, joint_limits, cart_limits, nullptr);
    if (check_res.violations.empty()) {
      accepted = true;
      break;
    }

    // Adaptive collocation mesh refinement: insert knots at offending intervals and warm-start
    // from the exact analytic cubic profile + transferred equality multipliers.
    if (static_cast<int>(grid.size()) - 1 >= kMaxRefinedGrid) {
      break;
    }
    const size_t N_old = grid.size() - 1;
    std::vector<std::pair<double, double>> best_in_seg(N_old, {-1.0, 0.0});
    for (const auto & vio : check_res.violations) {
      auto it = std::upper_bound(grid.begin(), grid.end(), vio.u);
      size_t idx = (it == grid.begin()) ? 0 : static_cast<size_t>(std::distance(grid.begin(), it) - 1);
      idx = std::min(idx, N_old - 1);
      if (vio.ratio > best_in_seg[idx].second) {
        best_in_seg[idx] = {vio.u, vio.ratio};
      }
    }
    std::vector<double> new_u;
    for (size_t idx = 0; idx < N_old; ++idx) {
      if (best_in_seg[idx].second > 0.0) {
        const double ua = grid[idx], ub = grid[idx + 1];
        double um = best_in_seg[idx].first;
        if (um - ua < 0.15 * (ub - ua) || ub - um < 0.15 * (ub - ua)) {
          um = 0.5 * (ua + ub);
        }
        new_u.push_back(um);
      }
    }
    if (new_u.empty()) {
      break;
    }

    std::vector<double> grid_next = grid;
    grid_next.insert(grid_next.end(), new_u.begin(), new_u.end());
    std::sort(grid_next.begin(), grid_next.end());
    grid_next.erase(
      std::unique(grid_next.begin(), grid_next.end(), [](double x, double y) { return std::abs(x - y) < 1e-8; }),
      grid_next.end());
    grid_next.front() = 0.0;
    grid_next.back() = 1.0;

    const size_t M_new = grid_next.size();
    std::vector<double> t_next(M_new, 0.0);
    std::vector<double> v_next(M_new, 0.0);
    std::vector<double> a_next(M_new, 0.0);
    for (size_t k = 1; k + 1 < M_new; ++k) {
      auto it = std::upper_bound(timing.u.begin(), timing.u.end(), grid_next[k]);
      size_t i = (it == timing.u.begin()) ? 0 : static_cast<size_t>(std::distance(timing.u.begin(), it) - 1);
      i = std::min(i, timing.dt.size() - 1);
      double lo = 0.0, hi = timing.dt[i];
      for (int bs = 0; bs < 42; ++bs) {
        const double mid = 0.5 * (lo + hi);
        const double um = timing.u[i] + timing.v[i] * mid + 0.5 * timing.a[i] * mid * mid +
          (1.0 / 6.0) * timing.j[i] * mid * mid * mid;
        if (um < grid_next[k]) {
          lo = mid;
        } else {
          hi = mid;
        }
      }
      const double d = 0.5 * (lo + hi);
      t_next[k] = timing.t[i] + d;
      v_next[k] = std::max(0.0, timing.v[i] + timing.a[i] * d + 0.5 * timing.j[i] * d * d);
      a_next[k] = timing.a[i] + timing.j[i] * d;
    }
    t_next.back() = timing.duration();

    std::vector<double> dt_next(M_new - 1, 0.0);
    std::vector<double> y1_next(M_new - 1, 0.0);
    std::vector<double> y2_next(M_new - 1, 0.0);
    for (size_t k = 0; k + 1 < M_new; ++k) {
      dt_next[k] = std::max(1e-7, t_next[k + 1] - t_next[k]);
      const double um = 0.5 * (grid_next[k] + grid_next[k + 1]);
      auto it = std::upper_bound(grid.begin(), grid.end(), um);
      size_t p_idx = (it == grid.begin()) ? 0 : static_cast<size_t>(std::distance(grid.begin(), it) - 1);
      p_idx = std::min(p_idx, N_old - 1);
      y1_next[k] = y1[p_idx];
      y2_next[k] = y2[p_idx];
    }

    grid.swap(grid_next);
    v.swap(v_next);
    a.swap(a_next);
    dt.swap(dt_next);
    y1.swap(y1_next);
    y2.swap(y2_next);
    build_grid_data(grid, sampler, joint_limits, cart_limits, nodes, mids);
  }

  if (!accepted) {
    err_msg = "3rd-order TOPP solver could not satisfy velocity/acceleration/jerk limits at 1 kHz "
      "(max ratios v=" + std::to_string(check_res.max_rv) +
      ", a=" + std::to_string(check_res.max_ra) +
      ", j=" + std::to_string(check_res.max_rj) + ")";
    return false;
  }

  const double final_1khz_T = std::ceil(timing.duration() * 1000.0 - 1e-9) * 0.001;
  if (final_1khz_T > kMaxDurationSec) {
    err_msg = "Computed trajectory duration=" + std::to_string(final_1khz_T) +
      " s exceeds 300 s limit";
    return false;
  }

  evaluate_1khz(timing, sampler, joint_limits, cart_limits, &out_traj);
  return true;
}

}  // namespace lite6_studio_planner
