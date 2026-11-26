#include "lite6_studio_planner/cartesian_collocation_solver.hpp"

#include "lite6_kinematics/motion_tolerances.hpp"
#include "lite6_studio_planner/joint_pva_solver.hpp"

#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace lite6_studio_planner
{
namespace
{

using D = lite6_interfaces::msg::CompileDiagnostic;
using IKResult = lite6_kinematics::IKResult;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Mat3x6 = Eigen::Matrix<double, 3, 6>;

constexpr double kCharLength = 0.25;  // Matches KinematicsEngine::check_singularity
constexpr double kTwoPi = 6.2831853071795864769;
constexpr double kSingEps = 1e-10;
constexpr int kJoints = 6;
constexpr int kNodePrimal = 18;   // q(6), v(6), a(6)
constexpr int kIntervalDual = 12; // lam_q(6), lam_v(6)
constexpr int kBlockStride = 30;  // 18 + 12
constexpr int kBandWidth = 30;    // Exact half-bandwidth of K_band

void add_diag(
  std::vector<D> & diags,
  uint8_t stage,
  const SourceLocation & loc,
  const std::string & code,
  const std::string & msg)
{
  D d;
  d.severity = D::SEVERITY_ERROR;
  d.stage = stage;
  d.line_number = loc.line;
  d.column_number = loc.column;
  d.statement_index = loc.statement_index;
  d.command_text = loc.text;
  d.code = code;
  d.message = msg;
  diags.push_back(std::move(d));
}

void add_warning(
  std::vector<D> & diags,
  uint8_t stage,
  const SourceLocation & loc,
  const std::string & code,
  const std::string & msg)
{
  D d;
  d.severity = D::SEVERITY_WARNING;
  d.stage = stage;
  d.line_number = loc.line;
  d.column_number = loc.column;
  d.statement_index = loc.statement_index;
  d.command_text = loc.text;
  d.code = code;
  d.message = msg;
  diags.push_back(std::move(d));
}

std::string ik_error_code(IKResult res)
{
  switch (res) {
    case IKResult::ERR_SINGULARITY: return "ERR_SINGULARITY";
    case IKResult::ERR_QUADRANT_JUMP: return "ERR_QUADRANT_JUMP";
    case IKResult::ERR_LIMIT_OR_COLLISION: return "ERR_IK_LIMIT_OR_COLLISION";
    default: return "ERR_NO_IK_SOLUTION";
  }
}

Vec6 to_vec6(const std::array<double, 6> & a)
{
  Vec6 v;
  for (int i = 0; i < 6; ++i) {
    v[i] = a[static_cast<size_t>(i)];
  }
  return v;
}

std::array<double, 6> to_array(const Vec6 & v)
{
  std::array<double, 6> a{};
  for (int i = 0; i < 6; ++i) {
    a[static_cast<size_t>(i)] = v[i];
  }
  return a;
}

double rot_angle(const Eigen::Matrix3d & a, const Eigen::Matrix3d & b)
{
  return std::abs(Eigen::AngleAxisd(a.transpose() * b).angle());
}

// Newton refinement onto an exact SE(3) pose while staying on the same IK branch.
bool newton_ik(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  pinocchio::FrameIndex tcp,
  const Eigen::Matrix4d & target,
  const Vec6 & q_guess,
  Vec6 & q_out)
{
  const Eigen::Vector3d p_t = target.topRightCorner<3, 1>();
  const Eigen::Matrix3d r_t = target.topLeftCorner<3, 3>();
  Vec6 q = q_guess;
  pinocchio::Data::Matrix6x J(6, 6);
  for (int it = 0; it < 10; ++it) {
    pinocchio::forwardKinematics(model, data, q);
    const auto & frame = pinocchio::updateFramePlacement(model, data, tcp);
    Vec6 e;
    e.head<3>() = p_t - frame.translation();
    e.tail<3>() = pinocchio::log3(Eigen::Matrix3d(r_t * frame.rotation().transpose()));
    if (e.norm() < 1e-13) {
      break;
    }
    J.setZero();
    pinocchio::computeFrameJacobian(model, data, q, tcp, pinocchio::LOCAL_WORLD_ALIGNED, J);
    const Eigen::FullPivLU<Mat6> lu(J);
    if (!lu.isInvertible()) {
      return false;
    }
    Vec6 dq = lu.solve(e);
    if (!dq.allFinite()) {
      return false;
    }
    const double max_step = dq.cwiseAbs().maxCoeff();
    if (max_step > 0.1) {
      dq *= 0.1 / max_step;
    }
    q += dq;
  }
  pinocchio::forwardKinematics(model, data, q);
  const auto & frame = pinocchio::updateFramePlacement(model, data, tcp);
  const double p_err = (p_t - frame.translation()).norm();
  const double r_err = pinocchio::log3(Eigen::Matrix3d(r_t * frame.rotation().transpose())).norm();
  q_out = q;
  return p_err < 1e-9 && r_err < 1e-8 && (q - q_guess).cwiseAbs().maxCoeff() < 0.08;
}

// Converts LOCAL_WORLD_ALIGNED Cartesian twist V and optional classical acceleration A
// at joint configuration q into joint velocity dq and acceleration ddq.
bool twist_acc_to_joint(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  pinocchio::FrameIndex tcp,
  const Vec6 & q,
  const Vec6 & V,
  const Vec6 * A,
  double sing_thresh,
  Vec6 & dq_out,
  Vec6 * ddq_out,
  double * sigma_out = nullptr)
{
  pinocchio::Data::Matrix6x J(6, 6);
  J.setZero();
  pinocchio::computeFrameJacobian(model, data, q, tcp, pinocchio::LOCAL_WORLD_ALIGNED, J);
  Mat6 Jn = J;
  Jn.topRows<3>() /= kCharLength;
  const Eigen::JacobiSVD<Mat6> svd(Jn);
  const double sigma = svd.singularValues()[5];
  if (sigma_out) {
    *sigma_out = sigma;
  }
  if (!std::isfinite(sigma) || sigma < sing_thresh) {
    return false;
  }
  const Mat6 J6 = J;
  const Eigen::FullPivLU<Mat6> lu(J6);
  if (!lu.isInvertible()) {
    return false;
  }
  dq_out = lu.solve(V);
  if (!dq_out.allFinite()) {
    return false;
  }
  if (A != nullptr && ddq_out != nullptr) {
    pinocchio::Data::Matrix6x dJ(6, 6);
    dJ.setZero();
    pinocchio::computeJointJacobiansTimeVariation(model, data, q, dq_out);
    pinocchio::updateFramePlacement(model, data, tcp);
    pinocchio::getFrameJacobianTimeVariation(model, data, tcp, pinocchio::LOCAL_WORLD_ALIGNED, dJ);
    *ddq_out = lu.solve(*A - dJ * dq_out);
    if (!ddq_out->allFinite()) {
      return false;
    }
  }
  return true;
}

// ============================================================================
// Exact CartesianPath wrapper over a solved piecewise-cubic joint trajectory
// ============================================================================

class CollocationCartesianPath : public CartesianPath
{
public:
  CollocationCartesianPath(
    pinocchio::Model model,
    pinocchio::FrameIndex tcp_frame_id,
    std::vector<double> knot_times,
    std::vector<Vec6> q_knots,
    std::vector<Vec6> v_knots,
    std::vector<Vec6> a_knots,
    std::vector<size_t> wp_node_indices,
    std::vector<SourceLocation> seg_locs,
    MotionOptions options)
  : model_(std::move(model)),
    tcp_frame_id_(tcp_frame_id),
    knot_times_(std::move(knot_times)),
    q_knots_(std::move(q_knots)),
    v_knots_(std::move(v_knots)),
    a_knots_(std::move(a_knots)),
    wp_node_indices_(std::move(wp_node_indices)),
    seg_locs_(std::move(seg_locs)),
    options_(options)
  {
    pinocchio::Data data(model_);
    const double T = total_duration();
    const int n_int = std::clamp(static_cast<int>(std::ceil(T * 120.0)), 32, 800);
    const double dt = T / static_cast<double>(n_int);
    double len = 0.0;
    for (int k = 0; k < n_int; ++k) {
      Vec6 q, v, a;
      eval_cubic_at_time((k + 0.5) * dt, q, v, a);
      pinocchio::forwardKinematics(model_, data, q, v);
      pinocchio::updateFramePlacement(model_, data, tcp_frame_id_);
      const auto vel = pinocchio::getFrameVelocity(
        model_, data, tcp_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED);
      len += dt * std::hypot(vel.linear().norm(), kCharLength * vel.angular().norm());
    }
    total_length_ = std::max(len, 1e-6);
  }

  double total_length() const override { return total_length_; }

  CartesianSample eval_normalized(double u) const override
  {
    const double T = total_duration();
    const double t = std::clamp(u, 0.0, 1.0) * T;
    Vec6 q, v, a;
    const size_t interval = eval_cubic_at_time(t, q, v, a);

    pinocchio::Data data(model_);
    pinocchio::forwardKinematics(model_, data, q, v, a);
    const auto & oMf = pinocchio::updateFramePlacement(model_, data, tcp_frame_id_);
    const auto vel = pinocchio::getFrameVelocity(
      model_, data, tcp_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED);
    const auto acc = pinocchio::getFrameClassicalAcceleration(
      model_, data, tcp_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED);

    CartesianSample smp;
    smp.pose = Eigen::Matrix4d::Identity();
    smp.pose.topLeftCorner<3, 3>() = oMf.rotation();
    smp.pose.topRightCorner<3, 1>() = oMf.translation();
    smp.p_s = vel.linear() * T;
    smp.w_s = vel.angular() * T;
    smp.p_ss = acc.linear() * (T * T);
    smp.w_ss = acc.angular() * (T * T);
    smp.p_sss.setZero();
    smp.vel_scale = options_.vel_scale;
    smp.acc_scale = options_.acc_scale;
    smp.jerk_scale = options_.jerk_scale;

    size_t seg_idx = 0;
    for (size_t s = 0; s + 1 < wp_node_indices_.size(); ++s) {
      if (interval >= wp_node_indices_[s] && interval < wp_node_indices_[s + 1]) {
        seg_idx = s;
        break;
      }
      seg_idx = s;
    }
    if (!seg_locs_.empty()) {
      smp.loc = seg_locs_[std::min(seg_idx, seg_locs_.size() - 1)];
    }
    return smp;
  }

  std::vector<double> piece_boundaries() const override
  {
    std::vector<double> out;
    const double T = total_duration();
    if (T <= 1e-12) {
      return out;
    }
    for (size_t k = 1; k + 1 < wp_node_indices_.size(); ++k) {
      const double u = knot_times_[wp_node_indices_[k]] / T;
      if (u > 1e-9 && u < 1.0 - 1e-9) {
        out.push_back(u);
      }
    }
    return out;
  }

private:
  double total_duration() const
  {
    return knot_times_.empty() ? 0.0 : std::max(1e-9, knot_times_.back());
  }

  size_t eval_cubic_at_time(double t, Vec6 & q, Vec6 & v, Vec6 & a) const
  {
    const size_t N = knot_times_.size() - 1;
    const double tc = std::clamp(t, 0.0, knot_times_.back());
    auto it = std::upper_bound(knot_times_.begin() + 1, knot_times_.end() - 1, tc);
    const size_t i = static_cast<size_t>(it - knot_times_.begin() - 1);
    const double h = std::max(1e-12, knot_times_[i + 1] - knot_times_[i]);
    const double tau = std::clamp(tc - knot_times_[i], 0.0, h);
    const Vec6 jerk = (a_knots_[i + 1] - a_knots_[i]) / h;
    a = a_knots_[i] + jerk * tau;
    v = v_knots_[i] + a_knots_[i] * tau + (0.5 * tau * tau) * jerk;
    q = q_knots_[i] + v_knots_[i] * tau + (0.5 * tau * tau) * a_knots_[i] +
      (tau * tau * tau / 6.0) * jerk;
    return std::min(i, N - 1);
  }

  pinocchio::Model model_;
  pinocchio::FrameIndex tcp_frame_id_;
  std::vector<double> knot_times_;
  std::vector<Vec6> q_knots_;
  std::vector<Vec6> v_knots_;
  std::vector<Vec6> a_knots_;
  std::vector<size_t> wp_node_indices_;
  std::vector<SourceLocation> seg_locs_;
  MotionOptions options_;
  double total_length_ = 1e-6;
};

// ============================================================================
// Vanderbei Banded SQD LDL^T Solver + Exact Schur Complement for Free T_s
// ============================================================================

class BorderedBandedKKT
{
public:
  void init(size_t n_band, size_t n_border, const std::vector<bool> & is_primal)
  {
    n_band_ = n_band;
    n_border_ = n_border;
    is_primal_ = is_primal;
    band_.assign(n_band_ * (kBandWidth + 1), 0.0);
    L_work_.resize(n_band_ * (kBandWidth + 1));
    D_work_.resize(n_band_);
    B_.assign(n_border_, std::vector<double>(n_band_, 0.0));
    U_work_.assign(n_border_, std::vector<double>(n_band_, 0.0));
    B_min_row_.assign(n_border_, n_band_);
    B_max_row_.assign(n_border_, 0);
    D_T_.assign(n_border_, 0.0);
  }

  void clear()
  {
    std::fill(band_.begin(), band_.end(), 0.0);
    for (auto & col : B_) {
      std::fill(col.begin(), col.end(), 0.0);
    }
    std::fill(B_min_row_.begin(), B_min_row_.end(), n_band_);
    std::fill(B_max_row_.begin(), B_max_row_.end(), 0);
    std::fill(D_T_.begin(), D_T_.end(), 0.0);
  }

  inline void add_band(size_t r, size_t c, double val)
  {
    if (r > c) {
      std::swap(r, c);
    }
    const size_t diff = c - r;
    if (diff <= static_cast<size_t>(kBandWidth)) {
      band_[r * (kBandWidth + 1) + diff] += val;
    }
  }

  inline void set_band_diag(size_t r, double val)
  {
    band_[r * (kBandWidth + 1)] = val;
  }

  inline void add_border(size_t r_band, size_t c_border, double val)
  {
    B_[c_border][r_band] += val;
    if (r_band < B_min_row_[c_border]) {
      B_min_row_[c_border] = r_band;
    }
    if (r_band > B_max_row_[c_border]) {
      B_max_row_[c_border] = r_band;
    }
  }

  inline void add_T_diag(size_t c_border, double val)
  {
    D_T_[c_border] += val;
  }

  // Copies matrix state into a factorization copy, adds inertia regularization (delta_w, delta_c),
  // factors K_band = L D L^T with exact SQD inertia check, then forms and Cholesky-factors
  // the Schur complement S = D_T - B^T K_band^{-1} B (exact inertia by Sylvester's law).
  bool factor_and_solve(
    const std::vector<double> & rhs_z,
    const std::vector<double> & rhs_T,
    const std::vector<bool> & is_fixed,
    double delta_w,
    double delta_c,
    std::vector<double> & out_dz,
    std::vector<double> & out_dT) const
  {
    L_work_.assign(band_.begin(), band_.end());
    auto & L = L_work_;
    auto & D = D_work_;

    // Apply primal/dual regularization on free variables and enforce identity on fixed variables.
    for (size_t k = 0; k < n_band_; ++k) {
      if (is_primal_[k]) {
        if (is_fixed[k]) {
          L[k * (kBandWidth + 1)] = 1.0;
        } else {
          L[k * (kBandWidth + 1)] += delta_w;
        }
      } else {
        L[k * (kBandWidth + 1)] -= delta_c;
      }
    }

    // In-place upper-banded LDL^T factorization with exact SQD pivot sign enforcement.
    constexpr double kMinPivot = 1e-13;
    for (size_t k = 0; k < n_band_; ++k) {
      double * row_k = &L[k * (kBandWidth + 1)];
      const double dk = row_k[0];
      if (!std::isfinite(dk)) {
        return false;
      }
      if (is_primal_[k]) {
        if (dk <= kMinPivot) {
          return false;
        }
      } else {
        if (dk >= -kMinPivot) {
          return false;
        }
      }
      D[k] = dk;
      const double inv_dk = 1.0 / dk;
      const size_t j_max = std::min(n_band_ - 1, k + static_cast<size_t>(kBandWidth));
      for (size_t j = k + 1; j <= j_max; ++j) {
        const double v_j = row_k[j - k];
        if (v_j == 0.0) {
          continue;
        }
        const double l_jk = v_j * inv_dk;
        double * row_j = &L[j * (kBandWidth + 1)];
        for (size_t l = j; l <= j_max; ++l) {
          row_j[l - j] -= l_jk * row_k[l - k];
        }
        row_k[j - k] = l_jk;
      }
    }

    auto solve_band = [&](const std::vector<double> & b, std::vector<double> & x, size_t start_row) {
      x.assign(b.begin(), b.end());
      // Forward substitution: L^T stored in upper band rows -> L y = b
      for (size_t k = start_row; k < n_band_; ++k) {
        const double xk = x[k];
        if (xk == 0.0) {
          continue;
        }
        const double * row_k = &L[k * (kBandWidth + 1)];
        const size_t j_max = std::min(n_band_ - 1, k + static_cast<size_t>(kBandWidth));
        for (size_t j = k + 1; j <= j_max; ++j) {
          x[j] -= row_k[j - k] * xk;
        }
      }
      // Diagonal scaling
      for (size_t k = start_row; k < n_band_; ++k) {
        x[k] /= D[k];
      }
      // Backward substitution: L^T x = y
      for (size_t k = n_band_; k-- > 0;) {
        const double * row_k = &L[k * (kBandWidth + 1)];
        const size_t j_max = std::min(n_band_ - 1, k + static_cast<size_t>(kBandWidth));
        double sum = x[k];
        for (size_t j = k + 1; j <= j_max; ++j) {
          sum -= row_k[j - k] * x[j];
        }
        x[k] = sum;
      }
    };

    solve_band(rhs_z, out_dz, 0);
    out_dT.assign(n_border_, 0.0);
    if (n_border_ == 0) {
      return true;
    }

    // Solve K_band * U_c = B_c for each free segment duration.
    for (size_t c = 0; c < n_border_; ++c) {
      solve_band(B_[c], U_work_[c], std::min(B_min_row_[c], n_band_));
    }

    // Form symmetric Schur complement S = (D_T + delta_w I) - B^T U using exact column support.
    Eigen::MatrixXd S(static_cast<Eigen::Index>(n_border_), static_cast<Eigen::Index>(n_border_));
    Eigen::VectorXd rhs_s(static_cast<Eigen::Index>(n_border_));
    for (size_t a = 0; a < n_border_; ++a) {
      const size_t k_lo = B_min_row_[a];
      const size_t k_hi = (k_lo < n_band_) ? std::min(n_band_ - 1, B_max_row_[a]) : 0;
      double dot_rhs = 0.0;
      const auto & Ba = B_[a];
      if (k_lo < n_band_) {
        for (size_t k = k_lo; k <= k_hi; ++k) {
          dot_rhs += Ba[k] * out_dz[k];
        }
      }
      rhs_s[static_cast<Eigen::Index>(a)] = rhs_T[a] - dot_rhs;

      for (size_t b = a; b < n_border_; ++b) {
        double dot_u = 0.0;
        const auto & Ub = U_work_[b];
        if (k_lo < n_band_) {
          for (size_t k = k_lo; k <= k_hi; ++k) {
            dot_u += Ba[k] * Ub[k];
          }
        }
        const double val = ((a == b) ? (D_T_[a] + delta_w) : 0.0) - dot_u;
        S(static_cast<Eigen::Index>(a), static_cast<Eigen::Index>(b)) = val;
        S(static_cast<Eigen::Index>(b), static_cast<Eigen::Index>(a)) = val;
      }
    }

    // Exact inertia check: S must be strictly positive definite.
    const Eigen::LLT<Eigen::MatrixXd> llt(S);
    if (llt.info() != Eigen::Success) {
      return false;
    }
    const Eigen::VectorXd dT_vec = llt.solve(rhs_s);
    if (!dT_vec.allFinite()) {
      return false;
    }
    for (size_t c = 0; c < n_border_; ++c) {
      out_dT[c] = dT_vec[static_cast<Eigen::Index>(c)];
      const double dtc = out_dT[c];
      const auto & Uc = U_work_[c];
      for (size_t k = 0; k < n_band_; ++k) {
        out_dz[k] -= Uc[k] * dtc;
      }
    }
    return true;
  }

private:
  size_t n_band_ = 0;
  size_t n_border_ = 0;
  std::vector<bool> is_primal_;
  std::vector<double> band_;
  mutable std::vector<double> L_work_;
  mutable std::vector<double> D_work_;
  std::vector<std::vector<double>> B_;
  mutable std::vector<std::vector<double>> U_work_;
  std::vector<size_t> B_min_row_;
  std::vector<size_t> B_max_row_;
  std::vector<double> D_T_;
};

// ============================================================================
// Global IK Branch Selection (Dynamic Programming Over Branches & 2pi Wraps)
// ============================================================================

struct ResolvedWaypointChain
{
  std::vector<Vec6> q;          // size M + 1
  std::vector<Vec6> dq_pin;     // size M + 1 (valid where has_v[k])
  std::vector<Vec6> ddq_pin;    // size M + 1 (valid where has_a[k])
  std::vector<bool> has_v;      // size M + 1 (true at 0 and M)
  std::vector<bool> has_a;      // size M + 1 (true at 0 and M)
  double min_sigma_smooth = 0.1;
  double cost = 0.0;
};

struct IKBranchCandidate
{
  Vec6 q;
  Vec6 dq;
  Vec6 ddq;
  double sigma_min = 0.0;
  double sigma_smooth = 0.0;
};

// Returns up to max_chains continuous, limit-feasible, collision-free IK waypoint chains.
bool resolve_ik_chains_dp(
  const lite6_kinematics::KinematicsEngine & engine,
  const Vec6 & start_q,
  const std::vector<CartesianCollocationWaypoint> & waypoints,
  const std::array<lite6_kinematics::JointLimit, 6> & limits,
  const lite6_kinematics::CartesianLimit & cart_limits,
  const MotionOptions & opts,
  size_t max_chains,
  std::vector<ResolvedWaypointChain> & out_chains,
  std::vector<D> & out_diagnostics,
  const std::function<bool()> & is_cancelled = {})
{
  const size_t M = waypoints.size();
  if (is_cancelled && is_cancelled()) {
    add_diag(
      out_diagnostics, D::STAGE_PLANNING,
      waypoints.empty() ? SourceLocation{} : waypoints.front().loc,
      "ERR_CANCELLED", "Motion cancelled by supervisor");
    return false;
  }
  const auto & model = engine.get_pin_model();
  const auto tcp = engine.tcp_frame_id();
  const double sing_thresh = engine.get_singularity_threshold();
  const double vs = opts.vel_scale;
  const double as = opts.acc_scale;
  const double v_trans_lim = cart_limits.max_trans_vel * vs;
  const double v_rot_lim = cart_limits.max_rot_vel * vs;
  const double a_trans_lim = cart_limits.max_trans_acc * as;
  const double a_rot_lim = cart_limits.max_rot_acc * as;

  auto workspace = engine.collision_workspace();

  // Validate waypoint Cartesian limits up front.
  for (size_t k = 0; k < M; ++k) {
    if (is_cancelled && is_cancelled()) {
      add_diag(
        out_diagnostics, D::STAGE_PLANNING, waypoints[k].loc,
        "ERR_CANCELLED", "Motion cancelled by supervisor");
      return false;
    }
    const auto & wp = waypoints[k];
    if (k + 1 < M && wp.acceleration.has_value() && !wp.velocity.has_value()) {
      add_diag(
        out_diagnostics, D::STAGE_PLANNING, wp.loc, "ERR_PVA_ACC_REQUIRES_VEL",
        "MovePVA Cartesian waypoint " + std::to_string(k + 1) +
        " specifies acceleration without velocity (Cartesian acceleration requires velocity)");
      return false;
    }
    if (k + 1 < M && wp.velocity.has_value()) {
      const Vec6 V = to_vec6(*wp.velocity);
      if (V.head<3>().norm() > v_trans_lim * (1.0 + 1e-6)) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4)
            << "MovePVA waypoint " << (k + 1) << " translational speed="
            << V.head<3>().norm() << " m/s exceeds scaled limit=" << v_trans_lim << " m/s";
        add_diag(out_diagnostics, D::STAGE_KINEMATIC_LIMIT, wp.loc, "ERR_PVA_CART_VEL_LIMIT", oss.str());
        return false;
      }
      if (V.tail<3>().norm() > v_rot_lim * (1.0 + 1e-6)) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4)
            << "MovePVA waypoint " << (k + 1) << " rotational speed="
            << V.tail<3>().norm() << " rad/s exceeds scaled limit=" << v_rot_lim << " rad/s";
        add_diag(out_diagnostics, D::STAGE_KINEMATIC_LIMIT, wp.loc, "ERR_PVA_CART_VEL_LIMIT", oss.str());
        return false;
      }
      if (wp.acceleration.has_value()) {
        const Vec6 A = to_vec6(*wp.acceleration);
        if (A.head<3>().norm() > a_trans_lim * (1.0 + 1e-6)) {
          std::ostringstream oss;
          oss << std::fixed << std::setprecision(4)
              << "MovePVA waypoint " << (k + 1) << " translational acceleration="
              << A.head<3>().norm() << " m/s^2 exceeds scaled limit=" << a_trans_lim << " m/s^2";
          add_diag(out_diagnostics, D::STAGE_KINEMATIC_LIMIT, wp.loc, "ERR_PVA_CART_ACC_LIMIT", oss.str());
          return false;
        }
        if (A.tail<3>().norm() > a_rot_lim * (1.0 + 1e-6)) {
          std::ostringstream oss;
          oss << std::fixed << std::setprecision(4)
              << "MovePVA waypoint " << (k + 1) << " rotational acceleration="
              << A.tail<3>().norm() << " rad/s^2 exceeds scaled limit=" << a_rot_lim << " rad/s^2";
          add_diag(out_diagnostics, D::STAGE_KINEMATIC_LIMIT, wp.loc, "ERR_PVA_CART_ACC_LIMIT", oss.str());
          return false;
        }
      }
    }
  }

  // Evaluate start node smooth sigma.
  CartesianCollocationSolver::TcpDerivatives start_td;
  CartesianCollocationSolver::evaluate_tcp_derivatives(
    model, workspace.data, tcp, start_q, Vec6::Zero(), Vec6::Zero(), false, start_td);

  // Stage-wise candidate lists (stage 0 is the fixed start_q).
  std::vector<std::vector<IKBranchCandidate>> layers(M + 1);
  layers[0].push_back({start_q, Vec6::Zero(), Vec6::Zero(),
    engine.check_singularity(to_array(start_q), workspace.data), start_td.sigma_smooth});

  static constexpr std::array<double, 6> kWeights{{1.5, 1.5, 1.2, 1.0, 1.0, 0.8}};

  for (size_t k = 0; k < M; ++k) {
    const auto & wp = waypoints[k];
    const bool pin_v = (k + 1 < M) && wp.velocity.has_value();
    const bool pin_a = pin_v && wp.acceleration.has_value();
    const Vec6 V = pin_v ? to_vec6(*wp.velocity) : Vec6::Zero();
    const Vec6 A = pin_a ? to_vec6(*wp.acceleration) : Vec6::Zero();

    // Use the best candidate from the previous layer as IK seed to retrieve all base branches.
    const std::array<double, 6> seed = to_array(layers[k].front().q);
    std::vector<std::array<double, 6>> raw_sols;
    lite6_kinematics::IKDiagnostics ik_diag;
    const auto status = engine.solve_optimal_ik(
      wp.pose, seed, raw_sols, /*continuous=*/false, workspace, &ik_diag);
    if (status != IKResult::SUCCESS || raw_sols.empty()) {
      add_diag(
        out_diagnostics, D::STAGE_PLANNING, wp.loc, ik_error_code(status),
        "MovePVA Cartesian waypoint " + std::to_string(k + 1) + " IK failed: " + ik_diag.summary());
      return false;
    }

    // Filter branches against singularity and pinned joint (v, a) limits, tracking diagnostics.
    std::string fail_code;
    std::string fail_msg;
    std::vector<IKBranchCandidate> base_branches;

    for (const auto & s_arr : raw_sols) {
      Vec6 q_ref = to_vec6(s_arr);
      Vec6 q_exact;
      if (newton_ik(model, workspace.data, tcp, wp.pose, q_ref, q_exact)) {
        q_ref = q_exact;
      }
      // Verify refined q still satisfies joint position limits.
      bool pos_ok = true;
      for (int j = 0; j < 6; ++j) {
        if (q_ref[j] < limits[static_cast<size_t>(j)].min_pos - lite6_kinematics::tolerance::kJointPosition ||
            q_ref[j] > limits[static_cast<size_t>(j)].max_pos + lite6_kinematics::tolerance::kJointPosition)
        {
          pos_ok = false;
          break;
        }
        q_ref[j] = std::clamp(
          q_ref[j],
          limits[static_cast<size_t>(j)].min_pos,
          limits[static_cast<size_t>(j)].max_pos);
      }
      if (!pos_ok) {
        continue;
      }

      double sigma_min = 0.0;
      Vec6 dq = Vec6::Zero();
      Vec6 ddq = Vec6::Zero();
      if (pin_v) {
        if (!twist_acc_to_joint(
              model, workspace.data, tcp, q_ref, V, pin_a ? &A : nullptr,
              sing_thresh, dq, pin_a ? &ddq : nullptr, &sigma_min))
        {
          if (fail_code.empty()) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(6)
                << "MovePVA Cartesian waypoint " << (k + 1)
                << " is near singularity (normalized Jacobian sigma_min=" << sigma_min
                << " < singularity_threshold=" << sing_thresh << ")";
            fail_code = "ERR_SINGULARITY";
            fail_msg = oss.str();
          }
          continue;
        }
      } else {
        sigma_min = engine.check_singularity(to_array(q_ref), workspace.data);
        if (sigma_min < sing_thresh) {
          if (fail_code.empty()) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(6)
                << "MovePVA Cartesian waypoint " << (k + 1)
                << " is near singularity (normalized Jacobian sigma_min=" << sigma_min
                << " < singularity_threshold=" << sing_thresh << ")";
            fail_code = "ERR_SINGULARITY";
            fail_msg = oss.str();
          }
          continue;
        }
      }

      bool lim_ok = true;
      if (pin_v) {
        for (int j = 0; j < 6; ++j) {
          const double v_lim = limits[static_cast<size_t>(j)].max_vel * vs;
          if (std::abs(dq[j]) > v_lim * (1.0 + 1e-6)) {
            if (fail_code.empty()) {
              std::ostringstream oss;
              oss << std::fixed << std::setprecision(5)
                  << "MovePVA Cartesian waypoint " << (k + 1) << " twist requires joint" << (j + 1)
                  << " velocity=" << std::abs(dq[j]) << " rad/s > scaled limit=" << v_lim << " rad/s";
              fail_code = "ERR_PVA_VEL_LIMIT";
              fail_msg = oss.str();
            }
            lim_ok = false;
            break;
          }
          if (pin_a) {
            const double a_lim = limits[static_cast<size_t>(j)].max_acc * as;
            if (std::abs(ddq[j]) > a_lim * (1.0 + 1e-6)) {
              if (fail_code.empty()) {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(5)
                    << "MovePVA Cartesian waypoint " << (k + 1) << " kinematics require joint" << (j + 1)
                    << " accel=" << std::abs(ddq[j]) << " rad/s^2 > scaled limit=" << a_lim << " rad/s^2";
                fail_code = "ERR_PVA_ACC_LIMIT";
                fail_msg = oss.str();
              }
              lim_ok = false;
              break;
            }
          }
        }
      }
      if (!lim_ok) {
        continue;
      }

      CartesianCollocationSolver::TcpDerivatives td;
      CartesianCollocationSolver::evaluate_tcp_derivatives(
        model, workspace.data, tcp, q_ref, Vec6::Zero(), Vec6::Zero(), false, td);
      base_branches.push_back({q_ref, dq, ddq, sigma_min, td.sigma_smooth});
    }

    if (base_branches.empty()) {
      add_diag(
        out_diagnostics, D::STAGE_KINEMATIC_LIMIT, wp.loc,
        fail_code.empty() ? "ERR_NO_IK_SOLUTION" : fail_code,
        fail_msg.empty() ? ("No feasible IK branch at waypoint " + std::to_string(k + 1)) : fail_msg);
      return false;
    }

    // Expand valid 2pi wraps closest to each candidate in the previous layer so multi-turn
    // joints (e.g. joint1/joint4/joint6 in [-2pi, +2pi]) never suffer artificial wrap jumps.
    auto & cur_layer = layers[k + 1];
    auto add_unique = [&](const IKBranchCandidate & cand) {
      for (const auto & ex : cur_layer) {
        if ((ex.q - cand.q).cwiseAbs().maxCoeff() < 1e-4) {
          return;
        }
      }
      cur_layer.push_back(cand);
    };

    for (const auto & base : base_branches) {
      add_unique(base);
      for (const auto & prev : layers[k]) {
        IKBranchCandidate wrapped = base;
        for (int j = 0; j < 6; ++j) {
          const auto & lim = limits[static_cast<size_t>(j)];
          const double base_ang = base.q[j];
          const int k_lo = static_cast<int>(std::ceil((lim.min_pos - base_ang - 1e-10) / kTwoPi));
          const int k_hi = static_cast<int>(std::floor((lim.max_pos - base_ang + 1e-10) / kTwoPi));
          if (k_lo <= k_hi) {
            const int best_m = std::clamp(
              static_cast<int>(std::round((prev.q[j] - base_ang) / kTwoPi)), k_lo, k_hi);
            wrapped.q[j] = std::clamp(base_ang + best_m * kTwoPi, lim.min_pos, lim.max_pos);
          }
        }
        add_unique(wrapped);
      }
    }
  }

  // Dynamic programming over layers 0..M keeping up to K_beam paths per node to extract top chains.
  struct PathNode
  {
    double cost = 0.0;
    size_t prev_cand = 0;
    size_t prev_rank = 0;
  };
  const size_t K_beam = std::max<size_t>(max_chains, 3);
  std::vector<std::vector<std::vector<PathNode>>> dp(M + 1);
  dp[0].resize(1);
  dp[0][0].push_back({0.0, 0, 0});

  for (size_t k = 0; k < M; ++k) {
    const size_t n_prev = layers[k].size();
    const size_t n_cur = layers[k + 1].size();
    dp[k + 1].resize(n_cur);

    for (size_t c = 0; c < n_cur; ++c) {
      const auto & to = layers[k + 1][c];
      std::vector<PathNode> cands;
      for (size_t p = 0; p < n_prev; ++p) {
        if (dp[k][p].empty()) {
          continue;
        }
        const auto & from = layers[k][p];
        double edge_cost = 0.0;
        for (int j = 0; j < 6; ++j) {
          const double dq_j = to.q[j] - from.q[j];
          edge_cost += kWeights[static_cast<size_t>(j)] * dq_j * dq_j;
        }
        // Penalize crossing wrist singularity (q5 sign flip) or singular/colliding midpoints.
        const Vec6 q_mid = 0.5 * (from.q + to.q);
        if (from.q[4] * to.q[4] < -1e-4) {
          edge_cost += 12.0;
        }
        const double mid_sig = engine.check_singularity(to_array(q_mid), workspace.data);
        if (mid_sig < sing_thresh * 2.0) {
          edge_cost += 25.0;
        }
        if (engine.check_collision(to_array(q_mid), workspace)) {
          edge_cost += 15.0;
        }
        for (size_t r = 0; r < dp[k][p].size(); ++r) {
          cands.push_back({dp[k][p][r].cost + edge_cost, p, r});
        }
      }
      std::sort(cands.begin(), cands.end(), [](const PathNode & a, const PathNode & b) {
        return a.cost < b.cost;
      });
      if (cands.size() > K_beam) {
        cands.resize(K_beam);
      }
      dp[k + 1][c] = std::move(cands);
    }
  }

  // Collect best end paths across final layer.
  struct FinalEntry
  {
    double cost;
    size_t cand;
    size_t rank;
  };
  std::vector<FinalEntry> finals;
  for (size_t c = 0; c < layers[M].size(); ++c) {
    for (size_t r = 0; r < dp[M][c].size(); ++r) {
      finals.push_back({dp[M][c][r].cost, c, r});
    }
  }
  std::sort(finals.begin(), finals.end(), [](const FinalEntry & a, const FinalEntry & b) {
    return a.cost < b.cost;
  });

  out_chains.clear();
  for (const auto & fin : finals) {
    if (out_chains.size() >= max_chains) {
      break;
    }
    ResolvedWaypointChain chain;
    chain.q.resize(M + 1);
    chain.dq_pin.assign(M + 1, Vec6::Zero());
    chain.ddq_pin.assign(M + 1, Vec6::Zero());
    chain.has_v.assign(M + 1, false);
    chain.has_a.assign(M + 1, false);
    chain.has_v[0] = chain.has_a[0] = true;
    chain.has_v[M] = chain.has_a[M] = true;
    chain.cost = fin.cost;
    chain.min_sigma_smooth = std::numeric_limits<double>::infinity();

    size_t cur_c = fin.cand;
    size_t cur_r = fin.rank;
    for (size_t k = M + 1; k-- > 0;) {
      const auto & item = layers[k][cur_c];
      chain.q[k] = item.q;
      chain.min_sigma_smooth = std::min(chain.min_sigma_smooth, item.sigma_smooth);
      if (k > 0 && k < M) {
        const auto & wp = waypoints[k - 1];
        if (wp.velocity.has_value()) {
          chain.has_v[k] = true;
          chain.dq_pin[k] = item.dq;
          if (wp.acceleration.has_value()) {
            chain.has_a[k] = true;
            chain.ddq_pin[k] = item.ddq;
          }
        }
      }
      if (k > 0) {
        const auto & pn = dp[k][cur_c][cur_r];
        cur_c = pn.prev_cand;
        cur_r = pn.prev_rank;
      }
    }
    out_chains.push_back(std::move(chain));
  }

  return !out_chains.empty();
}

// ============================================================================
// Direct-Transcription Collocation NLP Mesh & Primal-Dual IPM Solver
// ============================================================================

struct CollocationMesh
{
  size_t M = 0;                          // Number of segments
  size_t N = 0;                          // Total collocation intervals
  std::vector<size_t> wp_node;           // size M + 1: wp_node[0]=0, wp_node[M]=N
  std::vector<size_t> interval_seg;      // size N: segment index s in [0, M-1]
  std::vector<double> gamma;             // size N: h_i = gamma[i] * T[s], sum_{i in s} gamma[i] = 1
  std::vector<bool> seg_dt_fixed;        // size M
  std::vector<double> seg_dt_fixed_val;  // size M
  std::vector<int> seg_free_idx;         // size M: index in [0, M_free-1] or -1 if fixed
  std::vector<size_t> free_seg_of;       // size M_free
  std::vector<double> T_min;             // size M
  std::vector<double> T_max;             // size M
  std::vector<double> T_scale;           // size M (fixed nominal scale per segment)
  std::vector<double> h_nom;             // size N (fixed nominal interval width for scaling e_q)

  // Per-node primal vectors in physical units
  std::vector<Vec6> q;                   // size N + 1
  std::vector<Vec6> v;                   // size N + 1
  std::vector<Vec6> a;                   // size N + 1
  std::vector<double> T;                 // size M

  // Fixed flags for nodal primal variables (size (N+1)*18)
  std::vector<bool> node_fixed;
  // Physical box bounds for free nodal primal variables
  std::vector<double> node_lb;
  std::vector<double> node_ub;
  // Per-component primal scales (size 18: 1, V_s, A_s)
  std::array<double, 18> comp_scale{};

  // Scaled joint & Cartesian limits
  std::array<double, 6> q_min{};
  std::array<double, 6> q_max{};
  std::array<double, 6> v_lim{};
  std::array<double, 6> a_lim{};
  std::array<double, 6> j_lim{};
  double vt_lim = 1.0;
  double vr_lim = 2.5;
  double at_lim = 3.0;
  double ar_lim = 5.0;
  double sigma_safe = 0.0036;
  double sing_thresh = 0.003;
  double cart_safety = 0.996;
  double w_jerk = 2e-3;
  double w_acc = 1e-4;
  bool any_free_T = true;

  // Inequality constraint map
  struct IneqDesc
  {
    enum Kind : uint8_t { JERK_POS, JERK_NEG, CART_VT, CART_VR, CART_AT, CART_AR, SINGULARITY };
    Kind kind;
    size_t idx;    // interval index (for JERK_*) or node index (for CART_*, SINGULARITY)
    int joint;     // 0..5 for JERK_*, -1 otherwise
  };
  std::vector<IneqDesc> ineqs;
  std::vector<bool> node_needs_kin;  // size N + 1

  size_t band_size() const { return N * kBlockStride + kNodePrimal; }
  size_t border_size() const { return free_seg_of.size(); }

  static inline size_t z_q(size_t i, int j) { return i * kBlockStride + static_cast<size_t>(j); }
  static inline size_t z_v(size_t i, int j) { return i * kBlockStride + 6 + static_cast<size_t>(j); }
  static inline size_t z_a(size_t i, int j) { return i * kBlockStride + 12 + static_cast<size_t>(j); }
  static inline size_t z_lq(size_t i, int j) { return i * kBlockStride + 18 + static_cast<size_t>(j); }
  static inline size_t z_lv(size_t i, int j) { return i * kBlockStride + 24 + static_cast<size_t>(j); }

  void rebuild_metadata(const ResolvedWaypointChain & chain)
  {
    N = gamma.size();
    interval_seg.resize(N);
    for (size_t s = 0; s < M; ++s) {
      for (size_t i = wp_node[s]; i < wp_node[s + 1]; ++i) {
        interval_seg[i] = s;
      }
    }
    h_nom.resize(N);
    for (size_t i = 0; i < N; ++i) {
      h_nom[i] = std::max(1e-3, gamma[i] * T_scale[interval_seg[i]]);
    }

    seg_free_idx.assign(M, -1);
    free_seg_of.clear();
    for (size_t s = 0; s < M; ++s) {
      if (!seg_dt_fixed[s]) {
        seg_free_idx[s] = static_cast<int>(free_seg_of.size());
        free_seg_of.push_back(s);
      }
    }
    any_free_T = !free_seg_of.empty();

    // Node fixed flags and box bounds.
    node_fixed.assign((N + 1) * kNodePrimal, false);
    node_lb.assign((N + 1) * kNodePrimal, 0.0);
    node_ub.assign((N + 1) * kNodePrimal, 0.0);
    for (size_t i = 0; i <= N; ++i) {
      for (int j = 0; j < 6; ++j) {
        const size_t base = i * kNodePrimal;
        node_lb[base + j] = q_min[static_cast<size_t>(j)];
        node_ub[base + j] = q_max[static_cast<size_t>(j)];
        node_lb[base + 6 + j] = -v_lim[static_cast<size_t>(j)];
        node_ub[base + 6 + j] = v_lim[static_cast<size_t>(j)];
        node_lb[base + 12 + j] = -a_lim[static_cast<size_t>(j)];
        node_ub[base + 12 + j] = a_lim[static_cast<size_t>(j)];
      }
    }
    for (size_t k = 0; k <= M; ++k) {
      const size_t ni = wp_node[k];
      const size_t base = ni * kNodePrimal;
      for (int j = 0; j < 6; ++j) {
        node_fixed[base + j] = true;
        if (chain.has_v[k]) {
          node_fixed[base + 6 + j] = true;
        }
        if (chain.has_a[k]) {
          node_fixed[base + 12 + j] = true;
        }
      }
    }

    // Build general inequality list (jerk on all intervals, Cartesian + singularity on active nodes).
    ineqs.clear();
    for (size_t i = 0; i < N; ++i) {
      for (int j = 0; j < 6; ++j) {
        ineqs.push_back({IneqDesc::JERK_POS, i, j});
        ineqs.push_back({IneqDesc::JERK_NEG, i, j});
      }
    }
    node_needs_kin.assign(N + 1, false);
    for (size_t i = 0; i <= N; ++i) {
      const size_t base = i * kNodePrimal;
      const bool q_free = !node_fixed[base];
      const bool v_free = !node_fixed[base + 6];
      const bool a_free = !node_fixed[base + 12];
      if (!q_free && !v_free && !a_free) {
        continue;
      }
      node_needs_kin[i] = true;
      if (q_free || v_free) {
        ineqs.push_back({IneqDesc::CART_VT, i, -1});
        ineqs.push_back({IneqDesc::CART_VR, i, -1});
      }
      ineqs.push_back({IneqDesc::CART_AT, i, -1});
      ineqs.push_back({IneqDesc::CART_AR, i, -1});
      if (q_free) {
        ineqs.push_back({IneqDesc::SINGULARITY, i, -1});
      }
    }
  }
};

struct IpmSolveStats
{
  bool converged = false;
  bool kkt_optimal = false;
  bool primal_feasible = false;
  bool cancelled = false;
  int iterations = 0;
  double eq_inf = 0.0;
  double ineq_inf = 0.0;
  double dual_inf = 0.0;
  double comp_inf = 0.0;
  double mu = 0.0;
};

IpmSolveStats run_collocation_ipm(
  const pinocchio::Model & model,
  pinocchio::FrameIndex tcp_frame_id,
  CollocationMesh & mesh,
  int max_iters,
  double init_mu,
  const std::function<bool()> & is_cancelled = {})
{
  IpmSolveStats stats;
  const size_t N = mesh.N;
  const size_t n_band = mesh.band_size();
  const size_t n_border = mesh.border_size();
  const size_t n_ineq = mesh.ineqs.size();

  // Build band-level is_primal / is_fixed arrays.
  std::vector<bool> is_primal(n_band, false);
  std::vector<bool> is_fixed(n_band, false);
  for (size_t i = 0; i <= N; ++i) {
    const size_t z_off = i * kBlockStride;
    const size_t n_off = i * kNodePrimal;
    for (int c = 0; c < kNodePrimal; ++c) {
      is_primal[z_off + static_cast<size_t>(c)] = true;
      is_fixed[z_off + static_cast<size_t>(c)] = mesh.node_fixed[n_off + static_cast<size_t>(c)];
    }
  }

  // Normalized primal vector x_node (size (N+1)*18) and x_T (size n_border).
  auto pack_primal = [&](std::vector<double> & xn, std::vector<double> & xt) {
    xn.resize((N + 1) * kNodePrimal);
    for (size_t i = 0; i <= N; ++i) {
      const size_t base = i * kNodePrimal;
      for (int j = 0; j < 6; ++j) {
        xn[base + j] = mesh.q[i][j] / mesh.comp_scale[static_cast<size_t>(j)];
        xn[base + 6 + j] = mesh.v[i][j] / mesh.comp_scale[static_cast<size_t>(6 + j)];
        xn[base + 12 + j] = mesh.a[i][j] / mesh.comp_scale[static_cast<size_t>(12 + j)];
      }
    }
    xt.resize(n_border);
    for (size_t c = 0; c < n_border; ++c) {
      const size_t s = mesh.free_seg_of[c];
      xt[c] = mesh.T[s] / mesh.T_scale[s];
    }
  };

  auto unpack_primal = [&](const std::vector<double> & xn, const std::vector<double> & xt,
                           std::vector<Vec6> & q_out, std::vector<Vec6> & v_out,
                           std::vector<Vec6> & a_out, std::vector<double> & T_out)
  {
    q_out.resize(N + 1);
    v_out.resize(N + 1);
    a_out.resize(N + 1);
    for (size_t i = 0; i <= N; ++i) {
      const size_t base = i * kNodePrimal;
      for (int j = 0; j < 6; ++j) {
        q_out[i][j] = xn[base + j] * mesh.comp_scale[static_cast<size_t>(j)];
        v_out[i][j] = xn[base + 6 + j] * mesh.comp_scale[static_cast<size_t>(6 + j)];
        a_out[i][j] = xn[base + 12 + j] * mesh.comp_scale[static_cast<size_t>(12 + j)];
      }
    }
    T_out = mesh.T;
    for (size_t c = 0; c < n_border; ++c) {
      const size_t s = mesh.free_seg_of[c];
      T_out[s] = xt[c] * mesh.T_scale[s];
    }
  };

  // Normalized box bounds for free nodal variables & free segment durations.
  std::vector<double> lb_n((N + 1) * kNodePrimal, 0.0);
  std::vector<double> ub_n((N + 1) * kNodePrimal, 0.0);
  for (size_t i = 0; i <= N; ++i) {
    const size_t base = i * kNodePrimal;
    for (int c = 0; c < kNodePrimal; ++c) {
      const double sc = mesh.comp_scale[static_cast<size_t>(c)];
      lb_n[base + static_cast<size_t>(c)] = mesh.node_lb[base + static_cast<size_t>(c)] / sc;
      ub_n[base + static_cast<size_t>(c)] = mesh.node_ub[base + static_cast<size_t>(c)] / sc;
    }
  }
  std::vector<double> lb_t(n_border, 0.0);
  std::vector<double> ub_t(n_border, 0.0);
  for (size_t c = 0; c < n_border; ++c) {
    const size_t s = mesh.free_seg_of[c];
    lb_t[c] = mesh.T_min[s] / mesh.T_scale[s];
    ub_t[c] = mesh.T_max[s] / mesh.T_scale[s];
  }

  std::vector<double> xn, xt;
  pack_primal(xn, xt);

  // Push free initial primal variables strictly inside box bounds.
  for (size_t idx = 0; idx < xn.size(); ++idx) {
    if (mesh.node_fixed[idx]) {
      continue;
    }
    const double span = ub_n[idx] - lb_n[idx];
    const double margin = std::min(5e-3, 0.02 * span);
    xn[idx] = std::clamp(xn[idx], lb_n[idx] + margin, ub_n[idx] - margin);
  }
  for (size_t c = 0; c < n_border; ++c) {
    const double span = ub_t[c] - lb_t[c];
    const double margin = std::min(5e-3, 0.02 * span);
    xt[c] = std::clamp(xt[c], lb_t[c] + margin, ub_t[c] - margin);
  }

  // Dual and slack variables.
  double mu = init_mu;
  std::vector<double> y_eq(N * kIntervalDual, 0.0);  // [eq_q(6), eq_v(6)] per interval
  std::vector<double> zL_n(xn.size(), 0.0), zU_n(xn.size(), 0.0);
  for (size_t idx = 0; idx < xn.size(); ++idx) {
    if (!mesh.node_fixed[idx]) {
      zL_n[idx] = std::clamp(mu / std::max(1e-3, xn[idx] - lb_n[idx]), 1e-4, 1e2);
      zU_n[idx] = std::clamp(mu / std::max(1e-3, ub_n[idx] - xn[idx]), 1e-4, 1e2);
    }
  }
  std::vector<double> zL_t(n_border, 0.0), zU_t(n_border, 0.0);
  for (size_t c = 0; c < n_border; ++c) {
    zL_t[c] = std::clamp(mu / std::max(1e-3, xt[c] - lb_t[c]), 1e-4, 1e2);
    zU_t[c] = std::clamp(mu / std::max(1e-3, ub_t[c] - xt[c]), 1e-4, 1e2);
  }

  pinocchio::Data pin_data(model);
  std::vector<Vec6> q_cur, v_cur, a_cur;
  std::vector<double> T_cur;
  std::vector<CartesianCollocationSolver::TcpDerivatives> td_cur(N + 1);
  std::vector<double> eq_cur(N * kIntervalDual, 0.0);
  std::vector<double> g_cur(n_ineq, 0.0);

  const double w_jerk_eff = mesh.any_free_T ? mesh.w_jerk : 2.0;
  const double w_acc_eff = mesh.any_free_T ? mesh.w_acc : 0.05;

  const double vt_eff = mesh.vt_lim * mesh.cart_safety;
  const double vr_eff = mesh.vr_lim * mesh.cart_safety;
  const double at_eff = mesh.at_lim * mesh.cart_safety;
  const double ar_eff = mesh.ar_lim * mesh.cart_safety;
  const double inv_vt2 = 1.0 / (vt_eff * vt_eff);
  const double inv_vr2 = 1.0 / (vr_eff * vr_eff);
  const double inv_at2 = 1.0 / (at_eff * at_eff);
  const double inv_ar2 = 1.0 / (ar_eff * ar_eff);

  auto evaluate_state = [&](
    const std::vector<double> & pxn,
    const std::vector<double> & pxt,
    bool need_derivs,
    std::vector<Vec6> & q_v,
    std::vector<Vec6> & v_v,
    std::vector<Vec6> & a_v,
    std::vector<double> & T_v,
    std::vector<CartesianCollocationSolver::TcpDerivatives> & td_v,
    double & out_f,
    std::vector<double> & out_eq,
    std::vector<double> & out_g)
  {
    unpack_primal(pxn, pxt, q_v, v_v, a_v, T_v);
    out_f = 0.0;
    for (size_t c = 0; c < n_border; ++c) {
      out_f += T_v[mesh.free_seg_of[c]];
    }

    out_eq.resize(N * kIntervalDual);
    for (size_t i = 0; i < N; ++i) {
      const size_t s = mesh.interval_seg[i];
      const double h = mesh.gamma[i] * T_v[s];
      const double inv_h = 1.0 / h;
      for (int j = 0; j < 6; ++j) {
        const size_t ju = static_cast<size_t>(j);
        const double da = a_v[i + 1][j] - a_v[i][j];
        const double inv_jlim = 1.0 / mesh.j_lim[ju];
        out_f += w_jerk_eff * (da * da) * inv_h * (inv_jlim * inv_jlim);

        const double eq_q = q_v[i + 1][j] - q_v[i][j] - h * v_v[i][j] -
          (h * h / 6.0) * (2.0 * a_v[i][j] + a_v[i + 1][j]);
        const double eq_v = v_v[i + 1][j] - v_v[i][j] -
          0.5 * h * (a_v[i][j] + a_v[i + 1][j]);
        const double sc_q = mesh.comp_scale[6 + ju] * mesh.h_nom[i];
        const double sc_v = mesh.comp_scale[6 + ju];
        out_eq[i * kIntervalDual + static_cast<size_t>(j)] = eq_q / sc_q;
        out_eq[i * kIntervalDual + 6 + static_cast<size_t>(j)] = eq_v / sc_v;
      }
    }
    for (size_t i = 0; i <= N; ++i) {
      const double hw = (i < N) ? mesh.h_nom[i] : mesh.h_nom[N - 1];
      for (int j = 0; j < 6; ++j) {
        const double an = a_v[i][j] / mesh.a_lim[static_cast<size_t>(j)];
        out_f += w_acc_eff * hw * an * an;
      }
      if (mesh.node_needs_kin[i]) {
        CartesianCollocationSolver::evaluate_tcp_derivatives(
          model, pin_data, tcp_frame_id, q_v[i], v_v[i], a_v[i], need_derivs, td_v[i]);
      }
    }

    out_g.resize(n_ineq);
    for (size_t m = 0; m < n_ineq; ++m) {
      const auto & desc = mesh.ineqs[m];
      switch (desc.kind) {
        case CollocationMesh::IneqDesc::JERK_POS: {
          const size_t i = desc.idx;
          const int j = desc.joint;
          const double h = mesh.gamma[i] * T_v[mesh.interval_seg[i]];
          const double sc = mesh.comp_scale[static_cast<size_t>(12 + j)];
          out_g[m] = (a_v[i + 1][j] - a_v[i][j] - mesh.j_lim[static_cast<size_t>(j)] * h) / sc;
          break;
        }
        case CollocationMesh::IneqDesc::JERK_NEG: {
          const size_t i = desc.idx;
          const int j = desc.joint;
          const double h = mesh.gamma[i] * T_v[mesh.interval_seg[i]];
          const double sc = mesh.comp_scale[static_cast<size_t>(12 + j)];
          out_g[m] = (a_v[i][j] - a_v[i + 1][j] - mesh.j_lim[static_cast<size_t>(j)] * h) / sc;
          break;
        }
        case CollocationMesh::IneqDesc::CART_VT:
          out_g[m] = td_v[desc.idx].v_lin.squaredNorm() * inv_vt2 - 1.0;
          break;
        case CollocationMesh::IneqDesc::CART_VR:
          out_g[m] = td_v[desc.idx].v_ang.squaredNorm() * inv_vr2 - 1.0;
          break;
        case CollocationMesh::IneqDesc::CART_AT:
          out_g[m] = td_v[desc.idx].a_lin.squaredNorm() * inv_at2 - 1.0;
          break;
        case CollocationMesh::IneqDesc::CART_AR:
          out_g[m] = td_v[desc.idx].a_ang.squaredNorm() * inv_ar2 - 1.0;
          break;
        case CollocationMesh::IneqDesc::SINGULARITY:
          out_g[m] = 1.0 - td_v[desc.idx].sigma_smooth / mesh.sigma_safe;
          break;
      }
    }
  };

  double f_cur = 0.0;
  evaluate_state(xn, xt, true, q_cur, v_cur, a_cur, T_cur, td_cur, f_cur, eq_cur, g_cur);

  // Initialize inequality slacks strictly positive and duals consistent with mu.
  std::vector<double> s_ineq(n_ineq, 0.1);
  std::vector<double> lam(n_ineq, 0.1);
  for (size_t m = 0; m < n_ineq; ++m) {
    s_ineq[m] = std::max(0.08, -g_cur[m] + 0.02);
    lam[m] = std::clamp(mu / s_ineq[m], 1e-4, 1e2);
  }

  BorderedBandedKKT kkt;
  kkt.init(n_band, n_border, is_primal);

  auto compute_filter_pair = [&](
    const std::vector<double> & pxn,
    const std::vector<double> & pxt,
    const std::vector<double> & ps,
    double pf,
    const std::vector<double> & peq,
    const std::vector<double> & pg,
    double pmu,
    double & out_theta,
    double & out_phi)
  {
    double th = 0.0;
    for (double v : peq) {
      th += std::abs(v);
    }
    for (size_t m = 0; m < n_ineq; ++m) {
      th += std::abs(pg[m] + ps[m]);
    }
    double phi = pf;
    for (size_t idx = 0; idx < pxn.size(); ++idx) {
      if (!mesh.node_fixed[idx]) {
        phi -= pmu * std::log(std::max(1e-30, pxn[idx] - lb_n[idx]));
        phi -= pmu * std::log(std::max(1e-30, ub_n[idx] - pxn[idx]));
      }
    }
    for (size_t c = 0; c < n_border; ++c) {
      phi -= pmu * std::log(std::max(1e-30, pxt[c] - lb_t[c]));
      phi -= pmu * std::log(std::max(1e-30, ub_t[c] - pxt[c]));
    }
    for (size_t m = 0; m < n_ineq; ++m) {
      phi -= pmu * std::log(std::max(1e-30, ps[m]));
    }
    out_theta = th;
    out_phi = phi;
  };

  std::vector<std::pair<double, double>> filter;
  constexpr double kGammaTheta = 1e-5;
  constexpr double kGammaPhi = 1e-5;
  auto filter_accepts = [&](double th, double ph) -> bool {
    for (const auto & entry : filter) {
      if (th >= (1.0 - kGammaTheta) * entry.first && ph >= entry.second - kGammaPhi * entry.first) {
        return false;
      }
    }
    return true;
  };

  // Scratch buffers reused every IPM iteration.
  std::vector<double> rhs_z(n_band, 0.0);
  std::vector<double> rhs_T(n_border, 0.0);
  std::vector<double> grad_f_n(xn.size(), 0.0);
  std::vector<double> grad_f_t(n_border, 0.0);
  std::vector<double> stat_n(xn.size(), 0.0);
  std::vector<double> stat_t(n_border, 0.0);
  std::vector<double> dz(n_band, 0.0);
  std::vector<double> dT(n_border, 0.0);
  std::vector<double> ds(n_ineq, 0.0);
  std::vector<double> dlam(n_ineq, 0.0);
  std::vector<double> dzL_n(xn.size(), 0.0), dzU_n(xn.size(), 0.0);
  std::vector<double> dzL_t(n_border, 0.0), dzU_t(n_border, 0.0);

  // Per-inequality sparse gradient cache (at most 18 nodal entries + optional T entry).
  struct IneqGradCache
  {
    int n_node = 0;
    std::array<size_t, 18> z_idx{};
    std::array<double, 18> z_val{};
    int t_col = -1;
    double t_val = 0.0;
  };
  std::vector<IneqGradCache> g_grads(n_ineq);

  std::vector<double> xn_tr(xn.size()), xt_tr(n_border), s_tr(n_ineq);
  std::vector<Vec6> q_tr, v_tr, a_tr;
  std::vector<double> T_tr, eq_tr, g_tr;
  std::vector<CartesianCollocationSolver::TcpDerivatives> td_tr(N + 1);

  double last_delta_w = 0.0;
  int stall_count = 0;

  for (int iter = 0; iter < max_iters; ++iter) {
    if (is_cancelled && is_cancelled()) {
      stats.cancelled = true;
      return stats;
    }
    stats.iterations = iter + 1;

    // 1. Evaluate residuals, gradients, and assemble bordered banded KKT system.
    kkt.clear();
    std::fill(rhs_z.begin(), rhs_z.end(), 0.0);
    std::fill(rhs_T.begin(), rhs_T.end(), 0.0);
    std::fill(grad_f_n.begin(), grad_f_n.end(), 0.0);
    std::fill(grad_f_t.begin(), grad_f_t.end(), 0.0);

    // Objective gradient and exact convex Hessian.
    for (size_t c = 0; c < n_border; ++c) {
      const size_t s = mesh.free_seg_of[c];
      grad_f_t[c] += mesh.T_scale[s];
    }
    for (size_t i = 0; i < N; ++i) {
      const size_t s = mesh.interval_seg[i];
      const double gam = mesh.gamma[i];
      const double h = gam * T_cur[s];
      const double inv_h = 1.0 / h;
      const double inv_h2 = inv_h * inv_h;
      const double inv_h3 = inv_h2 * inv_h;
      const int t_col = mesh.seg_free_idx[s];
      const double dh_dxt = (t_col >= 0) ? (gam * mesh.T_scale[s]) : 0.0;

      for (int j = 0; j < 6; ++j) {
        const size_t ju = static_cast<size_t>(j);
        const double sc_a = mesh.comp_scale[12 + ju];
        const double c_j = w_jerk_eff / (mesh.j_lim[ju] * mesh.j_lim[ju]);
        const double da = a_cur[i + 1][j] - a_cur[i][j];

        const double g_ai = -2.0 * c_j * da * inv_h * sc_a;
        const double g_ai1 = +2.0 * c_j * da * inv_h * sc_a;
        grad_f_n[i * kNodePrimal + 12 + ju] += g_ai;
        grad_f_n[(i + 1) * kNodePrimal + 12 + ju] += g_ai1;

        const double h_aa = 2.0 * c_j * inv_h * sc_a * sc_a;
        const size_t za0 = CollocationMesh::z_a(i, j);
        const size_t za1 = CollocationMesh::z_a(i + 1, j);
        if (!is_fixed[za0]) {
          kkt.add_band(za0, za0, h_aa);
        }
        if (!is_fixed[za1]) {
          kkt.add_band(za1, za1, h_aa);
        }
        if (!is_fixed[za0] && !is_fixed[za1]) {
          kkt.add_band(za0, za1, -h_aa);
        }

        if (t_col >= 0) {
          const size_t tc = static_cast<size_t>(t_col);
          grad_f_t[tc] += -c_j * da * da * inv_h2 * dh_dxt;
          kkt.add_T_diag(tc, 2.0 * c_j * da * da * inv_h3 * dh_dxt * dh_dxt);
          const double h_at = 2.0 * c_j * da * inv_h2 * sc_a * dh_dxt;
          if (!is_fixed[za0]) {
            kkt.add_border(za0, tc, +h_at);
          }
          if (!is_fixed[za1]) {
            kkt.add_border(za1, tc, -h_at);
          }
        }
      }
    }
    for (size_t i = 0; i <= N; ++i) {
      const double hw = (i < N) ? mesh.h_nom[i] : mesh.h_nom[N - 1];
      for (int j = 0; j < 6; ++j) {
        const size_t ju = static_cast<size_t>(j);
        const double sc_a = mesh.comp_scale[12 + ju];
        const double inv_al2 = 1.0 / (mesh.a_lim[ju] * mesh.a_lim[ju]);
        grad_f_n[i * kNodePrimal + 12 + ju] += 2.0 * w_acc_eff * hw * a_cur[i][j] * inv_al2 * sc_a;
        const size_t za = CollocationMesh::z_a(i, j);
        if (!is_fixed[za]) {
          kkt.add_band(za, za, 2.0 * w_acc_eff * hw * inv_al2 * sc_a * sc_a);
        }
      }
    }

    stat_n = grad_f_n;
    stat_t = grad_f_t;

    // Equality constraints (exact cubic dynamics): Jacobian + exact Lagrangian Hessian in B & D_T.
    double eq_inf = 0.0;
    for (size_t i = 0; i < N; ++i) {
      const size_t s = mesh.interval_seg[i];
      const double gam = mesh.gamma[i];
      const double h = gam * T_cur[s];
      const int t_col = mesh.seg_free_idx[s];
      const double dh_dxt = (t_col >= 0) ? (gam * mesh.T_scale[s]) : 0.0;

      for (int j = 0; j < 6; ++j) {
        const size_t ju = static_cast<size_t>(j);
        const double sc_q_var = mesh.comp_scale[ju];
        const double sc_v_var = mesh.comp_scale[6 + ju];
        const double sc_a_var = mesh.comp_scale[12 + ju];
        const double inv_eq_q = 1.0 / (sc_v_var * mesh.h_nom[i]);
        const double inv_eq_v = 1.0 / sc_v_var;

        const double rq = eq_cur[i * kIntervalDual + ju];
        const double rv = eq_cur[i * kIntervalDual + 6 + ju];
        eq_inf = std::max({eq_inf, std::abs(rq), std::abs(rv)});

        const double yq = y_eq[i * kIntervalDual + ju];
        const double yv = y_eq[i * kIntervalDual + 6 + ju];

        const size_t zq0 = CollocationMesh::z_q(i, j);
        const size_t zv0 = CollocationMesh::z_v(i, j);
        const size_t za0 = CollocationMesh::z_a(i, j);
        const size_t zq1 = CollocationMesh::z_q(i + 1, j);
        const size_t zv1 = CollocationMesh::z_v(i + 1, j);
        const size_t za1 = CollocationMesh::z_a(i + 1, j);
        const size_t zlq = CollocationMesh::z_lq(i, j);
        const size_t zlv = CollocationMesh::z_lv(i, j);

        rhs_z[zlq] = -rq;
        rhs_z[zlv] = -rv;

        // J_eq_q entries (scaled by inv_eq_q and primal variable scale)
        const double jq_q0 = -sc_q_var * inv_eq_q;
        const double jq_q1 = +sc_q_var * inv_eq_q;
        const double jq_v0 = -h * sc_v_var * inv_eq_q;
        const double jq_a0 = -(h * h / 3.0) * sc_a_var * inv_eq_q;
        const double jq_a1 = -(h * h / 6.0) * sc_a_var * inv_eq_q;

        // J_eq_v entries (scaled by inv_eq_v and primal variable scale)
        const double jv_v0 = -sc_v_var * inv_eq_v;
        const double jv_v1 = +sc_v_var * inv_eq_v;
        const double jv_a0 = -0.5 * h * sc_a_var * inv_eq_v;
        const double jv_a1 = -0.5 * h * sc_a_var * inv_eq_v;

        auto add_jac = [&](size_t z_var, size_t n_var, size_t z_lam, double y_val, double j_val) {
          stat_n[n_var] += y_val * j_val;
          if (!is_fixed[z_var]) {
            kkt.add_band(z_var, z_lam, j_val);
          }
        };
        add_jac(zq0, i * kNodePrimal + ju, zlq, yq, jq_q0);
        add_jac(zq1, (i + 1) * kNodePrimal + ju, zlq, yq, jq_q1);
        add_jac(zv0, i * kNodePrimal + 6 + ju, zlq, yq, jq_v0);
        add_jac(za0, i * kNodePrimal + 12 + ju, zlq, yq, jq_a0);
        add_jac(za1, (i + 1) * kNodePrimal + 12 + ju, zlq, yq, jq_a1);

        add_jac(zv0, i * kNodePrimal + 6 + ju, zlv, yv, jv_v0);
        add_jac(zv1, (i + 1) * kNodePrimal + 6 + ju, zlv, yv, jv_v1);
        add_jac(za0, i * kNodePrimal + 12 + ju, zlv, yv, jv_a0);
        add_jac(za1, (i + 1) * kNodePrimal + 12 + ju, zlv, yv, jv_a1);

        if (t_col >= 0) {
          const size_t tc = static_cast<size_t>(t_col);
          const double jq_t = -(v_cur[i][j] + (h / 3.0) * (2.0 * a_cur[i][j] + a_cur[i + 1][j])) *
            dh_dxt * inv_eq_q;
          const double jv_t = -0.5 * (a_cur[i][j] + a_cur[i + 1][j]) * dh_dxt * inv_eq_v;
          stat_t[tc] += yq * jq_t + yv * jv_t;
          kkt.add_border(zlq, tc, jq_t);
          kkt.add_border(zlv, tc, jv_t);

          // Exact 2nd derivatives of dynamics equalities in Lagrangian: yq * d2(e_q) + yv * d2(e_v).
          const double h_v0_t = -yq * sc_v_var * dh_dxt * inv_eq_q;
          const double h_a0_t = (-yq * (2.0 * h / 3.0) * inv_eq_q - yv * 0.5 * inv_eq_v) *
            sc_a_var * dh_dxt;
          const double h_a1_t = (-yq * (h / 3.0) * inv_eq_q - yv * 0.5 * inv_eq_v) *
            sc_a_var * dh_dxt;
          const double h_tt = -yq * ((2.0 * a_cur[i][j] + a_cur[i + 1][j]) / 3.0) *
            dh_dxt * dh_dxt * inv_eq_q;
          if (!is_fixed[zv0]) {
            kkt.add_border(zv0, tc, h_v0_t);
          }
          if (!is_fixed[za0]) {
            kkt.add_border(za0, tc, h_a0_t);
          }
          if (!is_fixed[za1]) {
            kkt.add_border(za1, tc, h_a1_t);
          }
          kkt.add_T_diag(tc, h_tt);
        }
      }
    }

    // Inequality constraints: gradients, Gauss-Newton PSD Hessians, and condensed barrier terms.
    double ineq_inf = 0.0;
    double prim_slack_inf = 0.0;
    double comp_mu_inf = 0.0;
    double comp_zero_inf = 0.0;

    for (size_t m = 0; m < n_ineq; ++m) {
      const auto & desc = mesh.ineqs[m];
      auto & gc = g_grads[m];
      gc.n_node = 0;
      gc.t_col = -1;
      gc.t_val = 0.0;

      const double gm = g_cur[m];
      const double sm = s_ineq[m];
      const double lm = lam[m];
      ineq_inf = std::max(ineq_inf, std::max(0.0, gm));
      prim_slack_inf = std::max(prim_slack_inf, std::abs(gm + sm));
      comp_mu_inf = std::max(comp_mu_inf, std::abs(sm * lm - mu));
      comp_zero_inf = std::max(comp_zero_inf, std::abs(sm * lm));

      const double sig_m = lm / sm;
      const double rhs_coef = (mu + lm * (gm + sm)) / sm;

      auto push_node_grad = [&](size_t z_idx, size_t n_idx, double val) {
        stat_n[n_idx] += lm * val;
        if (!is_fixed[z_idx]) {
          gc.z_idx[static_cast<size_t>(gc.n_node)] = z_idx;
          gc.z_val[static_cast<size_t>(gc.n_node)] = val;
          ++gc.n_node;
          rhs_z[z_idx] -= rhs_coef * val;
        }
      };

      if (desc.kind == CollocationMesh::IneqDesc::JERK_POS ||
          desc.kind == CollocationMesh::IneqDesc::JERK_NEG)
      {
        const size_t i = desc.idx;
        const int j = desc.joint;
        const size_t ju = static_cast<size_t>(j);
        const double sgn = (desc.kind == CollocationMesh::IneqDesc::JERK_POS) ? 1.0 : -1.0;
        const size_t za0 = CollocationMesh::z_a(i, j);
        const size_t za1 = CollocationMesh::z_a(i + 1, j);
        push_node_grad(za0, i * kNodePrimal + 12 + ju, -sgn);
        push_node_grad(za1, (i + 1) * kNodePrimal + 12 + ju, +sgn);

        const size_t s = mesh.interval_seg[i];
        const int t_col = mesh.seg_free_idx[s];
        if (t_col >= 0) {
          const double sc_a = mesh.comp_scale[12 + ju];
          const double g_t = -mesh.j_lim[ju] * mesh.gamma[i] * mesh.T_scale[s] / sc_a;
          gc.t_col = t_col;
          gc.t_val = g_t;
          stat_t[static_cast<size_t>(t_col)] += lm * g_t;
          rhs_T[static_cast<size_t>(t_col)] -= rhs_coef * g_t;
        }
      } else {
        const size_t i = desc.idx;
        const auto & td = td_cur[i];
        const size_t n_base = i * kNodePrimal;

        auto add_norm_constraint = [&](
          const Eigen::Vector3d & r_vec,
          const Mat3x6 & dr_dq,
          const Mat3x6 * dr_dv,
          const Mat3x6 * dr_da,
          double inv_lim2)
        {
          // Gradient of ||r||^2 * inv_lim2 - 1 with respect to scaled (q, v, a)
          const Eigen::Matrix<double, 6, 1> gq = (2.0 * inv_lim2) * (dr_dq.transpose() * r_vec);
          for (int j = 0; j < 6; ++j) {
            const size_t ju = static_cast<size_t>(j);
            push_node_grad(CollocationMesh::z_q(i, j), n_base + ju, gq[j] * mesh.comp_scale[ju]);
          }
          if (dr_dv != nullptr) {
            const Eigen::Matrix<double, 6, 1> gv = (2.0 * inv_lim2) * (dr_dv->transpose() * r_vec);
            for (int j = 0; j < 6; ++j) {
              const size_t ju = static_cast<size_t>(j);
              push_node_grad(
                CollocationMesh::z_v(i, j), n_base + 6 + ju, gv[j] * mesh.comp_scale[6 + ju]);
            }
          }
          if (dr_da != nullptr) {
            const Eigen::Matrix<double, 6, 1> ga = (2.0 * inv_lim2) * (dr_da->transpose() * r_vec);
            for (int j = 0; j < 6; ++j) {
              const size_t ju = static_cast<size_t>(j);
              push_node_grad(
                CollocationMesh::z_a(i, j), n_base + 12 + ju, ga[j] * mesh.comp_scale[12 + ju]);
            }
          }

          // Exact PSD Gauss-Newton Hessian: lm * (2 * inv_lim2) * J_r^T * J_r
          const double gn_w = lm * 2.0 * inv_lim2;
          if (gn_w > 1e-14) {
            Eigen::Matrix<double, 3, 18> Jr_scaled;
            Jr_scaled.setZero();
            for (int j = 0; j < 6; ++j) {
              const size_t ju = static_cast<size_t>(j);
              if (!is_fixed[CollocationMesh::z_q(i, j)]) {
                Jr_scaled.col(j) = dr_dq.col(j) * mesh.comp_scale[ju];
              }
              if (dr_dv != nullptr && !is_fixed[CollocationMesh::z_v(i, j)]) {
                Jr_scaled.col(6 + j) = dr_dv->col(j) * mesh.comp_scale[6 + ju];
              }
              if (dr_da != nullptr && !is_fixed[CollocationMesh::z_a(i, j)]) {
                Jr_scaled.col(12 + j) = dr_da->col(j) * mesh.comp_scale[12 + ju];
              }
            }
            const int n_cols = (dr_da != nullptr) ? 18 : ((dr_dv != nullptr) ? 12 : 6);
            const size_t z_off = i * kBlockStride;
            for (int a_c = 0; a_c < n_cols; ++a_c) {
              if (is_fixed[z_off + static_cast<size_t>(a_c)]) {
                continue;
              }
              for (int b_c = a_c; b_c < n_cols; ++b_c) {
                if (is_fixed[z_off + static_cast<size_t>(b_c)]) {
                  continue;
                }
                const double dot = Jr_scaled.col(a_c).dot(Jr_scaled.col(b_c));
                if (dot != 0.0) {
                  kkt.add_band(
                    z_off + static_cast<size_t>(a_c), z_off + static_cast<size_t>(b_c), gn_w * dot);
                }
              }
            }
          }
        };

        switch (desc.kind) {
          case CollocationMesh::IneqDesc::CART_VT: {
            const Mat3x6 J_lin = td.J_lwa.topRows<3>();
            add_norm_constraint(td.v_lin, td.dv_lin_dq, &J_lin, nullptr, inv_vt2);
            break;
          }
          case CollocationMesh::IneqDesc::CART_VR: {
            const Mat3x6 J_ang = td.J_lwa.bottomRows<3>();
            add_norm_constraint(td.v_ang, td.dv_ang_dq, &J_ang, nullptr, inv_vr2);
            break;
          }
          case CollocationMesh::IneqDesc::CART_AT:
            add_norm_constraint(td.a_lin, td.da_lin_dq, &td.da_lin_dv, &td.da_lin_da, inv_at2);
            break;
          case CollocationMesh::IneqDesc::CART_AR:
            add_norm_constraint(td.a_ang, td.da_ang_dq, &td.da_ang_dv, &td.da_ang_da, inv_ar2);
            break;
          case CollocationMesh::IneqDesc::SINGULARITY: {
            const double coef = -1.0 / mesh.sigma_safe;
            for (int j = 0; j < 6; ++j) {
              const size_t ju = static_cast<size_t>(j);
              push_node_grad(
                CollocationMesh::z_q(i, j), n_base + ju,
                coef * td.dsigma_dq[j] * mesh.comp_scale[ju]);
            }
            break;
          }
          default:
            break;
        }
      }

      // Rank-1 barrier condensing term: (lm / sm) * grad_g * grad_g^T.
      for (int a_k = 0; a_k < gc.n_node; ++a_k) {
        const size_t za = gc.z_idx[static_cast<size_t>(a_k)];
        const double va = gc.z_val[static_cast<size_t>(a_k)];
        for (int b_k = a_k; b_k < gc.n_node; ++b_k) {
          kkt.add_band(za, gc.z_idx[static_cast<size_t>(b_k)],
            sig_m * va * gc.z_val[static_cast<size_t>(b_k)]);
        }
        if (gc.t_col >= 0) {
          kkt.add_border(za, static_cast<size_t>(gc.t_col), sig_m * va * gc.t_val);
        }
      }
      if (gc.t_col >= 0) {
        kkt.add_T_diag(static_cast<size_t>(gc.t_col), sig_m * gc.t_val * gc.t_val);
      }
    }

    // Box barriers on free nodal variables and free segment durations, plus dual residual.
    double dual_inf = 0.0;
    for (size_t i = 0; i <= N; ++i) {
      const size_t z_off = i * kBlockStride;
      const size_t n_off = i * kNodePrimal;
      for (int c = 0; c < kNodePrimal; ++c) {
        const size_t n_idx = n_off + static_cast<size_t>(c);
        const size_t z_idx = z_off + static_cast<size_t>(c);
        if (mesh.node_fixed[n_idx]) {
          rhs_z[z_idx] = 0.0;
          continue;
        }
        const double sL = xn[n_idx] - lb_n[n_idx];
        const double sU = ub_n[n_idx] - xn[n_idx];
        const double r_stat = stat_n[n_idx] - zL_n[n_idx] + zU_n[n_idx];
        dual_inf = std::max(dual_inf, std::abs(r_stat));
        comp_mu_inf = std::max({comp_mu_inf, std::abs(sL * zL_n[n_idx] - mu),
                                std::abs(sU * zU_n[n_idx] - mu)});
        comp_zero_inf = std::max({comp_zero_inf, std::abs(sL * zL_n[n_idx]),
                                  std::abs(sU * zU_n[n_idx])});

        kkt.add_band(z_idx, z_idx, zL_n[n_idx] / sL + zU_n[n_idx] / sU);
        rhs_z[z_idx] += -stat_n[n_idx] + mu / sL - mu / sU;
        // stat_n and the eliminated inequalities both include lm * grad_g;
        // cancel the duplicate contribution below.
      }
    }
    // Cancel the duplicate lm * grad_g contribution in the condensed RHS.
    for (size_t m = 0; m < n_ineq; ++m) {
      const auto & gc = g_grads[m];
      const double lm = lam[m];
      for (int a_k = 0; a_k < gc.n_node; ++a_k) {
        rhs_z[gc.z_idx[static_cast<size_t>(a_k)]] += lm * gc.z_val[static_cast<size_t>(a_k)];
      }
      if (gc.t_col >= 0) {
        rhs_T[static_cast<size_t>(gc.t_col)] += lm * gc.t_val;
      }
    }
    for (size_t c = 0; c < n_border; ++c) {
      const double sL = xt[c] - lb_t[c];
      const double sU = ub_t[c] - xt[c];
      const double r_stat = stat_t[c] - zL_t[c] + zU_t[c];
      dual_inf = std::max(dual_inf, std::abs(r_stat));
      comp_mu_inf = std::max({comp_mu_inf, std::abs(sL * zL_t[c] - mu),
                              std::abs(sU * zU_t[c] - mu)});
      comp_zero_inf = std::max({comp_zero_inf, std::abs(sL * zL_t[c]),
                                std::abs(sU * zU_t[c])});
      kkt.add_T_diag(c, zL_t[c] / sL + zU_t[c] / sU);
      rhs_T[c] += -stat_t[c] + mu / sL - mu / sU;
    }

    // Scale dual norm for KKT check (standard IPOPT scaling).
    double max_dual_norm = 1.0;
    for (double yv : y_eq) {
      max_dual_norm = std::max(max_dual_norm, std::abs(yv));
    }
    for (double lv : lam) {
      max_dual_norm = std::max(max_dual_norm, lv);
    }
    const double dual_scale = std::max(1.0, max_dual_norm / 100.0);
    const double dual_inf_scaled = dual_inf / dual_scale;

    stats.eq_inf = eq_inf;
    stats.ineq_inf = ineq_inf;
    stats.dual_inf = dual_inf_scaled;
    stats.comp_inf = comp_zero_inf;
    stats.mu = mu;

    const double prim_inf = std::max({eq_inf, ineq_inf, prim_slack_inf});
    // Overall KKT convergence check.
    if (eq_inf < 1e-7 && ineq_inf < 1e-5 && prim_slack_inf < 1e-6 &&
        dual_inf_scaled < 2e-3 && comp_zero_inf < 2e-5 && mu <= 1e-5)
    {
      stats.converged = true;
      stats.kkt_optimal = true;
      stats.primal_feasible = true;
      break;
    }

    // Farkas dual-divergence certificate: detect provably primal-infeasible fixed-dt problems fast.
    if (!mesh.any_free_T && iter >= 15 && max_dual_norm > 1e8 && eq_inf > 2e-3) {
      break;
    }

    // Adaptive barrier parameter update (Waechter & Biegler IPOPT rule):
    // update mu and shift only the barrier RHS terms directly so the already-assembled
    // KKT matrix is factored and solved on the same iteration without wasting an iteration.
    const double e_mu = std::max({dual_inf_scaled, prim_inf, comp_mu_inf / dual_scale});
    if (mu > 1e-7 && e_mu <= 10.0 * mu) {
      const double mu_new = std::max(1e-7, std::min(0.2 * mu, std::pow(mu, 1.45)));
      const double dmu = mu_new - mu;
      for (size_t m = 0; m < n_ineq; ++m) {
        const auto & gc = g_grads[m];
        const double coef = dmu / s_ineq[m];
        for (int a_k = 0; a_k < gc.n_node; ++a_k) {
          rhs_z[gc.z_idx[static_cast<size_t>(a_k)]] -= coef * gc.z_val[static_cast<size_t>(a_k)];
        }
        if (gc.t_col >= 0) {
          rhs_T[static_cast<size_t>(gc.t_col)] -= coef * gc.t_val;
        }
      }
      for (size_t i = 0; i <= N; ++i) {
        const size_t z_off = i * kBlockStride;
        const size_t n_off = i * kNodePrimal;
        for (int c = 0; c < kNodePrimal; ++c) {
          const size_t n_idx = n_off + static_cast<size_t>(c);
          if (!mesh.node_fixed[n_idx]) {
            const double sL = xn[n_idx] - lb_n[n_idx];
            const double sU = ub_n[n_idx] - xn[n_idx];
            rhs_z[z_off + static_cast<size_t>(c)] += dmu / sL - dmu / sU;
          }
        }
      }
      for (size_t c = 0; c < n_border; ++c) {
        const double sL = xt[c] - lb_t[c];
        const double sU = ub_t[c] - xt[c];
        rhs_T[c] += dmu / sL - dmu / sU;
      }
      mu = mu_new;
      stats.mu = mu;
      filter.clear();
    }

    // 2. Factor & solve bordered banded KKT with exact IPOPT inertia correction.
    bool solved = false;
    double delta_w = (last_delta_w > 0.0) ? std::max(1e-8, last_delta_w * 0.33) : 1e-8;
    double delta_c = 1e-9;
    for (int reg_try = 0; reg_try < 18; ++reg_try) {
      if (kkt.factor_and_solve(rhs_z, rhs_T, is_fixed, delta_w, delta_c, dz, dT)) {
        solved = true;
        last_delta_w = delta_w;
        break;
      }
      delta_w = (delta_w < 1e-6) ? 1e-5 : (delta_w * 10.0);
      if (reg_try >= 8) {
        delta_c = std::min(1e-3, delta_c * 10.0);
      }
    }
    if (!solved) {
      break;
    }

    // 3. Recover slack and dual steps + fraction-to-the-boundary step limits.
    const double tau_ftb = std::max(0.99, 1.0 - mu);
    double alpha_s_max = 1.0;
    double alpha_z_max = 1.0;

    for (size_t i = 0; i <= N; ++i) {
      const size_t z_off = i * kBlockStride;
      const size_t n_off = i * kNodePrimal;
      for (int c = 0; c < kNodePrimal; ++c) {
        const size_t n_idx = n_off + static_cast<size_t>(c);
        const size_t z_idx = z_off + static_cast<size_t>(c);
        if (mesh.node_fixed[n_idx]) {
          dzL_n[n_idx] = 0.0;
          dzU_n[n_idx] = 0.0;
          continue;
        }
        const double dx = dz[z_idx];
        const double sL = xn[n_idx] - lb_n[n_idx];
        const double sU = ub_n[n_idx] - xn[n_idx];
        if (dx < 0.0) {
          alpha_s_max = std::min(alpha_s_max, -tau_ftb * sL / dx);
        } else if (dx > 0.0) {
          alpha_s_max = std::min(alpha_s_max, tau_ftb * sU / dx);
        }
        dzL_n[n_idx] = (mu - zL_n[n_idx] * (sL + dx)) / sL;
        dzU_n[n_idx] = (mu - zU_n[n_idx] * (sU - dx)) / sU;
        if (dzL_n[n_idx] < 0.0) {
          alpha_z_max = std::min(alpha_z_max, -tau_ftb * zL_n[n_idx] / dzL_n[n_idx]);
        }
        if (dzU_n[n_idx] < 0.0) {
          alpha_z_max = std::min(alpha_z_max, -tau_ftb * zU_n[n_idx] / dzU_n[n_idx]);
        }
      }
    }
    for (size_t c = 0; c < n_border; ++c) {
      const double dx = dT[c];
      const double sL = xt[c] - lb_t[c];
      const double sU = ub_t[c] - xt[c];
      if (dx < 0.0) {
        alpha_s_max = std::min(alpha_s_max, -tau_ftb * sL / dx);
      } else if (dx > 0.0) {
        alpha_s_max = std::min(alpha_s_max, tau_ftb * sU / dx);
      }
      dzL_t[c] = (mu - zL_t[c] * (sL + dx)) / sL;
      dzU_t[c] = (mu - zU_t[c] * (sU - dx)) / sU;
      if (dzL_t[c] < 0.0) {
        alpha_z_max = std::min(alpha_z_max, -tau_ftb * zL_t[c] / dzL_t[c]);
      }
      if (dzU_t[c] < 0.0) {
        alpha_z_max = std::min(alpha_z_max, -tau_ftb * zU_t[c] / dzU_t[c]);
      }
    }
    for (size_t m = 0; m < n_ineq; ++m) {
      const auto & gc = g_grads[m];
      double g_dx = 0.0;
      for (int k = 0; k < gc.n_node; ++k) {
        g_dx += gc.z_val[static_cast<size_t>(k)] * dz[gc.z_idx[static_cast<size_t>(k)]];
      }
      if (gc.t_col >= 0) {
        g_dx += gc.t_val * dT[static_cast<size_t>(gc.t_col)];
      }
      ds[m] = -(g_cur[m] + s_ineq[m]) - g_dx;
      dlam[m] = (mu - lam[m] * (s_ineq[m] + ds[m])) / s_ineq[m];
      if (ds[m] < 0.0) {
        alpha_s_max = std::min(alpha_s_max, -tau_ftb * s_ineq[m] / ds[m]);
      }
      if (dlam[m] < 0.0) {
        alpha_z_max = std::min(alpha_z_max, -tau_ftb * lam[m] / dlam[m]);
      }
    }
    alpha_s_max = std::clamp(alpha_s_max, 0.0, 1.0);
    alpha_z_max = std::clamp(alpha_z_max, 0.0, 1.0);

    // 4. Filter line search.
    double theta_cur = 0.0, phi_cur = 0.0;
    compute_filter_pair(xn, xt, s_ineq, f_cur, eq_cur, g_cur, mu, theta_cur, phi_cur);

    // Directional derivative of barrier objective phi_mu along (dx, ds).
    double d_phi = 0.0;
    for (size_t i = 0; i <= N; ++i) {
      const size_t z_off = i * kBlockStride;
      const size_t n_off = i * kNodePrimal;
      for (int c = 0; c < kNodePrimal; ++c) {
        const size_t n_idx = n_off + static_cast<size_t>(c);
        if (!mesh.node_fixed[n_idx]) {
          const double g_phi = grad_f_n[n_idx] -
            mu / (xn[n_idx] - lb_n[n_idx]) + mu / (ub_n[n_idx] - xn[n_idx]);
          d_phi += g_phi * dz[z_off + static_cast<size_t>(c)];
        }
      }
    }
    for (size_t c = 0; c < n_border; ++c) {
      const double g_phi = grad_f_t[c] - mu / (xt[c] - lb_t[c]) + mu / (ub_t[c] - xt[c]);
      d_phi += g_phi * dT[c];
    }
    for (size_t m = 0; m < n_ineq; ++m) {
      d_phi -= (mu / s_ineq[m]) * ds[m];
    }

    double alpha = alpha_s_max;
    bool accepted = false;
    bool add_to_filter = false;
    double f_tr = 0.0;

    for (int ls = 0; ls < 12; ++ls) {
      bool valid_tr = true;
      for (size_t i = 0; i <= N; ++i) {
        const size_t z_off = i * kBlockStride;
        const size_t n_off = i * kNodePrimal;
        for (int c = 0; c < kNodePrimal; ++c) {
          const size_t n_idx = n_off + static_cast<size_t>(c);
          if (mesh.node_fixed[n_idx]) {
            xn_tr[n_idx] = xn[n_idx];
          } else {
            xn_tr[n_idx] = xn[n_idx] + alpha * dz[z_off + static_cast<size_t>(c)];
            if (!(xn_tr[n_idx] > lb_n[n_idx] && xn_tr[n_idx] < ub_n[n_idx])) {
              valid_tr = false;
            }
          }
        }
      }
      for (size_t c = 0; c < n_border; ++c) {
        xt_tr[c] = xt[c] + alpha * dT[c];
        if (!(xt_tr[c] > lb_t[c] && xt_tr[c] < ub_t[c])) {
          valid_tr = false;
        }
      }
      for (size_t m = 0; m < n_ineq; ++m) {
        s_tr[m] = s_ineq[m] + alpha * ds[m];
        if (!(s_tr[m] > 0.0)) {
          valid_tr = false;
        }
      }

      if (valid_tr) {
        evaluate_state(xn_tr, xt_tr, false, q_tr, v_tr, a_tr, T_tr, td_tr, f_tr, eq_tr, g_tr);
        double theta_tr = 0.0, phi_tr = 0.0;
        compute_filter_pair(xn_tr, xt_tr, s_tr, f_tr, eq_tr, g_tr, mu, theta_tr, phi_tr);
        if (std::isfinite(theta_tr) && std::isfinite(phi_tr) && filter_accepts(theta_tr, phi_tr)) {
          const bool switch_cond = (theta_cur <= 1e-4) && (d_phi < 0.0) &&
            (alpha * (-d_phi) > 1e-4 * std::pow(theta_cur, 1.1));
          const bool armijo = phi_tr <= phi_cur + 1e-4 * alpha * d_phi;
          const bool suffices = switch_cond ? armijo :
            (theta_tr <= (1.0 - kGammaTheta) * theta_cur ||
             phi_tr <= phi_cur - kGammaPhi * theta_cur);
          if (suffices) {
            accepted = true;
            add_to_filter = !(switch_cond && armijo);
            break;
          }
        }
      }
      alpha *= 0.5;
    }

    if (!accepted) {
      ++stall_count;
      // If genuinely primal-feasible at a larger barrier, drop mu and keep polishing.
      if (stall_count >= 2 && eq_inf < 1e-5 && ineq_inf < 2e-4 && mu > 1e-5) {
        mu = std::max(1e-7, 0.2 * mu);
        filter.clear();
        stall_count = 0;
        continue;
      }
      // On stall, check dual/complementarity KKT optimality explicitly before marking converged;
      // otherwise terminate as a non-optimal primal-feasible fallback so callers can emit a warning.
      if (stall_count >= 3 && eq_inf < 1e-6 && ineq_inf < 1e-4 && mu <= 1e-4) {
        stats.primal_feasible = true;
        const bool kkt_ok = (dual_inf_scaled < 2e-3 && comp_zero_inf < 2e-5 && mu <= 1e-5);
        stats.kkt_optimal = kkt_ok;
        stats.converged = kkt_ok;
        break;
      }
      // Feasibility restoration / watchdog: clear filter and take a tiny damped primal-dual step.
      filter.clear();
      alpha = std::min(alpha_s_max, 0.05);
      for (size_t i = 0; i <= N; ++i) {
        const size_t z_off = i * kBlockStride;
        const size_t n_off = i * kNodePrimal;
        for (int c = 0; c < kNodePrimal; ++c) {
          const size_t n_idx = n_off + static_cast<size_t>(c);
          if (!mesh.node_fixed[n_idx]) {
            const double margin = 1e-8 * (ub_n[n_idx] - lb_n[n_idx]);
            xn_tr[n_idx] = std::clamp(
              xn[n_idx] + alpha * dz[z_off + static_cast<size_t>(c)],
              lb_n[n_idx] + margin, ub_n[n_idx] - margin);
          } else {
            xn_tr[n_idx] = xn[n_idx];
          }
        }
      }
      for (size_t c = 0; c < n_border; ++c) {
        const double margin = 1e-8 * (ub_t[c] - lb_t[c]);
        xt_tr[c] = std::clamp(xt[c] + alpha * dT[c], lb_t[c] + margin, ub_t[c] - margin);
      }
      for (size_t m = 0; m < n_ineq; ++m) {
        s_tr[m] = std::max(1e-9, s_ineq[m] + alpha * ds[m]);
      }
      if (stall_count >= 6) {
        break;
      }
    } else {
      stall_count = 0;
      if (add_to_filter) {
        filter.emplace_back(theta_cur, phi_cur);
      }
    }

    xn.swap(xn_tr);
    xt.swap(xt_tr);
    s_ineq.swap(s_tr);

    // Update duals (damped proportionally on restoration steps).
    const double alpha_z = accepted ? alpha_z_max : std::min(alpha_z_max, alpha);
    for (size_t i = 0; i < N; ++i) {
      for (int j = 0; j < 6; ++j) {
        const size_t ju = static_cast<size_t>(j);
        y_eq[i * kIntervalDual + ju] += alpha * dz[CollocationMesh::z_lq(i, j)];
        y_eq[i * kIntervalDual + 6 + ju] += alpha * dz[CollocationMesh::z_lv(i, j)];
      }
    }
    for (size_t idx = 0; idx < xn.size(); ++idx) {
      if (!mesh.node_fixed[idx]) {
        zL_n[idx] = std::max(1e-12, zL_n[idx] + alpha_z * dzL_n[idx]);
        zU_n[idx] = std::max(1e-12, zU_n[idx] + alpha_z * dzU_n[idx]);
      }
    }
    for (size_t c = 0; c < n_border; ++c) {
      zL_t[c] = std::max(1e-12, zL_t[c] + alpha_z * dzL_t[c]);
      zU_t[c] = std::max(1e-12, zU_t[c] + alpha_z * dzU_t[c]);
    }
    for (size_t m = 0; m < n_ineq; ++m) {
      lam[m] = std::max(1e-12, lam[m] + alpha_z * dlam[m]);
    }

    evaluate_state(xn, xt, true, q_cur, v_cur, a_cur, T_cur, td_cur, f_cur, eq_cur, g_cur);
  }

  // Final primal residual evaluation and write-back into mesh.
  double final_eq = 0.0;
  for (double e : eq_cur) {
    final_eq = std::max(final_eq, std::abs(e));
  }
  double final_ineq = 0.0;
  for (double gv : g_cur) {
    final_ineq = std::max(final_ineq, std::max(0.0, gv));
  }
  stats.eq_inf = final_eq;
  stats.ineq_inf = final_ineq;
  stats.mu = mu;
  if (final_eq < 1e-5 && final_ineq < 2e-3) {
    stats.primal_feasible = true;
  }

  mesh.q = q_cur;
  mesh.v = v_cur;
  mesh.a = a_cur;
  mesh.T = T_cur;
  return stats;
}

// ============================================================================
// Warm-Start Generators (Joint 3rd-Order OTG + Cartesian-Guided Fallback)
// ============================================================================

void sample_cubic_hermite_segment(
  const Vec6 & q0, const Vec6 & v0, const Vec6 & a0,
  const Vec6 & q1, const Vec6 & v1, const Vec6 & a1,
  double T, double t,
  Vec6 & q_out, Vec6 & v_out, Vec6 & a_out)
{
  // Quintic Hermite interpolation on [0, T] for smooth initial guess.
  const double u = std::clamp(t / std::max(1e-9, T), 0.0, 1.0);
  const double u2 = u * u, u3 = u2 * u, u4 = u3 * u, u5 = u4 * u;
  const Vec6 dq = q1 - q0;
  const Vec6 c0 = q0;
  const Vec6 c1 = v0 * T;
  const Vec6 c2 = 0.5 * a0 * T * T;
  const Vec6 c3 = 10.0 * dq - (6.0 * v0 + 4.0 * v1) * T - (1.5 * a0 - 0.5 * a1) * T * T;
  const Vec6 c4 = -15.0 * dq + (8.0 * v0 + 7.0 * v1) * T + (1.5 * a0 - a1) * T * T;
  const Vec6 c5 = 6.0 * dq - 3.0 * (v0 + v1) * T - 0.5 * (a0 - a1) * T * T;
  q_out = c0 + c1 * u + c2 * u2 + c3 * u3 + c4 * u4 + c5 * u5;
  v_out = (c1 + 2.0 * c2 * u + 3.0 * c3 * u2 + 4.0 * c4 * u3 + 5.0 * c5 * u4) / T;
  a_out = (2.0 * c2 + 6.0 * c3 * u + 12.0 * c4 * u2 + 20.0 * c5 * u3) / (T * T);
}

void build_warm_start_mesh(
  const lite6_kinematics::KinematicsEngine & engine,
  const ResolvedWaypointChain & chain,
  const std::vector<CartesianCollocationWaypoint> & waypoints,
  const std::array<lite6_kinematics::JointLimit, 6> & limits,
  const lite6_kinematics::CartesianLimit & cart_limits,
  const MotionOptions & opts,
  bool prefer_cartesian_ik_seed,
  CollocationMesh & mesh,
  const std::function<bool()> & is_cancelled = {})
{
  const size_t M = waypoints.size();
  const auto & model = engine.get_pin_model();
  const auto tcp = engine.tcp_frame_id();
  auto workspace = engine.collision_workspace();

  mesh.M = M;
  mesh.sing_thresh = engine.get_singularity_threshold();
  mesh.sigma_safe = std::clamp(
    std::min(1.25 * mesh.sing_thresh, 0.90 * chain.min_sigma_smooth),
    1.03 * mesh.sing_thresh, 1.35 * mesh.sing_thresh);

  for (size_t j = 0; j < 6; ++j) {
    mesh.q_min[j] = limits[j].min_pos;
    mesh.q_max[j] = limits[j].max_pos;
    mesh.v_lim[j] = limits[j].max_vel * opts.vel_scale;
    mesh.a_lim[j] = limits[j].max_acc * opts.acc_scale;
    mesh.j_lim[j] = limits[j].max_jerk * opts.jerk_scale;
    mesh.comp_scale[j] = 1.0;
    mesh.comp_scale[6 + j] = std::max(0.1, mesh.v_lim[j]);
    mesh.comp_scale[12 + j] = std::max(0.5, mesh.a_lim[j]);
  }
  mesh.vt_lim = cart_limits.max_trans_vel * opts.vel_scale;
  mesh.vr_lim = cart_limits.max_rot_vel * opts.vel_scale;
  mesh.at_lim = cart_limits.max_trans_acc * opts.acc_scale;
  mesh.ar_lim = cart_limits.max_rot_acc * opts.acc_scale;

  // Compute conservative duration lower bounds T_min per segment.
  mesh.seg_dt_fixed.assign(M, false);
  mesh.seg_dt_fixed_val.assign(M, 0.0);
  mesh.T_min.assign(M, 0.01);
  mesh.T_max.assign(M, 60.0);
  std::vector<double> T_est(M, 0.5);

  Eigen::Matrix4d T_prev = engine.forward_pose(to_array(chain.q[0]), workspace.data);
  for (size_t s = 0; s < M; ++s) {
    const Eigen::Matrix4d & T_next = waypoints[s].pose;
    const double dp = (T_next.topRightCorner<3, 1>() - T_prev.topRightCorner<3, 1>()).norm();
    const double dr = rot_angle(T_prev.topLeftCorner<3, 3>(), T_next.topLeftCorner<3, 3>());
    double t_lo = std::max({0.005, 0.5 * dp / mesh.vt_lim, 0.5 * dr / mesh.vr_lim});
    double t_guess = std::max({0.08, 1.35 * dp / mesh.vt_lim, 1.35 * dr / mesh.vr_lim});
    for (int j = 0; j < 6; ++j) {
      const double dq_abs = std::abs(chain.q[s + 1][j] - chain.q[s][j]);
      t_lo = std::max(t_lo, 0.5 * dq_abs / mesh.v_lim[static_cast<size_t>(j)]);
      t_guess = std::max(t_guess, 1.35 * dq_abs / mesh.v_lim[static_cast<size_t>(j)]);
    }
    if (waypoints[s].dt.has_value()) {
      const double dt_ms = std::max(0.002, std::round(*waypoints[s].dt * 1000.0) * 0.001);
      mesh.seg_dt_fixed[s] = true;
      mesh.seg_dt_fixed_val[s] = dt_ms;
      T_est[s] = dt_ms;
      mesh.T_min[s] = std::min(t_lo, 0.5 * T_est[s]);
    } else {
      mesh.T_min[s] = std::min(t_lo, 59.0);
      T_est[s] = std::clamp(t_guess, mesh.T_min[s] + 0.02, 55.0);
    }
    T_prev = T_next;
  }

  // Try native 3rd-order JointPVASolver for a fast, dynamically consistent, joint-feasible warm start.
  std::vector<PVAWaypoint> j_wps(M);
  for (size_t s = 0; s < M; ++s) {
    j_wps[s].loc = waypoints[s].loc;
    j_wps[s].joint_pos = to_array(chain.q[s + 1]);
    if (s + 1 < M && chain.has_v[s + 1]) {
      j_wps[s].velocity = to_array(chain.dq_pin[s + 1]);
      if (chain.has_a[s + 1]) {
        j_wps[s].acceleration = to_array(chain.ddq_pin[s + 1]);
      }
    }
    j_wps[s].dt = waypoints[s].dt;
  }

  JointPVASolver::MultiWaypointResult otg_res;
  SourceLocation dummy_loc;
  std::string dummy_code, dummy_msg;
  const bool have_otg = !prefer_cartesian_ik_seed && JointPVASolver::solve_multi_waypoint(
    to_array(chain.q[0]), j_wps, limits, opts.vel_scale, opts.acc_scale, opts.jerk_scale,
    otg_res, dummy_loc, dummy_code, dummy_msg, is_cancelled);
  if (is_cancelled && is_cancelled()) {
    return;
  }

  // Choose initial per-segment collocation interval counts N_s.
  mesh.T.resize(M);
  mesh.T_scale.resize(M);
  std::vector<size_t> seg_N(M);
  mesh.wp_node.assign(M + 1, 0);
  mesh.gamma.clear();

  for (size_t s = 0; s < M; ++s) {
    double Ts = have_otg ? otg_res.segment_durations[s] : T_est[s];
    if (mesh.seg_dt_fixed[s]) {
      Ts = mesh.seg_dt_fixed_val[s];
    } else {
      Ts = std::clamp(Ts, mesh.T_min[s] + 0.01, 55.0);
    }
    mesh.T[s] = Ts;
    mesh.T_scale[s] = std::max(0.1, Ts);

    // Choose sub-interval count responsive to segment duration (denser for long/fast segments).
    const size_t ns = static_cast<size_t>(std::clamp(
      static_cast<int>(std::ceil(Ts / 0.09)), 8, 22));
    seg_N[s] = ns;
    mesh.wp_node[s + 1] = mesh.wp_node[s] + ns;
    const double g = 1.0 / static_cast<double>(ns);
    for (size_t k = 0; k < ns; ++k) {
      mesh.gamma.push_back(g);
    }
  }
  mesh.N = mesh.wp_node[M];
  mesh.q.assign(mesh.N + 1, Vec6::Zero());
  mesh.v.assign(mesh.N + 1, Vec6::Zero());
  mesh.a.assign(mesh.N + 1, Vec6::Zero());

  if (have_otg && !otg_res.trajectory.samples.empty()) {
    // Sample OTG 1 kHz trajectory onto collocation nodes.
    const auto & smps = otg_res.trajectory.samples;
    double t_acc = 0.0;
    for (size_t s = 0; s < M; ++s) {
      const double Ts = otg_res.segment_durations[s];
      const size_t i0 = mesh.wp_node[s];
      const size_t ns = seg_N[s];
      for (size_t k = 0; k <= ns; ++k) {
        const double t_q = t_acc + Ts * (static_cast<double>(k) / static_cast<double>(ns));
        const double idx_f = std::clamp(t_q * 1000.0, 0.0, static_cast<double>(smps.size() - 1));
        const size_t i_lo = static_cast<size_t>(std::floor(idx_f));
        const size_t i_hi = std::min(smps.size() - 1, i_lo + 1);
        const double frac = idx_f - static_cast<double>(i_lo);
        const size_t ni = i0 + k;
        mesh.q[ni] = (1.0 - frac) * to_vec6(smps[i_lo].q) + frac * to_vec6(smps[i_hi].q);
        mesh.v[ni] = (1.0 - frac) * to_vec6(smps[i_lo].dq) + frac * to_vec6(smps[i_hi].dq);
        mesh.a[ni] = (1.0 - frac) * to_vec6(smps[i_lo].ddq) + frac * to_vec6(smps[i_hi].ddq);
      }
      t_acc += Ts;
    }

    // Check peak Cartesian violation on the OTG seed; if free segments violate Cartesian speed/acc
    // heavily, gently inflate free T_s (and rescale v/a accordingly) for a cleaner IPM start.
    double max_cart_v = 0.0, max_cart_a = 0.0;
    for (size_t i = 0; i <= mesh.N; ++i) {
      CartesianCollocationSolver::TcpDerivatives td;
      CartesianCollocationSolver::evaluate_tcp_derivatives(
        model, workspace.data, tcp, mesh.q[i], mesh.v[i], mesh.a[i], false, td);
      max_cart_v = std::max({max_cart_v, td.v_lin.norm() / mesh.vt_lim, td.v_ang.norm() / mesh.vr_lim});
      max_cart_a = std::max({max_cart_a, td.a_lin.norm() / mesh.at_lim, td.a_ang.norm() / mesh.ar_lim});
    }
    const double scale_up = std::max(max_cart_v, std::sqrt(std::max(1.0, max_cart_a)));
    if (scale_up > 1.15) {
      const double f_scale = std::min(scale_up * 1.05, 4.0);
      for (size_t s = 0; s < M; ++s) {
        if (!mesh.seg_dt_fixed[s]) {
          mesh.T[s] = std::min(55.0, mesh.T[s] * f_scale);
          mesh.T_scale[s] = std::max(0.1, mesh.T[s]);
          for (size_t i = mesh.wp_node[s] + 1; i < mesh.wp_node[s + 1]; ++i) {
            mesh.v[i] /= f_scale;
            mesh.a[i] /= (f_scale * f_scale);
          }
        }
      }
    }
  } else {
    // Fallback / Cartesian-guided warm start: try continuous IK along SLERP/linear SE(3) chord,
    // falling back to quintic Hermite in joint space per segment.
    std::vector<Vec6> wp_v = chain.dq_pin;
    std::vector<Vec6> wp_a = chain.ddq_pin;
    for (size_t k = 1; k < M; ++k) {
      if (!chain.has_v[k]) {
        const Vec6 v_cr = (chain.q[k + 1] - chain.q[k - 1]) / (mesh.T[k - 1] + mesh.T[k]);
        for (int j = 0; j < 6; ++j) {
          wp_v[k][j] = std::clamp(
            v_cr[j], -0.6 * mesh.v_lim[static_cast<size_t>(j)],
            0.6 * mesh.v_lim[static_cast<size_t>(j)]);
        }
      }
    }

    Eigen::Matrix4d T_start = engine.forward_pose(to_array(chain.q[0]), workspace.data);
    for (size_t s = 0; s < M; ++s) {
      const double Ts = mesh.T[s];
      const size_t i0 = mesh.wp_node[s];
      const size_t ns = seg_N[s];
      const Eigen::Matrix4d & T_end = waypoints[s].pose;

      bool cart_ok = prefer_cartesian_ik_seed;
      std::vector<Vec6> q_cart(ns + 1);
      if (cart_ok) {
        q_cart[0] = chain.q[s];
        const Eigen::Vector3d p0 = T_start.topRightCorner<3, 1>();
        const Eigen::Vector3d p1 = T_end.topRightCorner<3, 1>();
        const Eigen::Quaterniond r0(T_start.topLeftCorner<3, 3>());
        const Eigen::Quaterniond r1(T_end.topLeftCorner<3, 3>());
        for (size_t k = 1; k < ns; ++k) {
          const double alpha = static_cast<double>(k) / static_cast<double>(ns);
          const double s_sm = alpha * alpha * (3.0 - 2.0 * alpha);
          Eigen::Matrix4d Tm = Eigen::Matrix4d::Identity();
          Tm.topRightCorner<3, 1>() = (1.0 - s_sm) * p0 + s_sm * p1;
          Tm.topLeftCorner<3, 3>() = r0.slerp(s_sm, r1).toRotationMatrix();
          Vec6 q_sol;
          if (!newton_ik(model, workspace.data, tcp, Tm, q_cart[k - 1], q_sol)) {
            cart_ok = false;
            break;
          }
          for (int j = 0; j < 6; ++j) {
            if (q_sol[j] < mesh.q_min[static_cast<size_t>(j)] ||
                q_sol[j] > mesh.q_max[static_cast<size_t>(j)])
            {
              cart_ok = false;
              break;
            }
          }
          if (!cart_ok || engine.check_singularity(to_array(q_sol), workspace.data) < mesh.sing_thresh) {
            cart_ok = false;
            break;
          }
          q_cart[k] = q_sol;
        }
        q_cart[ns] = chain.q[s + 1];
        if (cart_ok && (q_cart[ns] - q_cart[ns - 1]).cwiseAbs().maxCoeff() > 0.6) {
          cart_ok = false;
        }
      }

      if (cart_ok) {
        const double h = Ts / static_cast<double>(ns);
        for (size_t k = 0; k <= ns; ++k) {
          mesh.q[i0 + k] = q_cart[k];
        }
        for (size_t k = 1; k < ns; ++k) {
          mesh.v[i0 + k] = (q_cart[k + 1] - q_cart[k - 1]) / (2.0 * h);
          mesh.a[i0 + k] = (q_cart[k + 1] - 2.0 * q_cart[k] + q_cart[k - 1]) / (h * h);
        }
      } else {
        for (size_t k = 0; k <= ns; ++k) {
          const double t = Ts * (static_cast<double>(k) / static_cast<double>(ns));
          sample_cubic_hermite_segment(
            chain.q[s], wp_v[s], wp_a[s], chain.q[s + 1], wp_v[s + 1], wp_a[s + 1],
            Ts, t, mesh.q[i0 + k], mesh.v[i0 + k], mesh.a[i0 + k]);
        }
      }
      T_start = T_end;
    }
  }

  // Enforce exact pinned waypoint values.
  for (size_t k = 0; k <= M; ++k) {
    const size_t ni = mesh.wp_node[k];
    mesh.q[ni] = chain.q[k];
    if (chain.has_v[k]) {
      mesh.v[ni] = chain.dq_pin[k];
    }
    if (chain.has_a[k]) {
      mesh.a[ni] = chain.ddq_pin[k];
    }
  }
  mesh.rebuild_metadata(chain);
}

// Bisects selected intervals in the collocation mesh using exact cubic interpolation so
// dynamics defects remain identically zero on the refined mesh.
void refine_mesh_intervals(
  CollocationMesh & mesh,
  const ResolvedWaypointChain & chain,
  const std::vector<bool> & split_interval)
{
  std::vector<double> new_gamma;
  std::vector<Vec6> new_q, new_v, new_a;
  std::vector<size_t> new_wp_node(mesh.M + 1, 0);

  new_q.push_back(mesh.q[0]);
  new_v.push_back(mesh.v[0]);
  new_a.push_back(mesh.a[0]);

  for (size_t s = 0; s < mesh.M; ++s) {
    new_wp_node[s] = new_gamma.size();
    for (size_t i = mesh.wp_node[s]; i < mesh.wp_node[s + 1]; ++i) {
      if (split_interval[i]) {
        const double h = mesh.gamma[i] * mesh.T[s];
        const double hm = 0.5 * h;
        const Vec6 u_j = (mesh.a[i + 1] - mesh.a[i]) / h;
        const Vec6 am = mesh.a[i] + u_j * hm;
        const Vec6 vm = mesh.v[i] + mesh.a[i] * hm + (0.5 * hm * hm) * u_j;
        const Vec6 qm = mesh.q[i] + mesh.v[i] * hm + (0.5 * hm * hm) * mesh.a[i] +
          (hm * hm * hm / 6.0) * u_j;

        new_gamma.push_back(0.5 * mesh.gamma[i]);
        new_q.push_back(qm);
        new_v.push_back(vm);
        new_a.push_back(am);

        new_gamma.push_back(0.5 * mesh.gamma[i]);
        new_q.push_back(mesh.q[i + 1]);
        new_v.push_back(mesh.v[i + 1]);
        new_a.push_back(mesh.a[i + 1]);
      } else {
        new_gamma.push_back(mesh.gamma[i]);
        new_q.push_back(mesh.q[i + 1]);
        new_v.push_back(mesh.v[i + 1]);
        new_a.push_back(mesh.a[i + 1]);
      }
    }
  }
  new_wp_node[mesh.M] = new_gamma.size();

  mesh.gamma = std::move(new_gamma);
  mesh.wp_node = std::move(new_wp_node);
  mesh.q = std::move(new_q);
  mesh.v = std::move(new_v);
  mesh.a = std::move(new_a);
  mesh.rebuild_metadata(chain);
}

// Evaluates the exact piecewise-cubic trajectory at 1 kHz and inspects all joint, Cartesian,
// and singularity limits, marking any interval that has an inter-knot violation.
struct Verify1kHzReport
{
  bool all_ok = true;
  bool cancelled = false;
  double max_pos_exc = 0.0;
  double max_vel_ratio = 0.0;
  double max_acc_ratio = 0.0;
  double max_jerk_ratio = 0.0;
  double max_cart_v_ratio = 0.0;
  double max_cart_a_ratio = 0.0;
  double min_sigma = 1.0;
  std::vector<bool> violating_intervals;
  std::string first_error_code;
  std::string first_error_msg;
  SourceLocation first_error_loc;
};

Verify1kHzReport sample_and_verify_1khz(
  const lite6_kinematics::KinematicsEngine & engine,
  const CollocationMesh & mesh,
  const std::vector<CartesianCollocationWaypoint> & waypoints,
  SampledTrajectory1kHz & out_sampled,
  std::vector<double> & out_knot_times,
  const std::function<bool()> & is_cancelled = {})
{
  Verify1kHzReport rep;
  rep.violating_intervals.assign(mesh.N, false);

  const auto & model = engine.get_pin_model();
  const auto tcp = engine.tcp_frame_id();
  auto workspace = engine.collision_workspace();

  // Build cumulative knot times.
  out_knot_times.resize(mesh.N + 1, 0.0);
  for (size_t i = 0; i < mesh.N; ++i) {
    out_knot_times[i + 1] = out_knot_times[i] + mesh.gamma[i] * mesh.T[mesh.interval_seg[i]];
  }
  const double T_total = out_knot_times.back();
  constexpr double kMaxCollocationDurationSec = 300.0;
  if (!std::isfinite(T_total) || T_total <= 0.0 || T_total > kMaxCollocationDurationSec) {
    rep.all_ok = false;
    rep.first_error_code = "ERR_DURATION_LIMIT";
    rep.first_error_msg = "Cartesian collocation trajectory duration is non-finite or exceeds 300 s limit";
    if (!waypoints.empty()) {
      rep.first_error_loc = waypoints.front().loc;
    }
    return rep;
  }
  const int total_ticks = std::max(2, static_cast<int>(std::llround(T_total * 1000.0)));
  const double T_grid = static_cast<double>(total_ticks) * 0.001;

  out_sampled.samples.clear();
  out_sampled.samples.reserve(static_cast<size_t>(total_ticks + 1));
  out_sampled.duration_sec = T_grid;

  auto record_err = [&](const SourceLocation & loc, const std::string & code, const std::string & msg) {
    rep.all_ok = false;
    if (rep.first_error_code.empty()) {
      rep.first_error_loc = loc;
      rep.first_error_code = code;
      rep.first_error_msg = msg;
    }
  };

  size_t cur_i = 0;
  for (int tick = 0; tick <= total_ticks; ++tick) {
    if (tick % 64 == 0 && is_cancelled && is_cancelled()) {
      rep.all_ok = false;
      rep.cancelled = true;
      return rep;
    }
    const double t_tick = static_cast<double>(tick) * 0.001;
    // Map [0, T_grid] linearly onto [0, T_total] (exact 1:1 after 1 ms snapping).
    const double t = (tick == total_ticks) ? T_total : std::min(T_total, t_tick * (T_total / T_grid));
    while (cur_i + 1 < mesh.N && out_knot_times[cur_i + 1] < t) {
      ++cur_i;
    }
    const double h = std::max(1e-12, out_knot_times[cur_i + 1] - out_knot_times[cur_i]);
    const double tau = std::clamp(t - out_knot_times[cur_i], 0.0, h);
    const Vec6 jerk = (mesh.a[cur_i + 1] - mesh.a[cur_i]) / h;
    Vec6 a = mesh.a[cur_i] + jerk * tau;
    Vec6 v = mesh.v[cur_i] + mesh.a[cur_i] * tau + (0.5 * tau * tau) * jerk;
    Vec6 q = mesh.q[cur_i] + mesh.v[cur_i] * tau + (0.5 * tau * tau) * mesh.a[cur_i] +
      (tau * tau * tau / 6.0) * jerk;

    if (tick == 0) {
      q = mesh.q.front();
      v.setZero();
      a.setZero();
    } else if (tick == total_ticks) {
      q = mesh.q.back();
      v.setZero();
      a.setZero();
    }

    const size_t seg = mesh.interval_seg[cur_i];
    const auto & loc = waypoints[seg].loc;

    SampledTrajectory1kHz::Sample smp;
    smp.time_sec = t_tick;
    smp.path_u = (T_grid > 1e-9) ? (t_tick / T_grid) : 1.0;
    smp.q = to_array(q);
    smp.dq = to_array(v);
    smp.ddq = to_array(a);
    smp.loc = loc;
    out_sampled.samples.push_back(smp);

    // Check joint bounds.
    for (int j = 0; j < 6; ++j) {
      const size_t ju = static_cast<size_t>(j);
      const double pos_exc = std::max({0.0, mesh.q_min[ju] - q[j], q[j] - mesh.q_max[ju]});
      rep.max_pos_exc = std::max(rep.max_pos_exc, pos_exc);
      if (pos_exc > lite6_kinematics::tolerance::kTrajectoryPosition) {
        rep.violating_intervals[cur_i] = true;
        record_err(loc, "ERR_JOINT_POS_LIMIT",
          "Collocation trajectory violates joint" + std::to_string(j + 1) + " position limit");
      }
      const double vr = std::abs(v[j]) / mesh.v_lim[ju];
      rep.max_vel_ratio = std::max(rep.max_vel_ratio, vr);
      if (std::abs(v[j]) > mesh.v_lim[ju] + lite6_kinematics::tolerance::kTrajectoryVelocity) {
        rep.violating_intervals[cur_i] = true;
        record_err(loc, "ERR_PVA_VEL_LIMIT",
          "Collocation trajectory violates joint" + std::to_string(j + 1) + " velocity limit");
      }
      const double ar = std::abs(a[j]) / mesh.a_lim[ju];
      rep.max_acc_ratio = std::max(rep.max_acc_ratio, ar);
      if (std::abs(a[j]) > mesh.a_lim[ju] + lite6_kinematics::tolerance::kTrajectoryAcceleration) {
        rep.violating_intervals[cur_i] = true;
        record_err(loc, "ERR_PVA_ACC_LIMIT",
          "Collocation trajectory violates joint" + std::to_string(j + 1) + " acceleration limit");
      }
      if (tick > 0) {
        const double dj = std::abs(smp.ddq[ju] - out_sampled.samples[static_cast<size_t>(tick - 1)].ddq[ju]) * 1000.0;
        rep.max_jerk_ratio = std::max(rep.max_jerk_ratio, dj / mesh.j_lim[ju]);
        if (dj > mesh.j_lim[ju] * 1.001 + 1e-3) {
          rep.violating_intervals[cur_i] = true;
          record_err(loc, "ERR_PVA_JERK_LIMIT",
            "Collocation trajectory violates joint" + std::to_string(j + 1) + " jerk limit");
        }
      }
    }

    // Check Cartesian velocity, classical acceleration, and singularity at every 1 kHz sample.
    pinocchio::forwardKinematics(model, workspace.data, q, v, a);
    pinocchio::updateFramePlacement(model, workspace.data, tcp);
    const auto vel = pinocchio::getFrameVelocity(
      model, workspace.data, tcp, pinocchio::LOCAL_WORLD_ALIGNED);
    const auto acc = pinocchio::getFrameClassicalAcceleration(
      model, workspace.data, tcp, pinocchio::LOCAL_WORLD_ALIGNED);

    const double r_vt = vel.linear().norm() / mesh.vt_lim;
    const double r_vr = vel.angular().norm() / mesh.vr_lim;
    const double r_at = acc.linear().norm() / mesh.at_lim;
    const double r_ar = acc.angular().norm() / mesh.ar_lim;
    rep.max_cart_v_ratio = std::max({rep.max_cart_v_ratio, r_vt, r_vr});
    rep.max_cart_a_ratio = std::max({rep.max_cart_a_ratio, r_at, r_ar});

    if (r_vt > 1.0 + 1e-4 || r_vr > 1.0 + 1e-4) {
      rep.violating_intervals[cur_i] = true;
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(4)
          << "MovePVA Cartesian segment " << (seg + 1) << " exceeds scaled Cartesian velocity limit at t="
          << t_tick << " s (trans=" << vel.linear().norm() << "/" << mesh.vt_lim
          << " m/s, rot=" << vel.angular().norm() << "/" << mesh.vr_lim << " rad/s)";
      record_err(loc, "ERR_PVA_CART_VEL_LIMIT", oss.str());
    }
    if (r_at > 1.0 + 1e-4 || r_ar > 1.0 + 1e-4) {
      rep.violating_intervals[cur_i] = true;
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(4)
          << "MovePVA Cartesian segment " << (seg + 1) << " exceeds scaled Cartesian acceleration limit at t="
          << t_tick << " s (trans=" << acc.linear().norm() << "/" << mesh.at_lim
          << " m/s^2, rot=" << acc.angular().norm() << "/" << mesh.ar_lim << " rad/s^2)";
      record_err(loc, "ERR_PVA_CART_ACC_LIMIT", oss.str());
    }

    if (tick % 5 == 0 || tick == total_ticks) {
      const double sigma = engine.check_singularity(smp.q, workspace.data);
      rep.min_sigma = std::min(rep.min_sigma, sigma);
      if (sigma < mesh.sing_thresh) {
        rep.violating_intervals[cur_i] = true;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6)
            << "Kinematic singularity in MovePVA Cartesian segment " << (seg + 1)
            << " at t=" << t_tick << " s (sigma_min=" << sigma
            << " < singularity_threshold=" << mesh.sing_thresh << ")";
        record_err(loc, "ERR_SINGULARITY", oss.str());
      }
    }
  }
  return rep;
}

// Checks whether the sampled 1 kHz trajectory is free of self/environment Coal collisions,
// returning the first colliding segment index (-1 if completely collision-free).
int find_first_colliding_segment(
  const lite6_kinematics::KinematicsEngine & engine,
  const SampledTrajectory1kHz & sampled,
  const std::vector<double> & seg_end_times,
  std::string & out_detail,
  const std::function<bool()> & is_cancelled = {},
  bool * out_cancelled = nullptr)
{
  if (out_cancelled != nullptr) {
    *out_cancelled = false;
  }
  auto workspace = engine.collision_workspace();
  auto segment_of_time = [&](double t) -> int {
    for (size_t s = 0; s < seg_end_times.size(); ++s) {
      if (t <= seg_end_times[s] + 1e-6) {
        return static_cast<int>(s);
      }
    }
    return static_cast<int>(seg_end_times.size()) - 1;
  };

  if (sampled.samples.empty()) {
    return -1;
  }
  if (is_cancelled && is_cancelled()) {
    if (out_cancelled != nullptr) {
      *out_cancelled = true;
    }
    return -1;
  }
  if (engine.check_collision(sampled.samples[0].q, workspace, &out_detail)) {
    return 0;
  }
  std::array<double, 6> last_checked = sampled.samples[0].q;
  constexpr double kCollisionCheckStepRad = 0.025;
  for (size_t k = 1; k < sampled.samples.size(); ++k) {
    if (k % 64 == 0 && is_cancelled && is_cancelled()) {
      if (out_cancelled != nullptr) {
        *out_cancelled = true;
      }
      return -1;
    }
    const auto & curr = sampled.samples[k];
    double max_step = 0.0;
    for (size_t i = 0; i < 6; ++i) {
      max_step = std::max(max_step, std::abs(curr.q[i] - last_checked[i]));
    }
    const bool is_last = (k + 1 == sampled.samples.size());
    if (!is_last && max_step < kCollisionCheckStepRad) {
      continue;
    }
    const int subdivs = std::max(1, static_cast<int>(std::ceil(max_step / 0.04)));
    for (int sub = 1; sub <= subdivs; ++sub) {
      const double alpha = static_cast<double>(sub) / static_cast<double>(subdivs);
      std::array<double, 6> q_interp{};
      for (size_t i = 0; i < 6; ++i) {
        q_interp[i] = last_checked[i] + alpha * (curr.q[i] - last_checked[i]);
      }
      if (engine.check_collision(q_interp, workspace, &out_detail)) {
        return segment_of_time(curr.time_sec);
      }
    }
    last_checked = curr.q;
  }
  return -1;
}

}  // namespace

// ============================================================================
// Exact Analytical Pinocchio TCP Derivatives + Closed-Form Kinematic Hessian
// ============================================================================

void CartesianCollocationSolver::evaluate_tcp_derivatives(
  const pinocchio::Model & model,
  pinocchio::Data & data,
  pinocchio::FrameIndex tcp_frame_id,
  const Vec6 & q,
  const Vec6 & dq,
  const Vec6 & ddq,
  bool compute_derivatives,
  TcpDerivatives & out)
{
  const auto & frame = model.frames[tcp_frame_id];
  if (!compute_derivatives) {
    pinocchio::forwardKinematics(model, data, q, dq, ddq);
    pinocchio::computeJointJacobians(model, data, q);
    data.oMf[tcp_frame_id] = data.oMi[frame.parentJoint] * frame.placement;
    const auto & oMf = data.oMf[tcp_frame_id];
    out.p = oMf.translation();
    out.R = oMf.rotation();
    const auto v_lwa = pinocchio::getFrameVelocity(
      model, data, tcp_frame_id, pinocchio::LOCAL_WORLD_ALIGNED);
    const auto a_cls = pinocchio::getFrameClassicalAcceleration(
      model, data, tcp_frame_id, pinocchio::LOCAL_WORLD_ALIGNED);
    out.v_lin = v_lwa.linear();
    out.v_ang = v_lwa.angular();
    out.a_lin = a_cls.linear();
    out.a_ang = a_cls.angular();

    pinocchio::Data::Matrix6x J(6, 6);
    J.setZero();
    pinocchio::getFrameJacobian(model, data, tcp_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
    out.J_lwa = J;
    Mat6 Jn = out.J_lwa;
    Jn.topRows<3>() /= kCharLength;
    const Mat6 A = Jn * Jn.transpose() + kSingEps * Mat6::Identity();
    const Mat6 A_inv = A.ldlt().solve(Mat6::Identity());
    out.sigma_smooth = 1.0 / std::sqrt(std::max(1e-30, A_inv.trace()));
    out.dsigma_dq.setZero();
    return;
  }

  pinocchio::computeForwardKinematicsDerivatives(model, data, q, dq, ddq);
  data.oMf[tcp_frame_id] = data.oMi[frame.parentJoint] * frame.placement;
  const auto & oMf = data.oMf[tcp_frame_id];

  out.p = oMf.translation();
  out.R = oMf.rotation();
  const auto v_lwa = pinocchio::getFrameVelocity(
    model, data, tcp_frame_id, pinocchio::LOCAL_WORLD_ALIGNED);
  const auto a_cls = pinocchio::getFrameClassicalAcceleration(
    model, data, tcp_frame_id, pinocchio::LOCAL_WORLD_ALIGNED);
  out.v_lin = v_lwa.linear();
  out.v_ang = v_lwa.angular();
  out.a_lin = a_cls.linear();
  out.a_ang = a_cls.angular();

  pinocchio::Data::Matrix3x dv_lin_dq(3, 6), da_lin_dq(3, 6), da_lin_dv(3, 6), da_lin_da(3, 6);
  dv_lin_dq.setZero();
  da_lin_dq.setZero();
  da_lin_dv.setZero();
  da_lin_da.setZero();
  pinocchio::getPointClassicAccelerationDerivatives(
    model, data, frame.parentJoint, frame.placement, pinocchio::LOCAL_WORLD_ALIGNED,
    dv_lin_dq, da_lin_dq, da_lin_dv, da_lin_da);

  pinocchio::Data::Matrix6x dv_sp_dq(6, 6), da_sp_dq(6, 6), da_sp_dv(6, 6), da_sp_da(6, 6);
  dv_sp_dq.setZero();
  da_sp_dq.setZero();
  da_sp_dv.setZero();
  da_sp_da.setZero();
  pinocchio::getFrameAccelerationDerivatives(
    model, data, tcp_frame_id, pinocchio::LOCAL_WORLD_ALIGNED,
    dv_sp_dq, da_sp_dq, da_sp_dv, da_sp_da);

  out.J_lwa = da_sp_da;
  out.dv_lin_dq = dv_lin_dq;
  out.da_lin_dq = da_lin_dq;
  out.da_lin_dv = da_lin_dv;
  out.da_lin_da = da_lin_da;
  out.dv_ang_dq = dv_sp_dq.bottomRows<3>();
  out.da_ang_dq = da_sp_dq.bottomRows<3>();
  out.da_ang_dv = da_sp_dv.bottomRows<3>();
  out.da_ang_da = da_sp_da.bottomRows<3>();

  // Smooth singularity proxy sigma_smooth = [tr((Jn Jn^T + eps I)^-1)]^(-1/2) <= sigma_min(Jn)
  // and its exact analytical gradient using the closed-form 6-DOF serial revolute kinematic Hessian.
  Mat6 Jn = out.J_lwa;
  Jn.topRows<3>() /= kCharLength;
  const Mat6 A = Jn * Jn.transpose() + kSingEps * Mat6::Identity();
  const Mat6 A_inv = A.ldlt().solve(Mat6::Identity());
  const double tr_inv = std::max(1e-30, A_inv.trace());
  out.sigma_smooth = 1.0 / std::sqrt(tr_inv);

  const double sig3 = out.sigma_smooth * out.sigma_smooth * out.sigma_smooth;
  const Mat6 W = sig3 * (A_inv * (A_inv * Jn));
  const double inv_L = 1.0 / kCharLength;

  // For serial 6R chain with LWA Jacobian columns s_j = [v_j; w_j]:
  //   m < j: d(s_j)/d(q_m) = [w_m x v_j; w_m x w_j]
  //   m >= j: d(s_j)/d(q_m) = [w_j x v_m; 0]
  for (int m = 0; m < 6; ++m) {
    const Eigen::Vector3d vm = out.J_lwa.block<3, 1>(0, m);
    const Eigen::Vector3d wm = out.J_lwa.block<3, 1>(3, m);
    double grad_m = 0.0;
    for (int j = 0; j < 6; ++j) {
      const Eigen::Vector3d vj = out.J_lwa.block<3, 1>(0, j);
      const Eigen::Vector3d wj = out.J_lwa.block<3, 1>(3, j);
      Eigen::Vector3d dv_dq, dw_dq;
      if (m < j) {
        dv_dq = wm.cross(vj);
        dw_dq = wm.cross(wj);
      } else {
        dv_dq = wj.cross(vm);
        dw_dq.setZero();
      }
      grad_m += (W.block<3, 1>(0, j).dot(dv_dq) * inv_L) + W.block<3, 1>(3, j).dot(dw_dq);
    }
    out.dsigma_dq[m] = grad_m;
  }
}

// ============================================================================
// Top-Level Solve Pipeline
// ============================================================================

bool CartesianCollocationSolver::solve(
  const lite6_kinematics::KinematicsEngine & engine,
  const std::array<double, 6> & start_q_arr,
  const std::vector<CartesianCollocationWaypoint> & waypoints,
  const std::array<lite6_kinematics::JointLimit, 6> & joint_limits,
  const lite6_kinematics::CartesianLimit & cart_limits,
  const MotionOptions & options,
  CartesianCollocationResult & out_result,
  std::vector<D> & out_diagnostics,
  const std::function<bool()> & is_cancelled)
{
  if (waypoints.empty()) {
    return false;
  }
  auto check_cancelled = [&]() -> bool {
    if (is_cancelled && is_cancelled()) {
      add_diag(
        out_diagnostics, D::STAGE_PLANNING, waypoints.front().loc,
        "ERR_CANCELLED", "Motion cancelled by supervisor");
      return true;
    }
    return false;
  };
  if (check_cancelled()) {
    return false;
  }

  const Vec6 start_q = to_vec6(start_q_arr);
  const auto & model = engine.get_pin_model();
  const auto tcp = engine.tcp_frame_id();

  // Stage 0: Global IK branch selection via Dynamic Programming (up to 3 top chains).
  std::vector<ResolvedWaypointChain> chains;
  if (!resolve_ik_chains_dp(
        engine, start_q, waypoints, joint_limits, cart_limits, options, 3,
        chains, out_diagnostics, is_cancelled))
  {
    return false;
  }

  auto try_solve_with_waypoints = [&](
    const std::vector<CartesianCollocationWaypoint> & active_wps,
    const ResolvedWaypointChain & active_chain,
    bool use_cart_warm_start,
    CartesianCollocationResult & cand_res,
    int & out_colliding_seg,
    std::string & out_col_detail,
    Verify1kHzReport & out_rep) -> bool
  {
    if (is_cancelled && is_cancelled()) {
      return false;
    }
    CollocationMesh mesh;
    build_warm_start_mesh(
      engine, active_chain, active_wps, joint_limits, cart_limits, options,
      use_cart_warm_start, mesh, is_cancelled);
    if (is_cancelled && is_cancelled()) {
      return false;
    }

    // Stage 2: Initial coarse-mesh IPM solve.
    int total_iters = 0;
    const double initial_mu = mesh.any_free_T ? 0.08 : 0.01;
    auto stats = run_collocation_ipm(model, tcp, mesh, 85, initial_mu, is_cancelled);
    total_iters += stats.iterations;
    if (stats.cancelled) {
      return false;
    }
    if (!stats.primal_feasible) {
      // Retry with a slightly adjusted initial barrier (and time inflation if free T exists).
      if (mesh.any_free_T) {
        for (size_t s = 0; s < mesh.M; ++s) {
          if (!mesh.seg_dt_fixed[s]) {
            mesh.T[s] = std::min(55.0, mesh.T[s] * 1.4);
          }
        }
        auto stats2 = run_collocation_ipm(model, tcp, mesh, 90, 0.2, is_cancelled);
        total_iters += stats2.iterations;
        stats = stats2;
      } else {
        auto stats2 = run_collocation_ipm(model, tcp, mesh, 110, 1e-3, is_cancelled);
        total_iters += stats2.iterations;
        stats = stats2;
      }
      if (stats.cancelled || !stats.primal_feasible) {
        return false;
      }
    }

    // Stage 3: Adaptive collocation mesh refinement until 1 kHz limits strictly hold.
    SampledTrajectory1kHz sampled;
    std::vector<double> knot_times;
    int refinements = 0;
    bool rep_fresh = false;
    auto run_refinement_rounds = [&](int max_rounds, int ipm_iters, double refine_mu) {
      for (int round = 0; round < max_rounds; ++round) {
        if (is_cancelled && is_cancelled()) {
          return;
        }
        out_rep = sample_and_verify_1khz(
          engine, mesh, active_wps, sampled, knot_times, is_cancelled);
        rep_fresh = true;
        if (out_rep.cancelled || out_rep.all_ok) {
          break;
        }
        // Bisect violating intervals (plus immediate neighbors for smooth curvature capture).
        std::vector<bool> to_split = out_rep.violating_intervals;
        size_t n_split = 0;
        for (size_t i = 0; i < mesh.N; ++i) {
          if (out_rep.violating_intervals[i]) {
            if (i > 0 && mesh.interval_seg[i - 1] == mesh.interval_seg[i]) {
              to_split[i - 1] = true;
            }
            if (i + 1 < mesh.N && mesh.interval_seg[i + 1] == mesh.interval_seg[i]) {
              to_split[i + 1] = true;
            }
          }
        }
        for (bool b : to_split) {
          if (b) {
            ++n_split;
          }
        }
        if (n_split == 0 || mesh.N + n_split > 640) {
          break;
        }
        refine_mesh_intervals(mesh, active_chain, to_split);
        ++refinements;
        mesh.cart_safety = std::max(0.96, mesh.cart_safety - 0.008);
        auto r_stats = run_collocation_ipm(model, tcp, mesh, ipm_iters, refine_mu, is_cancelled);
        total_iters += r_stats.iterations;
        stats = r_stats;
        rep_fresh = false;
        if (stats.cancelled) {
          break;
        }
      }
    };

    run_refinement_rounds(4, 50, 1e-3);
    if (is_cancelled && is_cancelled()) {
      return false;
    }

    // Snap free segment durations to exact 1 ms boundaries (ceil to preserve limit headroom)
    // and run a fast fixed-duration polish (+ localized post-snap refinement if needed) so
    // every waypoint lands on an exact 1 kHz tick with zero inter-knot limit bulge!
    if (mesh.any_free_T) {
      for (size_t s = 0; s < mesh.M; ++s) {
        if (!mesh.seg_dt_fixed[s]) {
          const double snapped = std::max(0.002, std::ceil(mesh.T[s] * 1000.0 - 1e-9) * 0.001);
          mesh.T[s] = snapped;
          mesh.seg_dt_fixed[s] = true;
          mesh.seg_dt_fixed_val[s] = snapped;
        }
      }
      mesh.rebuild_metadata(active_chain);
      auto p_stats = run_collocation_ipm(model, tcp, mesh, 55, 5e-4, is_cancelled);
      total_iters += p_stats.iterations;
      stats = p_stats;
      if (stats.cancelled) {
        return false;
      }
      if (stats.eq_inf > 1e-5) {
        auto p_stats2 = run_collocation_ipm(model, tcp, mesh, 60, 1e-4, is_cancelled);
        total_iters += p_stats2.iterations;
        stats = p_stats2;
        if (stats.cancelled) {
          return false;
        }
      }
      rep_fresh = false;
      run_refinement_rounds(3, 45, 5e-4);
      if (is_cancelled && is_cancelled()) {
        return false;
      }
    }

    if (stats.eq_inf > 1e-5) {
      return false;
    }
    if (!rep_fresh) {
      out_rep = sample_and_verify_1khz(
        engine, mesh, active_wps, sampled, knot_times, is_cancelled);
    }
    if (out_rep.cancelled || !out_rep.all_ok) {
      return false;
    }

    // Check Stage 5 Coal collision early so we can trigger collision-guided recovery if needed.
    std::vector<double> seg_end_times(mesh.M, 0.0);
    for (size_t s = 0; s < mesh.M; ++s) {
      seg_end_times[s] = knot_times[mesh.wp_node[s + 1]];
    }
    bool col_cancelled = false;
    out_colliding_seg = find_first_colliding_segment(
      engine, sampled, seg_end_times, out_col_detail, is_cancelled, &col_cancelled);
    if (col_cancelled) {
      return false;
    }

    std::vector<SourceLocation> seg_locs;
    seg_locs.reserve(active_wps.size());
    for (const auto & w : active_wps) {
      seg_locs.push_back(w.loc);
    }

    cand_res.trajectory = std::move(sampled);
    cand_res.segment_durations = mesh.T;
    cand_res.total_duration = knot_times.back();
    cand_res.total_ipm_iterations = total_iters;
    cand_res.mesh_refinements = refinements;
    cand_res.used_cartesian_warm_start = use_cart_warm_start;
    cand_res.kkt_optimal = stats.kkt_optimal;
    cand_res.converged = stats.converged || stats.primal_feasible;
    cand_res.cartesian_path = std::make_shared<CollocationCartesianPath>(
      model, tcp, std::move(knot_times), mesh.q, mesh.v, mesh.a,
      mesh.wp_node, std::move(seg_locs), options);
    return true;
  };

  auto emit_suboptimal_warning_if_needed = [&](const CartesianCollocationResult & res) {
    if (!res.kkt_optimal) {
      add_warning(
        out_diagnostics, D::STAGE_PLANNING, waypoints.front().loc, "WARN_COLLOCATION_SUBOPTIMAL",
        "Cartesian collocation IPM terminated on a primal-feasible fallback before full KKT "
        "optimality; trajectory passed 1 kHz kinematic and collision verification but may be "
        "slightly slower than time-optimal");
    }
  };

  Verify1kHzReport last_rep;
  std::string last_col_detail;
  bool had_collision_only = false;

  // Try each DP IK branch chain (starting with the globally optimal chain).
  for (size_t ch_idx = 0; ch_idx < chains.size(); ++ch_idx) {
    if (check_cancelled()) {
      return false;
    }
    const auto & chain = chains[ch_idx];
    for (int seed_mode = 0; seed_mode < 2; ++seed_mode) {
      if (check_cancelled()) {
        return false;
      }
      const bool use_cart_seed = (seed_mode == 1);
      CartesianCollocationResult cand_res;
      int col_seg = -1;
      std::string col_detail;
      Verify1kHzReport rep;

      if (!try_solve_with_waypoints(
            waypoints, chain, use_cart_seed, cand_res, col_seg, col_detail, rep))
      {
        if (check_cancelled()) {
          return false;
        }
        if (!rep.first_error_code.empty()) {
          last_rep = rep;
        }
        continue;
      }

      if (col_seg < 0) {
        emit_suboptimal_warning_if_needed(cand_res);
        out_result = std::move(cand_res);
        return true;
      }

      had_collision_only = true;
      last_col_detail = col_detail;

      // Collision-guided waypoint midpoint recovery: insert a collision-free intermediate pose
      // along vertical, outward-radial, or lateral offsets from the SE(3) chord midpoint.
      // Supports both free-dt and fixed-dt segments (splitting fixed dt into exact 1 ms halves).
      const auto & col_wp = waypoints[static_cast<size_t>(col_seg)];
      const bool can_split = !col_wp.dt.has_value() || (*col_wp.dt >= 0.004);
      if (can_split) {
        auto workspace = engine.collision_workspace();
        const Eigen::Matrix4d T_a = (col_seg == 0) ?
          engine.forward_pose(to_array(start_q), workspace.data) :
          waypoints[static_cast<size_t>(col_seg - 1)].pose;
        const Eigen::Matrix4d & T_b = col_wp.pose;
        const Eigen::Vector3d pa = T_a.topRightCorner<3, 1>();
        const Eigen::Vector3d pb = T_b.topRightCorner<3, 1>();
        const Eigen::Quaterniond ra(T_a.topLeftCorner<3, 3>());
        const Eigen::Quaterniond rb(T_b.topLeftCorner<3, 3>());
        const Eigen::Vector3d p_chord = 0.5 * (pa + pb);

        Eigen::Vector3d r_hat(p_chord.x(), p_chord.y(), 0.0);
        if (r_hat.norm() > 1e-6) {
          r_hat.normalize();
        } else {
          r_hat = Eigen::Vector3d(1.0, 0.0, 0.0);
        }
        const Eigen::Vector3d t_hat(-r_hat.y(), r_hat.x(), 0.0);
        const Eigen::Vector3d z_hat(0.0, 0.0, 1.0);

        const std::array<Eigen::Vector3d, 8> offsets{{
          Eigen::Vector3d::Zero(),
          0.06 * z_hat,
          0.14 * z_hat,
          0.08 * r_hat + 0.04 * z_hat,
          0.12 * r_hat,
          0.08 * t_hat + 0.04 * z_hat,
          -0.08 * t_hat + 0.04 * z_hat,
          0.22 * z_hat
        }};

        for (const auto & off : offsets) {
          if (check_cancelled()) {
            return false;
          }
          Eigen::Matrix4d T_mid = Eigen::Matrix4d::Identity();
          T_mid.topRightCorner<3, 1>() = p_chord + off;
          T_mid.topLeftCorner<3, 3>() = ra.slerp(0.5, rb).toRotationMatrix();

          std::vector<CartesianCollocationWaypoint> aug_wps = waypoints;
          CartesianCollocationWaypoint mid_wp;
          mid_wp.loc = col_wp.loc;
          mid_wp.pose = T_mid;
          if (col_wp.dt.has_value()) {
            const double T_seg = *col_wp.dt;
            const double dt1 = std::clamp(
              std::round(0.5 * T_seg * 1000.0) * 0.001, 0.002, T_seg - 0.002);
            const double dt2 = T_seg - dt1;
            mid_wp.dt = dt1;
            aug_wps[static_cast<size_t>(col_seg)].dt = dt2;
          }
          aug_wps.insert(aug_wps.begin() + col_seg, mid_wp);

          std::vector<ResolvedWaypointChain> aug_chains;
          std::vector<D> aug_diags;
          if (!resolve_ik_chains_dp(
                engine, start_q, aug_wps, joint_limits, cart_limits, options, 1,
                aug_chains, aug_diags, is_cancelled))
          {
            if (check_cancelled()) {
              return false;
            }
            continue;
          }
          CartesianCollocationResult aug_res;
          int aug_col = -1;
          std::string aug_col_detail;
          Verify1kHzReport aug_rep;
          if (try_solve_with_waypoints(
                aug_wps, aug_chains.front(), false, aug_res, aug_col, aug_col_detail, aug_rep) &&
              aug_col < 0)
          {
            // Merge the split segment durations back so segment_durations matches waypoints.size().
            if (aug_res.segment_durations.size() == waypoints.size() + 1) {
              const size_t cs = static_cast<size_t>(col_seg);
              aug_res.segment_durations[cs] += aug_res.segment_durations[cs + 1];
              aug_res.segment_durations.erase(aug_res.segment_durations.begin() + col_seg + 1);
            }
            emit_suboptimal_warning_if_needed(aug_res);
            out_result = std::move(aug_res);
            return true;
          }
        }
      }
    }
  }

  if (had_collision_only) {
    add_diag(
      out_diagnostics, D::STAGE_COLLISION, waypoints.front().loc, "ERR_COLLISION",
      "Direct-transcription Cartesian MovePVA encountered collision: " + last_col_detail);
  } else if (!last_rep.first_error_code.empty()) {
    add_diag(
      out_diagnostics, D::STAGE_KINEMATIC_LIMIT, last_rep.first_error_loc,
      last_rep.first_error_code, last_rep.first_error_msg);
  } else {
    add_diag(
      out_diagnostics, D::STAGE_PLANNING, waypoints.front().loc, "ERR_PVA_INFEASIBLE",
      "Direct-transcription Cartesian MovePVA found no feasible trajectory satisfying "
      "scaled Cartesian, joint, and singularity limits");
  }
  return false;
}

}  // namespace lite6_studio_planner
