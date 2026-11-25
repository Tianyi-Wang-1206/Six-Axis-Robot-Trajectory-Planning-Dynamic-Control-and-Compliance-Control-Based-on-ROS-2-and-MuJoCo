#include "lite6_studio_planner/curve_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>

namespace lite6_studio_planner
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kCharLength = 0.25;  // m/rad characteristic length for orientation arc length
// Corner blends near 180 deg reversals are forbidden because a smooth reversal requires a
// physical rest stop (cos(165 deg) = -0.9659258262890683).
constexpr double kMaxBlendReversalAngleDeg = 165.0;
constexpr double kMinBlendCosAngle = -0.9659258262890683;

Eigen::Matrix3d so3_hat(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(),
       v.z(), 0.0, -v.x(),
       -v.y(), v.x(), 0.0;
  return m;
}

Eigen::Vector3d so3_log(const Eigen::Matrix3d & R)
{
  const Eigen::AngleAxisd aa(R);
  const double ang = aa.angle();
  if (!std::isfinite(ang) || ang < 1e-12) {
    return Eigen::Vector3d::Zero();
  }
  return ang * aa.axis();
}

Eigen::Matrix3d so3_exp(const Eigen::Vector3d & w)
{
  const double ang = w.norm();
  if (ang < 1e-12) {
    const Eigen::Matrix3d K = so3_hat(w);
    return Eigen::Matrix3d::Identity() + K + 0.5 * (K * K);
  }
  return Eigen::AngleAxisd(ang, w / ang).toRotationMatrix();
}

// Left Jacobian J_l(phi) of SO(3) and its directional derivative d/ds J_l(phi) along dphi.
void so3_Jl_and_dJl(
  const Eigen::Vector3d & phi, const Eigen::Vector3d & dphi,
  Eigen::Matrix3d & Jl, Eigen::Matrix3d & dJl)
{
  const double th = phi.norm();
  const Eigen::Matrix3d P = so3_hat(phi);
  const Eigen::Matrix3d dP = so3_hat(dphi);
  const Eigen::Matrix3d P2 = P * P;
  const Eigen::Matrix3d dP2 = dP * P + P * dP;
  if (th < 1e-5) {
    const double t2 = th * th;
    const double c1 = 0.5 - t2 / 24.0 + (t2 * t2) / 720.0;
    const double c2 = (1.0 / 6.0) - t2 / 120.0 + (t2 * t2) / 5040.0;
    const double dc1_dt2 = -1.0 / 24.0 + t2 / 360.0;
    const double dc2_dt2 = -1.0 / 120.0 + t2 / 2520.0;
    const double dt2 = 2.0 * phi.dot(dphi);
    Jl = Eigen::Matrix3d::Identity() + c1 * P + c2 * P2;
    dJl = (dc1_dt2 * dt2) * P + c1 * dP + (dc2_dt2 * dt2) * P2 + c2 * dP2;
    return;
  }
  const double t2 = th * th;
  const double t3 = t2 * th;
  const double t4 = t2 * t2;
  const double s = std::sin(th);
  const double c = std::cos(th);
  const double c1 = (1.0 - c) / t2;
  const double c2 = (th - s) / t3;
  const double dth = phi.dot(dphi) / th;
  const double dc1 = (th * s - 2.0 * (1.0 - c)) / t3 * dth;
  const double dc2 = (th * (1.0 - c) - 3.0 * (th - s)) / t4 * dth;
  Jl = Eigen::Matrix3d::Identity() + c1 * P + c2 * P2;
  dJl = dc1 * P + c1 * dP + dc2 * P2 + c2 * dP2;
}

// Inverse left Jacobian J_l^{-1}(phi) of SO(3) and its derivative along dphi.
// Valid for ||phi|| < pi.
void so3_Jl_inv_and_dJl_inv(
  const Eigen::Vector3d & phi, const Eigen::Vector3d & dphi,
  Eigen::Matrix3d & Jli, Eigen::Matrix3d & dJli)
{
  const double th = phi.norm();
  const Eigen::Matrix3d P = so3_hat(phi);
  const Eigen::Matrix3d dP = so3_hat(dphi);
  const Eigen::Matrix3d P2 = P * P;
  const Eigen::Matrix3d dP2 = dP * P + P * dP;
  if (th < 1e-5) {
    const double t2 = th * th;
    const double c2 = (1.0 / 12.0) + t2 / 720.0 + (t2 * t2) / 30240.0;
    const double dc2_dt2 = 1.0 / 720.0 + t2 / 15120.0;
    const double dt2 = 2.0 * phi.dot(dphi);
    Jli = Eigen::Matrix3d::Identity() - 0.5 * P + c2 * P2;
    dJli = -0.5 * dP + (dc2_dt2 * dt2) * P2 + c2 * dP2;
    return;
  }
  const double t2 = th * th;
  const double t4 = t2 * t2;
  const double half = 0.5 * th;
  const double cot_h = std::cos(half) / std::sin(half);
  const double c2 = (1.0 - half * cot_h) / t2;
  const double dhcot = 0.5 * (cot_h - half * (1.0 + cot_h * cot_h));
  const double dth = phi.dot(dphi) / th;
  const double dc2 = (-dhcot * t2 - (1.0 - half * cot_h) * 2.0 * th) / t4 * dth;
  Jli = Eigen::Matrix3d::Identity() - 0.5 * P + c2 * P2;
  dJli = -0.5 * dP + dc2 * P2 + c2 * dP2;
}

// Inverse right Jacobian J_r^{-1}(phi) = J_l^{-1}(-phi) and its derivative along dphi.
void so3_Jr_inv_and_dJr_inv(
  const Eigen::Vector3d & phi, const Eigen::Vector3d & dphi,
  Eigen::Matrix3d & Jri, Eigen::Matrix3d & dJri)
{
  so3_Jl_inv_and_dJl_inv(-phi, -dphi, Jri, dJri);
}

// Evaluates a degree-7 Bernstein/Bezier curve and its 1st..3rd derivatives with respect to
// parameter s = tau * L (inv_L = 1 / L).
template <typename Vec>
void eval_bezier7(
  const std::array<Vec, 8> & P, double tau, double inv_L,
  Vec & q, Vec & q1, Vec & q2, Vec & q3)
{
  const double t = std::clamp(tau, 0.0, 1.0);
  const double u = 1.0 - t;
  const double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t, t6 = t5 * t, t7 = t6 * t;
  const double u2 = u * u, u3 = u2 * u, u4 = u3 * u, u5 = u4 * u, u6 = u5 * u, u7 = u6 * u;

  // Value: sum_{k=0}^7 C(7,k) u^(7-k) t^k P_k
  q = (u7) * P[0] +
      (7.0 * u6 * t) * P[1] +
      (21.0 * u5 * t2) * P[2] +
      (35.0 * u4 * t3) * P[3] +
      (35.0 * u3 * t4) * P[4] +
      (21.0 * u2 * t5) * P[5] +
      (7.0 * u * t6) * P[6] +
      (t7) * P[7];

  // Forward differences for exact Bernstein derivatives.
  std::array<Vec, 7> d1;
  for (size_t k = 0; k < 7; ++k) {
    d1[k] = P[k + 1] - P[k];
  }
  std::array<Vec, 6> d2;
  for (size_t k = 0; k < 6; ++k) {
    d2[k] = d1[k + 1] - d1[k];
  }
  std::array<Vec, 5> d3;
  for (size_t k = 0; k < 5; ++k) {
    d3[k] = d2[k + 1] - d2[k];
  }

  const double inv_L2 = inv_L * inv_L;
  const double inv_L3 = inv_L2 * inv_L;

  q1 = (7.0 * inv_L) * (
    (u6) * d1[0] +
    (6.0 * u5 * t) * d1[1] +
    (15.0 * u4 * t2) * d1[2] +
    (20.0 * u3 * t3) * d1[3] +
    (15.0 * u2 * t4) * d1[4] +
    (6.0 * u * t5) * d1[5] +
    (t6) * d1[6]);

  q2 = (42.0 * inv_L2) * (
    (u5) * d2[0] +
    (5.0 * u4 * t) * d2[1] +
    (10.0 * u3 * t2) * d2[2] +
    (10.0 * u2 * t3) * d2[3] +
    (5.0 * u * t4) * d2[4] +
    (t5) * d2[5]);

  q3 = (210.0 * inv_L3) * (
    (u4) * d3[0] +
    (4.0 * u3 * t) * d3[1] +
    (6.0 * u2 * t2) * d3[2] +
    (4.0 * u * t3) * d3[3] +
    (t4) * d3[4]);
}

template <typename Coeffs, typename Out>
void eval_septic(
  const Coeffs & c, double tau, double inv_h, Out & q, Out & q1, Out & q2, Out & q3)
{
  const double t2 = tau * tau;
  const double t3 = t2 * tau;
  const double t4 = t3 * tau;
  const double t5 = t4 * tau;
  const double t6 = t5 * tau;
  const double t7 = t6 * tau;
  const double inv_h2 = inv_h * inv_h;
  const double inv_h3 = inv_h2 * inv_h;
  q = c[0] + c[1] * tau + c[2] * t2 + c[3] * t3 + c[4] * t4 + c[5] * t5 + c[6] * t6 + c[7] * t7;
  q1 = (c[1] + 2.0 * c[2] * tau + 3.0 * c[3] * t2 + 4.0 * c[4] * t3 +
    5.0 * c[5] * t4 + 6.0 * c[6] * t5 + 7.0 * c[7] * t6) * inv_h;
  q2 = (2.0 * c[2] + 6.0 * c[3] * tau + 12.0 * c[4] * t2 +
    20.0 * c[5] * t3 + 30.0 * c[6] * t4 + 42.0 * c[7] * t5) * inv_h2;
  q3 = (6.0 * c[3] + 24.0 * c[4] * tau + 60.0 * c[5] * t2 +
    120.0 * c[6] * t3 + 210.0 * c[7] * t4) * inv_h3;
}

template <typename Vec>
std::array<Vec, 8> make_septic_hermite(
  const Vec & p0, const Vec & v0, const Vec & a0, const Vec & j0,
  const Vec & p1, const Vec & v1, const Vec & a1, const Vec & j1,
  double h)
{
  const double h2 = h * h;
  const double h3 = h2 * h;
  std::array<Vec, 8> c{};
  c[0] = p0;
  c[1] = v0 * h;
  c[2] = 0.5 * a0 * h2;
  c[3] = (1.0 / 6.0) * j0 * h3;

  const Vec d0 = p1 - (c[0] + c[1] + c[2] + c[3]);
  const Vec d1 = v1 * h - (c[1] + 2.0 * c[2] + 3.0 * c[3]);
  const Vec d2 = a1 * h2 - (2.0 * c[2] + 6.0 * c[3]);
  const Vec d3 = j1 * h3 - 6.0 * c[3];

  c[4] = 35.0 * d0 - 15.0 * d1 + 2.5 * d2 - (1.0 / 6.0) * d3;
  c[5] = -84.0 * d0 + 39.0 * d1 - 7.0 * d2 + 0.5 * d3;
  c[6] = 70.0 * d0 - 34.0 * d1 + 6.5 * d2 - 0.5 * d3;
  c[7] = -20.0 * d0 + 10.0 * d1 - 2.0 * d2 + (1.0 / 6.0) * d3;
  return c;
}

class LinearPiece final : public CartesianCurvePiece
{
public:
  LinearPiece(
    const Eigen::Matrix4d & T0,
    const Eigen::Matrix4d & T1,
    const MotionOptions & opts,
    const SourceLocation & loc)
  : p0_(T0.topRightCorner<3, 1>()),
    p1_(T1.topRightCorner<3, 1>()),
    R0_(T0.topLeftCorner<3, 3>()),
    opts_(opts),
    loc_(loc)
  {
    rotvec_local_ = so3_log(R0_.transpose() * T1.topLeftCorner<3, 3>());
    const double trans_len = (p1_ - p0_).norm();
    const double rot_len = rotvec_local_.norm();
    full_length_ = std::max({1e-6, trans_len, kCharLength * rot_len});
    s_start_ = 0.0;
    s_end_ = full_length_;
  }

  void set_trim(double s_start, double s_end)
  {
    s_start_ = std::clamp(s_start, 0.0, full_length_);
    s_end_ = std::clamp(s_end, s_start_ + 1e-6, full_length_);
  }

  double full_length() const { return full_length_; }
  double length() const override { return s_end_ - s_start_; }
  SourceLocation location() const override { return loc_; }
  MotionOptions options() const override { return opts_; }

  CartesianSample eval_untrimmed(double s) const
  {
    const double tau = std::clamp(s / full_length_, 0.0, 1.0);
    const double invL = 1.0 / full_length_;
    CartesianSample out;
    out.pose = Eigen::Matrix4d::Identity();
    out.pose.topRightCorner<3, 1>() = p0_ + tau * (p1_ - p0_);
    out.pose.topLeftCorner<3, 3>() = R0_ * so3_exp(tau * rotvec_local_);
    out.p_s = (p1_ - p0_) * invL;
    out.p_ss.setZero();
    out.p_sss.setZero();
    out.w_s = (R0_ * rotvec_local_) * invL;
    out.w_ss.setZero();
    out.vel_scale = opts_.vel_scale;
    out.acc_scale = opts_.acc_scale;
    out.jerk_scale = opts_.jerk_scale;
    out.loc = loc_;
    return out;
  }

  CartesianSample eval(double s_local) const override
  {
    return eval_untrimmed(s_start_ + std::clamp(s_local, 0.0, length()));
  }

private:
  Eigen::Vector3d p0_, p1_;
  Eigen::Matrix3d R0_;
  Eigen::Vector3d rotvec_local_;
  double full_length_ = 1e-6;
  double s_start_ = 0.0;
  double s_end_ = 1e-6;
  MotionOptions opts_;
  SourceLocation loc_;
};

class CircularPiece final : public CartesianCurvePiece
{
public:
  // If explicit_rotvec_local is provided, orientation rotates continuously along that
  // body-frame rotation vector without SO(3) log wrapping (preserving |angle| >= pi).
  CircularPiece(
    const Eigen::Vector3d & center,
    const Eigen::Vector3d & ex,
    const Eigen::Vector3d & ey,
    const Eigen::Vector3d & ez,
    double radius,
    double axial_pitch,
    double sweep_angle,
    const Eigen::Matrix3d & R0,
    const Eigen::Matrix3d & R1,
    const MotionOptions & opts,
    const SourceLocation & loc,
    const std::optional<Eigen::Vector3d> & explicit_rotvec_local = std::nullopt)
  : center_(center),
    ex_(ex),
    ey_(ey),
    ez_(ez),
    radius_(radius),
    axial_pitch_(axial_pitch),
    angle_(sweep_angle),
    R0_(R0),
    opts_(opts),
    loc_(loc)
  {
    rotvec_local_ = explicit_rotvec_local.has_value()
      ? *explicit_rotvec_local
      : so3_log(R0_.transpose() * R1);
    const double arc_len =
      std::hypot(radius_ * std::abs(angle_), axial_pitch_);
    const double rot_len = rotvec_local_.norm();
    full_length_ = std::max({1e-6, arc_len, kCharLength * rot_len});
    s_start_ = 0.0;
    s_end_ = full_length_;
  }

  void set_trim(double s_start, double s_end)
  {
    s_start_ = std::clamp(s_start, 0.0, full_length_);
    s_end_ = std::clamp(s_end, s_start_ + 1e-6, full_length_);
  }

  double full_length() const { return full_length_; }
  double length() const override { return s_end_ - s_start_; }
  SourceLocation location() const override { return loc_; }
  MotionOptions options() const override { return opts_; }

  CartesianSample eval_untrimmed(double s) const
  {
    const double tau = std::clamp(s / full_length_, 0.0, 1.0);
    const double invL = 1.0 / full_length_;
    const double th = angle_ * tau;
    const double dth = angle_ * invL;
    const double c = std::cos(th);
    const double sn = std::sin(th);

    CartesianSample out;
    out.pose = Eigen::Matrix4d::Identity();
    out.pose.topRightCorner<3, 1>() =
      center_ + radius_ * (c * ex_ + sn * ey_) + (tau * axial_pitch_) * ez_;
    out.pose.topLeftCorner<3, 3>() = R0_ * so3_exp(tau * rotvec_local_);
    out.p_s = radius_ * dth * (-sn * ex_ + c * ey_) + (axial_pitch_ * invL) * ez_;
    out.p_ss = -radius_ * (dth * dth) * (c * ex_ + sn * ey_);
    out.p_sss = radius_ * (dth * dth * dth) * (sn * ex_ - c * ey_);
    out.w_s = (R0_ * rotvec_local_) * invL;
    out.w_ss.setZero();
    out.vel_scale = opts_.vel_scale;
    out.acc_scale = opts_.acc_scale;
    out.jerk_scale = opts_.jerk_scale;
    out.loc = loc_;
    return out;
  }

  CartesianSample eval(double s_local) const override
  {
    return eval_untrimmed(s_start_ + std::clamp(s_local, 0.0, length()));
  }

private:
  Eigen::Vector3d center_, ex_, ey_, ez_;
  double radius_ = 0.0;
  double axial_pitch_ = 0.0;
  double angle_ = 0.0;
  Eigen::Matrix3d R0_;
  Eigen::Vector3d rotvec_local_;
  double full_length_ = 1e-6;
  double s_start_ = 0.0;
  double s_end_ = 1e-6;
  MotionOptions opts_;
  SourceLocation loc_;
};

// Septic smoothstep: 0 -> 1 with vanishing 1st..3rd derivatives at both ends.
void septic_smoothstep_derivs(double tau, double & a, double & da, double & dda)
{
  const double t = std::clamp(tau, 0.0, 1.0);
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  a = t4 * (35.0 + t * (-84.0 + t * (70.0 - 20.0 * t)));
  da = t3 * (140.0 + t * (-420.0 + t * (420.0 - 140.0 * t)));
  dda = t2 * (420.0 + t * (-1680.0 + t * (2100.0 - 840.0 * t)));
}

// C^3 corner blend between the end of one piece (s0) and the start of the next (s1),
// trimmed symmetrically by trim_len on each side (blend length L = 2 * trim_len).
// Position: degree-7 Bernstein/Bezier curve matching p, p_s, p_ss, p_sss at both ends.
//   For linear-to-linear corners (p_ss = p_sss = 0), all 8 control points lie strictly on
//   [a, W] and [W, b] inside the corner triangle [a, W, b] (convex-hull property) and
//   collinear corners reproduce exact unit-speed straight lines.
// Orientation: R(s) = R_in(s) * Exp(alpha(s) * Log(R_in(s)^T R_out(s))), with exact
//   closed-form analytic SO(3) angular velocity w_s and angular acceleration w_ss.
class SepticBlendPiece final : public CartesianCurvePiece
{
public:
  SepticBlendPiece(const CartesianSample & s0, const CartesianSample & s1, double trim_len)
  : Ra_(s0.pose.topLeftCorner<3, 3>()), Rb_(s1.pose.topLeftCorner<3, 3>()), loc_(s0.loc)
  {
    opts_.vel_scale = std::min(s0.vel_scale, s1.vel_scale);
    opts_.acc_scale = std::min(s0.acc_scale, s1.acc_scale);
    opts_.jerk_scale = std::min(s0.jerk_scale, s1.jerk_scale);

    const Eigen::Vector3d p0 = s0.pose.topRightCorner<3, 1>();
    const Eigen::Vector3d p1 = s1.pose.topRightCorner<3, 1>();
    L_ = std::max(1e-5, 2.0 * trim_len);
    const double L = L_;
    const double L2 = L * L;
    const double L3 = L2 * L;

    // Degree-7 Bernstein/Bezier control points matching (p, p_s, p_ss, p_sss) at both ends.
    // When p_ss = p_sss = 0 and L = 2*trim_len, P[0..3] lie on [p0, W] and P[4..7] on [W, p1].
    pos_bezier_[0] = p0;
    pos_bezier_[1] = p0 + (L / 7.0) * s0.p_s;
    pos_bezier_[2] = p0 + (2.0 * L / 7.0) * s0.p_s + (L2 / 42.0) * s0.p_ss;
    pos_bezier_[3] = p0 + (3.0 * L / 7.0) * s0.p_s + (L2 / 14.0) * s0.p_ss +
      (L3 / 210.0) * s0.p_sss;
    pos_bezier_[7] = p1;
    pos_bezier_[6] = p1 - (L / 7.0) * s1.p_s;
    pos_bezier_[5] = p1 - (2.0 * L / 7.0) * s1.p_s + (L2 / 42.0) * s1.p_ss;
    pos_bezier_[4] = p1 - (3.0 * L / 7.0) * s1.p_s + (L2 / 14.0) * s1.p_ss -
      (L3 / 210.0) * s1.p_sss;

    // Body-frame constant angular rates of the neighbouring pieces (per unit s).
    wa_body_ = Ra_.transpose() * s0.w_s;
    wb_body_ = Rb_.transpose() * s1.w_s;

    double max_delta = 0.0;
    for (int k = 0; k <= 32; ++k) {
      max_delta = std::max(max_delta, delta(L_ * k / 32.0).norm());
    }
    valid_ = std::isfinite(max_delta) && max_delta < 2.6;
  }

  bool valid() const { return valid_; }
  double length() const override { return L_; }
  SourceLocation location() const override { return loc_; }
  MotionOptions options() const override { return opts_; }

  CartesianSample eval(double s_local) const override
  {
    const double sigma = std::clamp(s_local, 0.0, L_);
    const double tau = sigma / L_;
    const double invL = 1.0 / L_;

    CartesianSample out;
    out.pose = Eigen::Matrix4d::Identity();
    Eigen::Vector3d p, ps, pss, psss;
    eval_bezier7(pos_bezier_, tau, invL, p, ps, pss, psss);

    // Exact analytic SO(3) pose, angular velocity, and angular acceleration.
    const Eigen::Matrix3d R1 = Ra_ * so3_exp(sigma * wa_body_);
    const Eigen::Matrix3d R2 = Rb_ * so3_exp((sigma - L_) * wb_body_);
    const Eigen::Matrix3d E = R1.transpose() * R2;
    const Eigen::Vector3d d = so3_log(E);
    const Eigen::Vector3d Et_wa = E.transpose() * wa_body_;
    const Eigen::Vector3d wE = wb_body_ - Et_wa;

    Eigen::Matrix3d Jri, dJri_unused, Jri_unused, dJri;
    so3_Jr_inv_and_dJr_inv(d, Eigen::Vector3d::Zero(), Jri, dJri_unused);
    const Eigen::Vector3d dd = Jri * wE;
    so3_Jr_inv_and_dJr_inv(d, dd, Jri_unused, dJri);
    const Eigen::Vector3d dwE = wE.cross(Et_wa);
    const Eigen::Vector3d ddd = dJri * wE + Jri * dwE;

    double a = 0.0, da_tau = 0.0, dda_tau = 0.0;
    septic_smoothstep_derivs(tau, a, da_tau, dda_tau);
    const double da = da_tau * invL;
    const double dda = dda_tau * invL * invL;

    const Eigen::Vector3d phi = a * d;
    const Eigen::Vector3d dphi = da * d + a * dd;
    const Eigen::Vector3d ddphi = dda * d + 2.0 * da * dd + a * ddd;

    Eigen::Matrix3d Jl, dJl;
    so3_Jl_and_dJl(phi, dphi, Jl, dJl);
    const Eigen::Vector3d Jl_dphi = Jl * dphi;

    out.pose.topRightCorner<3, 1>() = p;
    out.pose.topLeftCorner<3, 3>() = R1 * so3_exp(phi);
    out.p_s = ps;
    out.p_ss = pss;
    out.p_sss = psss;
    out.w_s = R1 * (wa_body_ + Jl_dphi);
    out.w_ss = R1 * (wa_body_.cross(Jl_dphi) + dJl * dphi + Jl * ddphi);
    out.vel_scale = opts_.vel_scale;
    out.acc_scale = opts_.acc_scale;
    out.jerk_scale = opts_.jerk_scale;
    out.loc = loc_;
    return out;
  }

private:
  Eigen::Vector3d delta(double sigma) const
  {
    const Eigen::Matrix3d r_in = Ra_ * so3_exp(sigma * wa_body_);
    const Eigen::Matrix3d r_out = Rb_ * so3_exp((sigma - L_) * wb_body_);
    return so3_log(r_in.transpose() * r_out);
  }

  Eigen::Matrix3d Ra_, Rb_;
  Eigen::Vector3d wa_body_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d wb_body_ = Eigen::Vector3d::Zero();
  std::array<Eigen::Vector3d, 8> pos_bezier_{};
  double L_ = 1e-3;
  bool valid_ = true;
  MotionOptions opts_;
  SourceLocation loc_;
};

// Checks whether the corner between two consecutive Cartesian samples reverses direction
// beyond kMaxBlendReversalAngleDeg (near 180 deg reversal).
bool is_near_cartesian_reversal(
  const CartesianSample & end_in, const CartesianSample & start_out, double & out_angle_deg)
{
  out_angle_deg = 0.0;
  const double np_in = end_in.p_s.norm();
  const double np_out = start_out.p_s.norm();
  if (np_in > 1e-6 && np_out > 1e-6) {
    const double cos_a = std::clamp(
      (end_in.p_s / np_in).dot(start_out.p_s / np_out), -1.0, 1.0);
    out_angle_deg = std::acos(cos_a) * (180.0 / kPi);
    if (cos_a < kMinBlendCosAngle) {
      return true;
    }
  } else {
    const double nw_in = end_in.w_s.norm();
    const double nw_out = start_out.w_s.norm();
    if (nw_in > 1e-6 && nw_out > 1e-6) {
      const double cos_a = std::clamp(
        (end_in.w_s / nw_in).dot(start_out.w_s / nw_out), -1.0, 1.0);
      out_angle_deg = std::acos(cos_a) * (180.0 / kPi);
      if (cos_a < kMinBlendCosAngle) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace

bool CompositeCartesianPath::build(
  const Eigen::Matrix4d & initial_pose,
  const std::vector<MotionStatement> & statements,
  SourceLocation & err_loc,
  std::string & err_code,
  std::string & err_msg,
  std::vector<BlendWarning> * out_warnings)
{
  pieces_.clear();
  s_cum_.clear();
  total_length_ = 0.0;
  if (out_warnings != nullptr) {
    out_warnings->clear();
  }

  if (statements.empty()) {
    err_code = "ERR_EMPTY_PATH";
    err_msg = "Cannot build Cartesian path from empty statement list";
    return false;
  }

  struct RawPrim
  {
    std::shared_ptr<LinearPiece> lin;
    std::shared_ptr<CircularPiece> circ;
    double full_len = 0.0;
    double trim_start = 0.0;
    double trim_end = 0.0;
    double blend_after = 0.0;
    SourceLocation loc;

    CartesianSample eval_untrimmed(double s) const
    {
      return lin ? lin->eval_untrimmed(s) : circ->eval_untrimmed(s);
    }
    void apply_trim()
    {
      if (lin) {
        lin->set_trim(trim_start, trim_end);
      } else if (circ) {
        circ->set_trim(trim_start, trim_end);
      }
    }
    std::shared_ptr<CartesianCurvePiece> as_piece() const
    {
      return lin ? std::static_pointer_cast<CartesianCurvePiece>(lin)
                 : std::static_pointer_cast<CartesianCurvePiece>(circ);
    }
  };

  std::vector<RawPrim> raw_prims;
  Eigen::Matrix4d curr_pose = initial_pose;

  for (size_t idx = 0; idx < statements.size(); ++idx) {
    const auto & st = statements[idx];
    const double next_blend = (idx + 1 < statements.size()) ? st.options.blend_radius : 0.0;

    if (st.type == MotionCommandType::MOVE_L) {
      const double trans_len =
        (st.pose_target.topRightCorner<3, 1>() - curr_pose.topRightCorner<3, 1>()).norm();
      const double rot_len = so3_log(
        curr_pose.topLeftCorner<3, 3>().transpose() * st.pose_target.topLeftCorner<3, 3>()).norm();
      // Skip redundant zero-length MoveL statements inside multi-statement blended groups so
      // they cannot silently kill blending or inject a zero-tangent discontinuity.
      if (statements.size() > 1 && trans_len < 1e-7 && rot_len < 1e-7) {
        if (!raw_prims.empty()) {
          const double prev_req = raw_prims.back().blend_after;
          const double eff = (idx + 1 < statements.size()) ?
            std::min(prev_req, next_blend) :
            0.0;
          if (out_warnings != nullptr && prev_req > 1e-5 && eff < prev_req - 1e-5) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(4)
                << "Cartesian blend radius " << (eff <= 1e-5 ? "ignored" : "clamped")
                << " because following MoveL is zero-length (requested=" << prev_req
                << " m, effective=" << eff << " m)";
            out_warnings->push_back({
              raw_prims.back().loc,
              eff <= 1e-5 ? "WARN_BLEND_IGNORED" : "WARN_BLEND_CLAMPED",
              oss.str()});
          }
          raw_prims.back().blend_after = eff;
        }
        if (out_warnings != nullptr && next_blend > 1e-5) {
          const double eff_st = raw_prims.empty() ? 0.0 : raw_prims.back().blend_after;
          if (eff_st < next_blend - 1e-5) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(4)
                << "Cartesian blend radius " << (eff_st <= 1e-5 ? "ignored" : "clamped")
                << " on skipped zero-length MoveL (requested=" << next_blend
                << " m, effective=" << eff_st << " m)";
            out_warnings->push_back({
              st.loc,
              eff_st <= 1e-5 ? "WARN_BLEND_IGNORED" : "WARN_BLEND_CLAMPED",
              oss.str()});
          }
        }
        curr_pose = st.pose_target;
        continue;
      }
      RawPrim rp;
      rp.blend_after = next_blend;
      rp.loc = st.loc;
      rp.lin = std::make_shared<LinearPiece>(curr_pose, st.pose_target, st.options, st.loc);
      rp.full_len = rp.lin->full_length();
      rp.trim_start = 0.0;
      rp.trim_end = rp.full_len;
      curr_pose = st.pose_target;
      raw_prims.push_back(rp);
    } else if (st.type == MotionCommandType::MOVE_C_THREE_POINT) {
      RawPrim rp;
      rp.blend_after = next_blend;
      rp.loc = st.loc;
      const Eigen::Vector3d p0 = curr_pose.topRightCorner<3, 1>();
      const Eigen::Vector3d p_via = st.pose_via.topRightCorner<3, 1>();
      const Eigen::Vector3d p1 = st.pose_target.topRightCorner<3, 1>();
      const Eigen::Vector3d u = p_via - p0;
      const Eigen::Vector3d v = p1 - p0;
      const double max_sep = std::max({u.norm(), v.norm(), (p1 - p_via).norm()});
      if (max_sep < 1e-5) {
        err_loc = st.loc;
        err_code = "ERR_DEGENERATE_CIRCLE";
        err_msg = "MoveC 3-point arc points coincide (separation < 1e-5 m)";
        return false;
      }
      Eigen::Vector3d normal = u.cross(v);
      if (normal.norm() < 1e-5 * max_sep * max_sep) {
        err_loc = st.loc;
        err_code = "ERR_COLLINEAR_CIRCLE";
        err_msg = "MoveC start, via, and target points are collinear; cannot define a unique arc";
        return false;
      }
      normal.normalize();
      const Eigen::Vector3d e1 = u.normalized();
      const Eigen::Vector3d e2 = normal.cross(e1);
      const double x = v.dot(e1);
      const double y = v.dot(e2);
      if (std::abs(y) < 1e-8 * max_sep) {
        err_loc = st.loc;
        err_code = "ERR_COLLINEAR_CIRCLE";
        err_msg = "MoveC degenerate transverse separation";
        return false;
      }
      const double cx = 0.5 * u.norm();
      const double cy = (x * x + y * y - 2.0 * x * cx) / (2.0 * y);
      const Eigen::Vector3d center = p0 + cx * e1 + cy * e2;
      const double radius = (p0 - center).norm();
      if (!std::isfinite(radius) || radius < 1e-5 || radius > 50.0) {
        err_loc = st.loc;
        err_code = "ERR_CIRCLE_RADIUS";
        err_msg = "MoveC computed radius=" + std::to_string(radius) + " m is out of valid bounds";
        return false;
      }
      const Eigen::Vector3d ex = (p0 - center) / radius;
      const Eigen::Vector3d ey = normal.cross(ex);
      auto angle_of = [&](const Eigen::Vector3d & p) {
        double a = std::atan2((p - center).dot(ey), (p - center).dot(ex));
        if (a < 0.0) {
          a += 2.0 * kPi;
        }
        return a;
      };
      const double th_via = angle_of(p_via);
      double th_end = angle_of(p1);
      if (th_end + 1e-9 < th_via) {
        th_end += 2.0 * kPi;
      }
      if (th_via < 1e-5 || th_end - th_via < 1e-5 || th_end > 2.0 * kPi + 1e-5) {
        err_loc = st.loc;
        err_code = "ERR_CIRCLE_ARC_ORDER";
        err_msg = "MoveC auxiliary 'via' point does not lie strictly between start and target on arc";
        return false;
      }
      rp.circ = std::make_shared<CircularPiece>(
        center, ex, ey, normal, radius, 0.0, th_end,
        curr_pose.topLeftCorner<3, 3>(), st.pose_target.topLeftCorner<3, 3>(),
        st.options, st.loc);
      rp.full_len = rp.circ->full_length();
      rp.trim_start = 0.0;
      rp.trim_end = rp.full_len;
      curr_pose = st.pose_target;
      raw_prims.push_back(rp);
    } else if (st.type == MotionCommandType::MOVE_C_CENTER_ANGLE) {
      RawPrim rp;
      rp.blend_after = next_blend;
      rp.loc = st.loc;
      const Eigen::Vector3d p0 = curr_pose.topRightCorner<3, 1>();
      const Eigen::Vector3d ez = st.circle_axis.normalized();
      const Eigen::Vector3d rel = p0 - st.circle_center;
      const double axial_offset = rel.dot(ez);
      const Eigen::Vector3d radial_vec = rel - axial_offset * ez;
      const double radius = radial_vec.norm();
      if (radius < 1e-5) {
        err_loc = st.loc;
        err_code = "ERR_CIRCLE_CENTER_COINCIDENT";
        err_msg = "Start position lies on the rotation axis of MoveC (radius=" +
                  std::to_string(radius) + " m < 1e-5 m)";
        return false;
      }
      const Eigen::Vector3d ex = radial_vec / radius;
      const Eigen::Vector3d ey = ez.cross(ex);
      const Eigen::Vector3d eff_center = st.circle_center + axial_offset * ez;
      const Eigen::Matrix3d R0 = curr_pose.topLeftCorner<3, 3>();

      // When target_ori is omitted, pass the exact unwrapped local rotation vector
      // R0^T * (angle * ez) directly so |angle| >= pi rotates continuously without wrapping.
      if (st.has_explicit_target_ori) {
        rp.circ = std::make_shared<CircularPiece>(
          eff_center, ex, ey, ez, radius, 0.0, st.circle_angle_rad,
          R0, st.circle_target_rot, st.options, st.loc);
      } else {
        const Eigen::Vector3d rotvec_local = R0.transpose() * (st.circle_angle_rad * ez);
        const Eigen::Matrix3d R1 =
          Eigen::AngleAxisd(st.circle_angle_rad, ez).toRotationMatrix() * R0;
        rp.circ = std::make_shared<CircularPiece>(
          eff_center, ex, ey, ez, radius, 0.0, st.circle_angle_rad,
          R0, R1, st.options, st.loc, rotvec_local);
      }
      rp.full_len = rp.circ->full_length();
      rp.trim_start = 0.0;
      rp.trim_end = rp.full_len;
      curr_pose = rp.circ->eval_untrimmed(rp.full_len).pose;
      raw_prims.push_back(rp);
    } else {
      err_loc = st.loc;
      err_code = "ERR_UNSUPPORTED_CART_PRIMITIVE";
      err_msg = "Unsupported primitive in Cartesian composite path";
      return false;
    }
  }

  // If every statement in the group was zero-length, keep one zero-length piece so the path
  // compiles cleanly as a stationary no-op.
  if (raw_prims.empty()) {
    RawPrim rp;
    rp.loc = statements.back().loc;
    rp.lin = std::make_shared<LinearPiece>(
      initial_pose, initial_pose, statements.back().options, statements.back().loc);
    rp.full_len = rp.lin->full_length();
    rp.trim_start = 0.0;
    rp.trim_end = rp.full_len;
    raw_prims.push_back(rp);
  }

  // Compute trim distances for C^3 Bernstein/Bezier corner blending between consecutive segments.
  std::vector<double> trim_len(raw_prims.size(), 0.0);
  for (size_t i = 0; i + 1 < raw_prims.size(); ++i) {
    const double r = raw_prims[i].blend_after;
    if (r > 1e-5) {
      // Forbid blending near 180 deg reversals where a clean rest stop is physically required.
      const CartesianSample corner_in = raw_prims[i].eval_untrimmed(raw_prims[i].full_len);
      const CartesianSample corner_out = raw_prims[i + 1].eval_untrimmed(0.0);
      double angle_deg = 0.0;
      if (is_near_cartesian_reversal(corner_in, corner_out, angle_deg)) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1)
            << "Cartesian corner reversal angle (" << angle_deg
            << " deg) exceeds the maximum smooth blend angle (" << kMaxBlendReversalAngleDeg
            << " deg); remove blend to insert a clean rest stop";
        err_loc = raw_prims[i + 1].loc;
        err_code = "ERR_BLEND_REVERSAL";
        err_msg = oss.str();
        return false;
      }
      trim_len[i] = std::min({r, 0.42 * raw_prims[i].full_len, 0.42 * raw_prims[i + 1].full_len});
      if (out_warnings != nullptr && trim_len[i] < r - 1e-5) {
        const double eff = (trim_len[i] > 1e-5) ? trim_len[i] : 0.0;
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4)
            << "Cartesian blend radius " << (eff <= 1e-5 ? "ignored" : "clamped")
            << " to 42% of adjacent segment length (requested=" << r
            << " m, effective=" << eff << " m; segment lengths="
            << raw_prims[i].full_len << " m, " << raw_prims[i + 1].full_len << " m)";
        out_warnings->push_back({
          raw_prims[i].loc,
          eff <= 1e-5 ? "WARN_BLEND_IGNORED" : "WARN_BLEND_CLAMPED",
          oss.str()});
      }
      raw_prims[i].trim_end -= trim_len[i];
      raw_prims[i + 1].trim_start += trim_len[i];
    }
  }

  for (size_t i = 0; i < raw_prims.size(); ++i) {
    raw_prims[i].apply_trim();
    pieces_.push_back(raw_prims[i].as_piece());
    if (i + 1 < raw_prims.size() && trim_len[i] > 1e-5) {
      const CartesianSample s_in = raw_prims[i].eval_untrimmed(raw_prims[i].trim_end);
      const CartesianSample s_out = raw_prims[i + 1].eval_untrimmed(raw_prims[i + 1].trim_start);
      auto blend = std::make_shared<SepticBlendPiece>(s_in, s_out, trim_len[i]);
      if (!blend->valid()) {
        err_loc = raw_prims[i + 1].loc;
        err_code = "ERR_BLEND_ORIENTATION";
        err_msg = "Corner blend orientation change is too large to blend smoothly";
        return false;
      }
      pieces_.push_back(blend);
    }
  }

  s_cum_.resize(pieces_.size() + 1, 0.0);
  for (size_t i = 0; i < pieces_.size(); ++i) {
    s_cum_[i + 1] = s_cum_[i] + pieces_[i]->length();
  }
  s_begin_ = 0.0;
  total_length_ = std::max(1e-6, s_cum_.back());
  return true;
}

std::vector<double> CompositeCartesianPath::piece_boundaries() const
{
  std::vector<double> out;
  for (size_t i = 1; i + 1 < s_cum_.size(); ++i) {
    const double u = (s_cum_[i] - s_begin_) / total_length_;
    if (u > 1e-9 && u < 1.0 - 1e-9) {
      out.push_back(u);
    }
  }
  return out;
}

CompositeCartesianPath CompositeCartesianPath::restricted(double u_start) const
{
  CompositeCartesianPath sub = *this;
  const double u0 = std::clamp(u_start, 0.0, 1.0);
  sub.s_begin_ = s_begin_ + u0 * total_length_;
  sub.total_length_ = std::max(1e-9, total_length_ * (1.0 - u0));
  return sub;
}

CartesianSample CompositeCartesianPath::eval_normalized(double u) const
{
  const double u_clamped = std::clamp(u, 0.0, 1.0);
  const double s = s_begin_ + u_clamped * total_length_;

  const auto it = std::upper_bound(s_cum_.begin() + 1, s_cum_.end() - 1, s);
  const size_t idx = static_cast<size_t>(std::distance(s_cum_.begin() + 1, it));
  const double s_local = std::clamp(s - s_cum_[idx], 0.0, pieces_[idx]->length());
  CartesianSample sample = pieces_[idx]->eval(s_local);

  // Chain rule from arc-length s in [0, S] to normalized parameter u in [0, 1]: ds/du = S.
  const double S = total_length_;
  const double S2 = S * S;
  const double S3 = S2 * S;
  sample.p_s *= S;
  sample.p_ss *= S2;
  sample.p_sss *= S3;
  sample.w_s *= S;
  sample.w_ss *= S2;
  return sample;
}

bool C3CartesianSpline::build(
  const std::vector<Eigen::Matrix4d> & poses,
  const std::vector<SourceLocation> & seg_locs,
  const MotionOptions & opts,
  std::string & err_msg)
{
  segments_.clear();
  opts_ = opts;
  total_length_ = 1e-6;
  if (poses.size() < 2 || seg_locs.size() + 1 != poses.size()) {
    err_msg = "Cartesian spline requires at least two poses and matching segment locations";
    return false;
  }

  const size_t n = poses.size() - 1;
  std::vector<Eigen::Vector3d> p(n + 1);
  std::vector<Eigen::Matrix3d> R(n + 1);
  for (size_t k = 0; k <= n; ++k) {
    p[k] = poses[k].topRightCorner<3, 1>();
    R[k] = poses[k].topLeftCorner<3, 3>();
  }

  // Relative rotations and pseudo-arc-length knot spacing.
  std::vector<Eigen::Vector3d> delta_rot(n);
  std::vector<double> d_seg(n, 0.0);
  double total_arc = 0.0;
  for (size_t k = 0; k < n; ++k) {
    delta_rot[k] = so3_log(R[k].transpose() * R[k + 1]);
    const double r_ang = delta_rot[k].norm();
    if (!std::isfinite(r_ang) || r_ang > 2.95) {
      err_msg = "Cartesian spline segment " + std::to_string(k + 1) +
        " orientation change is too close to 180 deg for unambiguous SO(3) interpolation";
      return false;
    }
    d_seg[k] = std::max((p[k + 1] - p[k]).norm(), kCharLength * r_ang);
    total_arc += d_seg[k];
  }

  std::vector<double> u(n + 1, 0.0);
  if (total_arc < 1e-12) {
    total_length_ = 1e-6;
    for (size_t k = 0; k <= n; ++k) {
      u[k] = static_cast<double>(k) / static_cast<double>(n);
    }
  } else {
    const double floor = 1e-6 * total_arc;
    double acc = 0.0;
    for (size_t k = 0; k < n; ++k) {
      acc += std::max(d_seg[k], floor);
      u[k + 1] = acc;
    }
    total_length_ = acc;
    for (auto & uk : u) {
      uk /= acc;
    }
    u.back() = 1.0;
  }

  // 1. Natural cubic spline solve for position (m1, m2) and harmonic m3.
  std::vector<Eigen::Vector3d> pm2(n + 1, Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector3d> wa_nodal(n + 1, Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector3d> w_secant(n, Eigen::Vector3d::Zero());
  for (size_t k = 0; k < n; ++k) {
    const double h = u[k + 1] - u[k];
    w_secant[k] = (R[k] * delta_rot[k]) / h;
  }

  if (n >= 2) {
    std::vector<double> lower(n + 1, 0.0);
    std::vector<double> diag(n + 1, 1.0);
    std::vector<double> upper(n + 1, 0.0);
    std::vector<Eigen::Vector3d> rhs_p(n + 1, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> rhs_w(n + 1, Eigen::Vector3d::Zero());

    for (size_t i = 1; i < n; ++i) {
      const double hm = u[i] - u[i - 1];
      const double hp = u[i + 1] - u[i];
      lower[i] = hm;
      diag[i] = 2.0 * (hm + hp);
      upper[i] = hp;
      rhs_p[i] = 6.0 * ((p[i + 1] - p[i]) / hp - (p[i] - p[i - 1]) / hm);
      rhs_w[i] = 6.0 * (w_secant[i] - w_secant[i - 1]);
    }
    for (size_t i = 2; i < n; ++i) {
      const double w = lower[i] / diag[i - 1];
      diag[i] -= w * upper[i - 1];
      rhs_p[i] -= w * rhs_p[i - 1];
      rhs_w[i] -= w * rhs_w[i - 1];
    }
    pm2[n - 1] = rhs_p[n - 1] / diag[n - 1];
    wa_nodal[n - 1] = rhs_w[n - 1] / diag[n - 1];
    for (size_t i = n - 1; i-- > 1;) {
      pm2[i] = (rhs_p[i] - upper[i] * pm2[i + 1]) / diag[i];
      wa_nodal[i] = (rhs_w[i] - upper[i] * wa_nodal[i + 1]) / diag[i];
    }
  }

  std::vector<Eigen::Vector3d> pm1(n + 1, Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector3d> w_nodal(n + 1, Eigen::Vector3d::Zero());
  for (size_t i = 0; i < n; ++i) {
    const double h = u[i + 1] - u[i];
    pm1[i] = (p[i + 1] - p[i]) / h - (h / 6.0) * (2.0 * pm2[i] + pm2[i + 1]);
    w_nodal[i] = w_secant[i] - (h / 6.0) * (2.0 * wa_nodal[i] + wa_nodal[i + 1]);
  }
  {
    const double h = u[n] - u[n - 1];
    pm1[n] = (p[n] - p[n - 1]) / h + (h / 6.0) * (pm2[n - 1] + 2.0 * pm2[n]);
    w_nodal[n] = w_secant[n - 1] + (h / 6.0) * (wa_nodal[n - 1] + 2.0 * wa_nodal[n]);
  }

  std::vector<Eigen::Vector3d> pm3(n + 1, Eigen::Vector3d::Zero());
  for (size_t i = 1; i < n; ++i) {
    const double hm = u[i] - u[i - 1];
    const double hp = u[i + 1] - u[i];
    const Eigen::Vector3d sl_m = (pm2[i] - pm2[i - 1]) / hm;
    const Eigen::Vector3d sl_p = (pm2[i + 1] - pm2[i]) / hp;
    pm3[i] = (hp * sl_m + hm * sl_p) / (hm + hp);
  }
  pm3[0] = (pm2[1] - pm2[0]) / (u[1] - u[0]);
  pm3[n] = (pm2[n] - pm2[n - 1]) / (u[n] - u[n - 1]);

  // 2. Build per-segment septic position and Lie-algebra SO(3) Hermite polynomials.
  segments_.reserve(n);
  for (size_t k = 0; k < n; ++k) {
    const double h = u[k + 1] - u[k];
    Segment seg;
    seg.u0 = u[k];
    seg.h = h;
    seg.R0 = R[k];
    seg.loc = seg_locs[k];
    seg.p_c = make_septic_hermite(
      p[k], pm1[k], pm2[k], pm3[k],
      p[k + 1], pm1[k + 1], pm2[k + 1], pm3[k + 1], h);

    // At tau = 0: theta = 0, J_l(0) = I, dJ_l(0) * dtheta(0) = 0.
    const Eigen::Vector3d th0 = Eigen::Vector3d::Zero();
    const Eigen::Vector3d dth0 = R[k].transpose() * w_nodal[k];
    const Eigen::Vector3d ddth0 = R[k].transpose() * wa_nodal[k];

    // At tau = 1: theta = delta_rot[k], w_w = R_k J_l(delta) dth1,
    // a_w = R_k (dJ_l(delta, dth1) dth1 + J_l(delta) ddth1).
    const Eigen::Vector3d th1 = delta_rot[k];
    Eigen::Matrix3d Jli, dJli_unused;
    so3_Jl_inv_and_dJl_inv(th1, Eigen::Vector3d::Zero(), Jli, dJli_unused);
    const Eigen::Vector3d dth1 = Jli * (R[k].transpose() * w_nodal[k + 1]);
    Eigen::Matrix3d Jl_unused, dJl;
    so3_Jl_and_dJl(th1, dth1, Jl_unused, dJl);
    const Eigen::Vector3d ddth1 = Jli * (R[k].transpose() * wa_nodal[k + 1] - dJl * dth1);

    seg.r_c = make_septic_hermite<Eigen::Vector3d>(
      th0, dth0, ddth0, Eigen::Vector3d::Zero(),
      th1, dth1, ddth1, Eigen::Vector3d::Zero(), h);
    segments_.push_back(seg);
  }
  return true;
}

std::vector<double> C3CartesianSpline::piece_boundaries() const
{
  std::vector<double> out;
  for (size_t i = 1; i < segments_.size(); ++i) {
    if (segments_[i].u0 > 1e-9 && segments_[i].u0 < 1.0 - 1e-9) {
      out.push_back(segments_[i].u0);
    }
  }
  return out;
}

CartesianSample C3CartesianSpline::eval_normalized(double u) const
{
  CartesianSample out;
  if (segments_.empty()) {
    return out;
  }
  const double uc = std::clamp(u, 0.0, 1.0);
  const auto it = std::upper_bound(
    segments_.begin() + 1, segments_.end(), uc,
    [](double value, const Segment & seg) { return value < seg.u0; });
  const auto & seg = *(it - 1);
  const double tau = std::clamp((uc - seg.u0) / seg.h, 0.0, 1.0);
  const double inv_h = 1.0 / seg.h;

  Eigen::Vector3d p, p1, p2, p3;
  eval_septic(seg.p_c, tau, inv_h, p, p1, p2, p3);

  Eigen::Vector3d th, dth, ddth, dddth;
  eval_septic(seg.r_c, tau, inv_h, th, dth, ddth, dddth);

  Eigen::Matrix3d Jl, dJl;
  so3_Jl_and_dJl(th, dth, Jl, dJl);

  out.pose = Eigen::Matrix4d::Identity();
  out.pose.topRightCorner<3, 1>() = p;
  out.pose.topLeftCorner<3, 3>() = seg.R0 * so3_exp(th);
  out.p_s = p1;
  out.p_ss = p2;
  out.p_sss = p3;
  out.w_s = seg.R0 * (Jl * dth);
  out.w_ss = seg.R0 * (dJl * dth + Jl * ddth);
  out.vel_scale = opts_.vel_scale;
  out.acc_scale = opts_.acc_scale;
  out.jerk_scale = opts_.jerk_scale;
  out.loc = seg.loc;
  return out;
}

std::vector<double> C3JointSpline::chord_length_parameters(
  const std::vector<std::array<double, 6>> & knots)
{
  const size_t n = knots.size();
  std::vector<double> u(n, 0.0);
  if (n < 2) {
    return u;
  }
  std::vector<double> d(n - 1, 0.0);
  double total = 0.0;
  for (size_t i = 0; i + 1 < n; ++i) {
    double sq = 0.0;
    for (size_t j = 0; j < 6; ++j) {
      const double diff = knots[i + 1][j] - knots[i][j];
      sq += diff * diff;
    }
    d[i] = std::sqrt(sq);
    total += d[i];
  }
  if (total < 1e-12) {
    for (size_t i = 0; i < n; ++i) {
      u[i] = static_cast<double>(i) / static_cast<double>(n - 1);
    }
    return u;
  }
  // Keep coincident knots strictly ordered.
  const double floor = 1e-6 * total;
  double acc = 0.0;
  for (size_t i = 0; i + 1 < n; ++i) {
    acc += std::max(d[i], floor);
    u[i + 1] = acc;
  }
  for (auto & v : u) {
    v /= acc;
  }
  u.back() = 1.0;
  return u;
}

void C3JointSpline::build(
  const std::vector<std::array<double, 6>> & knots,
  const std::vector<double> & u_knots)
{
  const int n = static_cast<int>(knots.size()) - 1;
  if (n < 1) {
    segments_.clear();
    return;
  }

  const std::vector<double> u =
    (u_knots.size() == knots.size()) ? u_knots : chord_length_parameters(knots);

  std::vector<Vec6> q(static_cast<size_t>(n + 1));
  for (int i = 0; i <= n; ++i) {
    for (int j = 0; j < 6; ++j) {
      q[static_cast<size_t>(i)][j] = knots[static_cast<size_t>(i)][static_cast<size_t>(j)];
    }
  }

  // 1. Solve natural cubic spline tridiagonal system to obtain baseline C^2 nodal derivatives (m1, m2).
  std::vector<Vec6> m2(static_cast<size_t>(n + 1), Vec6::Zero());
  if (n >= 2) {
    std::vector<double> lower(static_cast<size_t>(n + 1), 0.0);
    std::vector<double> diag(static_cast<size_t>(n + 1), 1.0);
    std::vector<double> upper(static_cast<size_t>(n + 1), 0.0);
    std::vector<Vec6> rhs(static_cast<size_t>(n + 1), Vec6::Zero());

    for (int i = 1; i < n; ++i) {
      const double hm = u[static_cast<size_t>(i)] - u[static_cast<size_t>(i - 1)];
      const double hp = u[static_cast<size_t>(i + 1)] - u[static_cast<size_t>(i)];
      lower[static_cast<size_t>(i)] = hm;
      diag[static_cast<size_t>(i)] = 2.0 * (hm + hp);
      upper[static_cast<size_t>(i)] = hp;
      rhs[static_cast<size_t>(i)] =
        6.0 * ((q[static_cast<size_t>(i + 1)] - q[static_cast<size_t>(i)]) / hp -
               (q[static_cast<size_t>(i)] - q[static_cast<size_t>(i - 1)]) / hm);
    }

    for (int i = 2; i < n; ++i) {
      const double w = lower[static_cast<size_t>(i)] / diag[static_cast<size_t>(i - 1)];
      diag[static_cast<size_t>(i)] -= w * upper[static_cast<size_t>(i - 1)];
      rhs[static_cast<size_t>(i)] -= w * rhs[static_cast<size_t>(i - 1)];
    }
    m2[static_cast<size_t>(n - 1)] =
      rhs[static_cast<size_t>(n - 1)] / diag[static_cast<size_t>(n - 1)];
    for (int i = n - 2; i >= 1; --i) {
      m2[static_cast<size_t>(i)] =
        (rhs[static_cast<size_t>(i)] -
         upper[static_cast<size_t>(i)] * m2[static_cast<size_t>(i + 1)]) /
        diag[static_cast<size_t>(i)];
    }
  }

  std::vector<Vec6> m1(static_cast<size_t>(n + 1), Vec6::Zero());
  for (int i = 0; i < n; ++i) {
    const double h = u[static_cast<size_t>(i + 1)] - u[static_cast<size_t>(i)];
    m1[static_cast<size_t>(i)] =
      (q[static_cast<size_t>(i + 1)] - q[static_cast<size_t>(i)]) / h -
      (h / 6.0) * (2.0 * m2[static_cast<size_t>(i)] + m2[static_cast<size_t>(i + 1)]);
  }
  {
    const double h = u[static_cast<size_t>(n)] - u[static_cast<size_t>(n - 1)];
    m1[static_cast<size_t>(n)] =
      (q[static_cast<size_t>(n)] - q[static_cast<size_t>(n - 1)]) / h +
      (h / 6.0) * (m2[static_cast<size_t>(n - 1)] + 2.0 * m2[static_cast<size_t>(n)]);
  }

  // 2. Compute continuous nodal third derivatives m3 by harmonic averaging of adjacent secant slopes of m2.
  std::vector<Vec6> m3(static_cast<size_t>(n + 1), Vec6::Zero());
  for (int i = 1; i < n; ++i) {
    const double hm = u[static_cast<size_t>(i)] - u[static_cast<size_t>(i - 1)];
    const double hp = u[static_cast<size_t>(i + 1)] - u[static_cast<size_t>(i)];
    const Vec6 sl_m = (m2[static_cast<size_t>(i)] - m2[static_cast<size_t>(i - 1)]) / hm;
    const Vec6 sl_p = (m2[static_cast<size_t>(i + 1)] - m2[static_cast<size_t>(i)]) / hp;
    m3[static_cast<size_t>(i)] = (hp * sl_m + hm * sl_p) / (hm + hp);
  }
  if (n >= 1) {
    m3[0] = (m2[1] - m2[0]) / (u[1] - u[0]);
    m3[static_cast<size_t>(n)] =
      (m2[static_cast<size_t>(n)] - m2[static_cast<size_t>(n - 1)]) /
      (u[static_cast<size_t>(n)] - u[static_cast<size_t>(n - 1)]);
  }

  build_hermite(q, m1, m2, m3, u);
}

void C3JointSpline::build_hermite(
  const std::vector<Vec6> & q,
  const std::vector<Vec6> & m1,
  const std::vector<Vec6> & m2,
  const std::vector<Vec6> & m3,
  const std::vector<double> & u)
{
  segments_.clear();
  pinned_boundaries_.clear();
  const size_t n = q.size();
  if (n < 2 || m1.size() != n || m2.size() != n || m3.size() != n || u.size() != n) {
    return;
  }
  segments_.reserve(n - 1);
  for (size_t idx = 0; idx + 1 < n; ++idx) {
    const double h = u[idx + 1] - u[idx];
    Segment seg;
    seg.u0 = u[idx];
    seg.h = h;
    seg.c = make_septic_hermite(
      q[idx], m1[idx], m2[idx], m3[idx],
      q[idx + 1], m1[idx + 1], m2[idx + 1], m3[idx + 1], h);
    segments_.push_back(seg);
  }
}

void C3JointSpline::eval(
  double u,
  Vec6 & q,
  Vec6 & q_u,
  Vec6 & q_uu,
  Vec6 & q_uuu) const
{
  if (segments_.empty()) {
    q.setZero();
    q_u.setZero();
    q_uu.setZero();
    q_uuu.setZero();
    return;
  }

  const double uc = std::clamp(u, 0.0, 1.0);
  const auto it = std::upper_bound(
    segments_.begin() + 1, segments_.end(), uc,
    [](double value, const Segment & seg) { return value < seg.u0; });
  const auto & seg = *(it - 1);
  const double tau = std::clamp((uc - seg.u0) / seg.h, 0.0, 1.0);
  eval_septic(seg.c, tau, 1.0 / seg.h, q, q_u, q_uu, q_uuu);
}

bool JointCompositePath::build(
  const std::vector<std::array<double, 6>> & waypoints,
  const std::vector<MotionOptions> & segment_opts,
  const std::vector<SourceLocation> & segment_locs,
  const std::vector<Corner> & corners,
  std::string & err_msg,
  std::vector<double> * out_corner_trims)
{
  pieces_.clear();
  s_cum_.clear();
  s_begin_ = 0.0;
  total_length_ = 0.0;
  if (out_corner_trims != nullptr) {
    out_corner_trims->clear();
  }
  if (waypoints.size() < 2) {
    err_msg = "Joint path requires at least two waypoints";
    return false;
  }
  const size_t n = waypoints.size() - 1;
  if (segment_opts.size() != n || segment_locs.size() != n ||
      corners.size() + 1 != n)
  {
    err_msg = "Joint path segment metadata size mismatch";
    return false;
  }

  std::vector<Vec6> w(waypoints.size());
  for (size_t k = 0; k < waypoints.size(); ++k) {
    for (int j = 0; j < 6; ++j) {
      w[k][j] = waypoints[k][static_cast<size_t>(j)];
    }
  }
  std::vector<double> len(n);
  std::vector<Vec6> dir(n);
  for (size_t i = 0; i < n; ++i) {
    const Vec6 d = w[i + 1] - w[i];
    len[i] = d.norm();
    if (len[i] < 1e-9) {
      err_msg = "Blended joint motion contains a zero-length segment";
      return false;
    }
    dir[i] = d / len[i];
  }

  // Symmetric joint trims at each corner guarantee all 8 degree-7 Bezier control points
  // lie strictly on [a, W] and [W, b] inside the corner triangle [a, W, b] (convex hull).
  std::vector<double> trim_start(n, 0.0), trim_end(n, 0.0), corner_trim(n > 0 ? n - 1 : 0, 0.0);
  for (size_t k = 0; k + 1 < n; ++k) {
    const double d = std::min({
      std::max(0.0, corners[k].trim_in),
      std::max(0.0, corners[k].trim_out),
      0.45 * len[k],
      0.45 * len[k + 1]});
    if (d > 1e-12) {
      const double cos_a = std::clamp(dir[k].dot(dir[k + 1]), -1.0, 1.0);
      if (cos_a < kMinBlendCosAngle) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1)
            << "Joint corner reversal angle (" << (std::acos(cos_a) * 180.0 / kPi)
            << " deg) exceeds the maximum smooth blend angle (" << kMaxBlendReversalAngleDeg
            << " deg); remove blend to insert a clean rest stop";
        err_msg = oss.str();
        return false;
      }
      corner_trim[k] = d;
      trim_end[k] = d;
      trim_start[k + 1] = d;
    }
  }

  for (size_t i = 0; i < n; ++i) {
    const double line_len = len[i] - trim_start[i] - trim_end[i];
    if (line_len > 1e-12) {
      Piece line;
      line.blend = false;
      line.length = line_len;
      line.start = w[i] + trim_start[i] * dir[i];
      line.dir = dir[i];
      line.opts = segment_opts[i];
      line.loc = segment_locs[i];
      pieces_.push_back(line);
    }
    if (i + 1 < n && corner_trim[i] > 1e-12) {
      const double d = corner_trim[i];
      const Vec6 & W = w[i + 1];
      Piece blend;
      blend.blend = true;
      blend.length = 2.0 * d;
      // Degree-7 Bernstein/Bezier control points strictly on [a, W] and [W, b]:
      // matches C^3 boundary conditions (a, dir_in, 0, 0) -> (b, dir_out, 0, 0),
      // stays strictly inside triangle [a, W, b], and reproduces collinear lines exactly.
      blend.bezier[0] = W - d * dir[i];
      blend.bezier[1] = W - (5.0 / 7.0) * d * dir[i];
      blend.bezier[2] = W - (3.0 / 7.0) * d * dir[i];
      blend.bezier[3] = W - (1.0 / 7.0) * d * dir[i];
      blend.bezier[4] = W + (1.0 / 7.0) * d * dir[i + 1];
      blend.bezier[5] = W + (3.0 / 7.0) * d * dir[i + 1];
      blend.bezier[6] = W + (5.0 / 7.0) * d * dir[i + 1];
      blend.bezier[7] = W + d * dir[i + 1];
      blend.opts.vel_scale = std::min(segment_opts[i].vel_scale, segment_opts[i + 1].vel_scale);
      blend.opts.acc_scale = std::min(segment_opts[i].acc_scale, segment_opts[i + 1].acc_scale);
      blend.opts.jerk_scale = std::min(segment_opts[i].jerk_scale, segment_opts[i + 1].jerk_scale);
      blend.loc = segment_locs[i];
      pieces_.push_back(blend);
    }
  }
  if (pieces_.empty()) {
    err_msg = "Joint path has no motion";
    return false;
  }
  s_cum_.assign(pieces_.size() + 1, 0.0);
  for (size_t i = 0; i < pieces_.size(); ++i) {
    s_cum_[i + 1] = s_cum_[i] + pieces_[i].length;
  }
  total_length_ = s_cum_.back();
  if (out_corner_trims != nullptr) {
    *out_corner_trims = std::move(corner_trim);
  }
  return true;
}

size_t JointCompositePath::locate(double s, double & s_local) const
{
  const auto it = std::upper_bound(s_cum_.begin() + 1, s_cum_.end() - 1, s);
  const size_t idx = static_cast<size_t>(std::distance(s_cum_.begin() + 1, it));
  s_local = std::clamp(s - s_cum_[idx], 0.0, pieces_[idx].length);
  return idx;
}

void JointCompositePath::eval(
  double u, Vec6 & q, Vec6 & q_u, Vec6 & q_uu, Vec6 & q_uuu) const
{
  if (pieces_.empty()) {
    q.setZero();
    q_u.setZero();
    q_uu.setZero();
    q_uuu.setZero();
    return;
  }
  double s_local = 0.0;
  const auto & piece = pieces_[locate(s_begin_ + std::clamp(u, 0.0, 1.0) * total_length_, s_local)];
  Vec6 q_s, q_ss, q_sss;
  if (piece.blend) {
    eval_bezier7(piece.bezier, s_local / piece.length, 1.0 / piece.length, q, q_s, q_ss, q_sss);
  } else {
    q = piece.start + s_local * piece.dir;
    q_s = piece.dir;
    q_ss.setZero();
    q_sss.setZero();
  }
  // Chain rule from joint arc length s to u in [0, 1]: ds/du = S.
  const double S = total_length_;
  q_u = q_s * S;
  q_uu = q_ss * (S * S);
  q_uuu = q_sss * (S * S * S);
}

std::vector<double> JointCompositePath::piece_boundaries() const
{
  std::vector<double> out;
  for (size_t i = 1; i + 1 < s_cum_.size(); ++i) {
    const double u = (s_cum_[i] - s_begin_) / total_length_;
    if (u > 1e-9 && u < 1.0 - 1e-9) {
      out.push_back(u);
    }
  }
  return out;
}

bool JointCompositePath::options_at(double u, MotionOptions & opts, SourceLocation & loc) const
{
  if (pieces_.empty()) {
    return false;
  }
  double s_local = 0.0;
  const auto & piece = pieces_[locate(s_begin_ + std::clamp(u, 0.0, 1.0) * total_length_, s_local)];
  opts = piece.opts;
  loc = piece.loc;
  return true;
}

JointCompositePath JointCompositePath::restricted(double u_start) const
{
  JointCompositePath sub = *this;
  const double u0 = std::clamp(u_start, 0.0, 1.0);
  sub.s_begin_ = s_begin_ + u0 * total_length_;
  sub.total_length_ = std::max(1e-12, total_length_ * (1.0 - u0));
  return sub;
}

RestrictedCartesianPath::RestrictedCartesianPath(
  std::shared_ptr<const CartesianPath> path, double u_start)
: path_(std::move(path)), begin_(std::clamp(u_start, 0.0, 1.0)), span_(1.0 - begin_)
{
}

double RestrictedCartesianPath::total_length() const
{
  return path_->total_length() * span_;
}

CartesianSample RestrictedCartesianPath::eval_normalized(double u) const
{
  auto sample = path_->eval_normalized(begin_ + span_ * std::clamp(u, 0.0, 1.0));
  sample.p_s *= span_;
  sample.w_s *= span_;
  sample.p_ss *= span_ * span_;
  sample.w_ss *= span_ * span_;
  sample.p_sss *= span_ * span_ * span_;
  return sample;
}

std::vector<double> RestrictedCartesianPath::piece_boundaries() const
{
  std::vector<double> result;
  for (double u : path_->piece_boundaries()) {
    if (u > begin_ && u < 1.0) {
      result.push_back((u - begin_) / span_);
    }
  }
  return result;
}

}  // namespace lite6_studio_planner
