#include "lite6_studio_planner/curve_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace lite6_studio_planner
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kCharLength = 0.25;  // m/rad characteristic length for orientation arc length

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
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::AngleAxisd(ang, w / ang).toRotationMatrix();
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
    const SourceLocation & loc)
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
    rotvec_local_ = so3_log(R0_.transpose() * R1);
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
double septic_smoothstep(double tau)
{
  const double t = std::clamp(tau, 0.0, 1.0);
  const double t4 = t * t * t * t;
  return t4 * (35.0 + t * (-84.0 + t * (70.0 - 20.0 * t)));
}

// C^3 corner blend between the end of one piece (s0) and the start of the next (s1).
// Position: septic Hermite polynomial matching p, p_s, p_ss, p_sss at both ends.
// Orientation: R(s) = R_in(s) * Exp(alpha(s) * Log(R_in(s)^T R_out(s))), where R_in / R_out
// extrapolate the neighbouring pieces at their constant angular rates and alpha is a septic
// smoothstep. R therefore matches both neighbours up to the third derivative; its angular
// rate derivatives are evaluated numerically from this exact rotation function.
class SepticBlendPiece final : public CartesianCurvePiece
{
public:
  SepticBlendPiece(const CartesianSample & s0, const CartesianSample & s1)
  : Ra_(s0.pose.topLeftCorner<3, 3>()), Rb_(s1.pose.topLeftCorner<3, 3>()), loc_(s0.loc)
  {
    opts_.vel_scale = std::min(s0.vel_scale, s1.vel_scale);
    opts_.acc_scale = std::min(s0.acc_scale, s1.acc_scale);
    opts_.jerk_scale = std::min(s0.jerk_scale, s1.jerk_scale);

    const Eigen::Vector3d p0 = s0.pose.topRightCorner<3, 1>();
    const Eigen::Vector3d p1 = s1.pose.topRightCorner<3, 1>();
    const double chord = (p1 - p0).norm();
    const double rot_chord = kCharLength * so3_log(Ra_.transpose() * Rb_).norm();
    L_ = std::max({1e-4, 1.15 * chord, 1.15 * rot_chord});
    const double L2 = L_ * L_;
    const double L3 = L2 * L_;

    for (int axis = 0; axis < 3; ++axis) {
      const double c0 = p0[axis];
      const double c1 = s0.p_s[axis] * L_;
      const double c2 = 0.5 * s0.p_ss[axis] * L2;
      const double c3 = (1.0 / 6.0) * s0.p_sss[axis] * L3;

      const double d0 = p1[axis] - (c0 + c1 + c2 + c3);
      const double d1 = s1.p_s[axis] * L_ - (c1 + 2.0 * c2 + 3.0 * c3);
      const double d2 = s1.p_ss[axis] * L2 - (2.0 * c2 + 6.0 * c3);
      const double d3 = s1.p_sss[axis] * L3 - 6.0 * c3;

      const double c4 = 35.0 * d0 - 15.0 * d1 + 2.5 * d2 - (1.0 / 6.0) * d3;
      const double c5 = -84.0 * d0 + 39.0 * d1 - 7.0 * d2 + 0.5 * d3;
      const double c6 = 70.0 * d0 - 34.0 * d1 + 6.5 * d2 - 0.5 * d3;
      const double c7 = -20.0 * d0 + 10.0 * d1 - 2.0 * d2 + (1.0 / 6.0) * d3;
      pos_coeffs_[axis] = {{c0, c1, c2, c3, c4, c5, c6, c7}};
    }

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
    const double t2 = tau * tau;
    const double t3 = t2 * tau;
    const double t4 = t3 * tau;
    const double t5 = t4 * tau;
    const double t6 = t5 * tau;
    const double t7 = t6 * tau;

    const double invL = 1.0 / L_;
    const double invL2 = invL * invL;
    const double invL3 = invL2 * invL;

    CartesianSample out;
    out.pose = Eigen::Matrix4d::Identity();
    Eigen::Vector3d p, ps, pss, psss;
    for (int i = 0; i < 3; ++i) {
      const auto & c = pos_coeffs_[i];
      p[i] = c[0] + c[1] * tau + c[2] * t2 + c[3] * t3 + c[4] * t4 + c[5] * t5 + c[6] * t6 + c[7] * t7;
      ps[i] = (c[1] + 2.0 * c[2] * tau + 3.0 * c[3] * t2 + 4.0 * c[4] * t3 +
               5.0 * c[5] * t4 + 6.0 * c[6] * t5 + 7.0 * c[7] * t6) * invL;
      pss[i] = (2.0 * c[2] + 6.0 * c[3] * tau + 12.0 * c[4] * t2 +
                20.0 * c[5] * t3 + 30.0 * c[6] * t4 + 42.0 * c[7] * t5) * invL2;
      psss[i] = (6.0 * c[3] + 24.0 * c[4] * tau + 60.0 * c[5] * t2 +
                 120.0 * c[6] * t3 + 210.0 * c[7] * t4) * invL3;
    }

    out.pose.topRightCorner<3, 1>() = p;
    out.pose.topLeftCorner<3, 3>() = rotation(sigma);
    out.p_s = ps;
    out.p_ss = pss;
    out.p_sss = psss;
    // Five-point central stencil on the exact angular rate; the rotation function is
    // C^3 across the blend ends, so the stencil may straddle them.
    const double h = 2e-3 * L_;
    const Eigen::Vector3d wp2 = omega(sigma + 2.0 * h);
    const Eigen::Vector3d wp1 = omega(sigma + h);
    const Eigen::Vector3d wm1 = omega(sigma - h);
    const Eigen::Vector3d wm2 = omega(sigma - 2.0 * h);
    out.w_s = omega(sigma);
    out.w_ss = (-wp2 + 8.0 * wp1 - 8.0 * wm1 + wm2) / (12.0 * h);
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

  // Defined for any sigma; outside [0, L] it continues the neighbouring piece.
  Eigen::Matrix3d rotation(double sigma) const
  {
    const Eigen::Matrix3d r_in = Ra_ * so3_exp(sigma * wa_body_);
    const Eigen::Matrix3d r_out = Rb_ * so3_exp((sigma - L_) * wb_body_);
    const Eigen::Vector3d d = so3_log(r_in.transpose() * r_out);
    return r_in * so3_exp(septic_smoothstep(sigma / L_) * d);
  }

  // World-frame angular rate per unit s by a central difference of the rotation.
  Eigen::Vector3d omega(double sigma) const
  {
    const double h = 1e-4 * L_;
    return so3_log(rotation(sigma + h) * rotation(sigma - h).transpose()) / (2.0 * h);
  }

  Eigen::Matrix3d Ra_, Rb_;
  Eigen::Vector3d wa_body_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d wb_body_ = Eigen::Vector3d::Zero();
  std::array<std::array<double, 8>, 3> pos_coeffs_{};
  double L_ = 1e-3;
  bool valid_ = true;
  MotionOptions opts_;
  SourceLocation loc_;
};

}  // namespace

bool CompositeCartesianPath::build(
  const Eigen::Matrix4d & initial_pose,
  const std::vector<MotionStatement> & statements,
  SourceLocation & err_loc,
  std::string & err_code,
  std::string & err_msg)
{
  pieces_.clear();
  s_cum_.clear();
  total_length_ = 0.0;

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
    RawPrim rp;
    rp.blend_after = (idx + 1 < statements.size()) ? st.options.blend_radius : 0.0;

    if (st.type == MotionCommandType::MOVE_L) {
      rp.lin = std::make_shared<LinearPiece>(curr_pose, st.pose_target, st.options, st.loc);
      rp.full_len = rp.lin->full_length();
      rp.trim_start = 0.0;
      rp.trim_end = rp.full_len;
      curr_pose = st.pose_target;
    } else if (st.type == MotionCommandType::MOVE_C_THREE_POINT) {
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
    } else if (st.type == MotionCommandType::MOVE_C_CENTER_ANGLE) {
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
      const Eigen::Matrix3d R1 = st.has_explicit_target_ori ?
        st.circle_target_rot :
        (Eigen::AngleAxisd(st.circle_angle_rad, ez).toRotationMatrix() * R0);

      rp.circ = std::make_shared<CircularPiece>(
        eff_center, ex, ey, ez, radius, 0.0, st.circle_angle_rad,
        R0, R1, st.options, st.loc);
      rp.full_len = rp.circ->full_length();
      rp.trim_start = 0.0;
      rp.trim_end = rp.full_len;
      curr_pose = rp.circ->eval_untrimmed(rp.full_len).pose;
    } else {
      err_loc = st.loc;
      err_code = "ERR_UNSUPPORTED_CART_PRIMITIVE";
      err_msg = "Unsupported primitive in Cartesian composite path";
      return false;
    }
    raw_prims.push_back(rp);
  }

  // Compute trim distances for G^2 / C^3 corner blending between consecutive segments.
  std::vector<double> trim_len(raw_prims.size(), 0.0);
  for (size_t i = 0; i + 1 < raw_prims.size(); ++i) {
    const double r = raw_prims[i].blend_after;
    if (r > 1e-5) {
      trim_len[i] = std::min({r, 0.42 * raw_prims[i].full_len, 0.42 * raw_prims[i + 1].full_len});
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
      auto blend = std::make_shared<SepticBlendPiece>(s_in, s_out);
      if (!blend->valid()) {
        err_loc = statements[i + 1].loc;
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

  std::vector<Vec6> qv(q.begin(), q.end());
  build_hermite(qv, m1, m2, m3, u);
}

void C3JointSpline::build_hermite(
  const std::vector<Vec6> & q,
  const std::vector<Vec6> & m1,
  const std::vector<Vec6> & m2,
  const std::vector<Vec6> & m3,
  const std::vector<double> & u)
{
  segments_.clear();
  const size_t n = q.size();
  if (n < 2 || m1.size() != n || m2.size() != n || m3.size() != n || u.size() != n) {
    return;
  }
  segments_.reserve(n - 1);
  for (size_t idx = 0; idx + 1 < n; ++idx) {
    const double h = u[idx + 1] - u[idx];
    const double h2 = h * h;
    const double h3 = h2 * h;

    Segment seg;
    seg.u0 = u[idx];
    seg.h = h;
    seg.c[0] = q[idx];
    seg.c[1] = m1[idx] * h;
    seg.c[2] = 0.5 * m2[idx] * h2;
    seg.c[3] = (1.0 / 6.0) * m3[idx] * h3;

    const Vec6 d0 = q[idx + 1] - (seg.c[0] + seg.c[1] + seg.c[2] + seg.c[3]);
    const Vec6 d1 = m1[idx + 1] * h - (seg.c[1] + 2.0 * seg.c[2] + 3.0 * seg.c[3]);
    const Vec6 d2 = m2[idx + 1] * h2 - (2.0 * seg.c[2] + 6.0 * seg.c[3]);
    const Vec6 d3 = m3[idx + 1] * h3 - 6.0 * seg.c[3];

    seg.c[4] = 35.0 * d0 - 15.0 * d1 + 2.5 * d2 - (1.0 / 6.0) * d3;
    seg.c[5] = -84.0 * d0 + 39.0 * d1 - 7.0 * d2 + 0.5 * d3;
    seg.c[6] = 70.0 * d0 - 34.0 * d1 + 6.5 * d2 - 0.5 * d3;
    seg.c[7] = -20.0 * d0 + 10.0 * d1 - 2.0 * d2 + (1.0 / 6.0) * d3;
    segments_.push_back(seg);
  }
  finalize_bounds();
}

void C3JointSpline::finalize_bounds()
{
  max_q1_.setZero();
  max_q2_.setZero();
  max_q3_.setZero();
  const int samples = std::max<int>(200, static_cast<int>(4 * segments_.size()));
  for (int k = 0; k <= samples; ++k) {
    const double uk = static_cast<double>(k) / samples;
    Vec6 qv, q1, q2, q3;
    eval(uk, qv, q1, q2, q3);
    max_q1_ = max_q1_.cwiseMax(q1.cwiseAbs());
    max_q2_ = max_q2_.cwiseMax(q2.cwiseAbs());
    max_q3_ = max_q3_.cwiseMax(q3.cwiseAbs());
  }
}

namespace
{
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
}  // namespace

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
  std::string & err_msg)
{
  pieces_.clear();
  s_cum_.clear();
  s_begin_ = 0.0;
  total_length_ = 0.0;
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

  // Trims removed at the start (after corner i-1) and end (before corner i) of segment i.
  std::vector<double> trim_start(n, 0.0), trim_end(n, 0.0);
  for (size_t k = 0; k + 1 < n; ++k) {
    trim_end[k] = std::clamp(corners[k].trim_in, 0.0, 0.45 * len[k]);
    trim_start[k + 1] = std::clamp(corners[k].trim_out, 0.0, 0.45 * len[k + 1]);
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
    if (i + 1 < n && (trim_end[i] > 1e-12 || trim_start[i + 1] > 1e-12)) {
      const Vec6 a = w[i + 1] - trim_end[i] * dir[i];
      const Vec6 b = w[i + 1] + trim_start[i + 1] * dir[i + 1];
      Piece blend;
      blend.blend = true;
      blend.length = std::max(
        1e-9, 0.5 * (trim_end[i] + trim_start[i + 1]) + 0.5 * (b - a).norm());
      const double L = blend.length;
      // Septic Hermite: unit tangents at both ends, zero 2nd/3rd derivatives.
      blend.c[0] = a;
      blend.c[1] = dir[i] * L;
      blend.c[2].setZero();
      blend.c[3].setZero();
      const Vec6 d0 = b - (blend.c[0] + blend.c[1]);
      const Vec6 d1 = dir[i + 1] * L - blend.c[1];
      blend.c[4] = 35.0 * d0 - 15.0 * d1;
      blend.c[5] = -84.0 * d0 + 39.0 * d1;
      blend.c[6] = 70.0 * d0 - 34.0 * d1;
      blend.c[7] = -20.0 * d0 + 10.0 * d1;
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
    eval_septic(piece.c, s_local / piece.length, 1.0 / piece.length, q, q_s, q_ss, q_sss);
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

}  // namespace lite6_studio_planner
