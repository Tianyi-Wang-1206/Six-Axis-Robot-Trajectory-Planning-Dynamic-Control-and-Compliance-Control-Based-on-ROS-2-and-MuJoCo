#ifndef LITE6_STUDIO_PLANNER__CURVE_GEOMETRY_HPP_
#define LITE6_STUDIO_PLANNER__CURVE_GEOMETRY_HPP_

#include "lite6_studio_planner/dsl_ast.hpp"
#include "lite6_kinematics/kinematics_engine.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <memory>
#include <string>
#include <vector>

namespace lite6_studio_planner
{

// Pose and exact path derivatives of the TCP with respect to the path parameter.
// p_* are TCP origin derivatives; w_s is the world-frame angular velocity per unit
// path parameter and w_ss its derivative (LOCAL_WORLD_ALIGNED convention).
struct CartesianSample
{
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  Eigen::Vector3d p_s = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_ss = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_sss = Eigen::Vector3d::Zero();
  Eigen::Vector3d w_s = Eigen::Vector3d::Zero();
  Eigen::Vector3d w_ss = Eigen::Vector3d::Zero();
  double vel_scale = 0.5;
  double acc_scale = 0.5;
  double jerk_scale = 1.0;
  SourceLocation loc;
};

class CartesianCurvePiece
{
public:
  virtual ~CartesianCurvePiece() = default;
  virtual double length() const = 0;
  virtual CartesianSample eval(double s_local) const = 0;
  virtual SourceLocation location() const = 0;
  virtual MotionOptions options() const = 0;
};

// Abstract normalized Cartesian path parameterized on u in [0, 1].
class CartesianPath
{
public:
  virtual ~CartesianPath() = default;

  // Length of the path in pseudo arc length [m].
  virtual double total_length() const = 0;

  // Evaluates the path at normalized parameter u in [0, 1].
  // Derivatives in the returned CartesianSample are taken with respect to u.
  virtual CartesianSample eval_normalized(double u) const = 0;

  // Normalized parameters in (0, 1) where one piece or spline segment ends and the next begins.
  virtual std::vector<double> piece_boundaries() const = 0;
};

// A view of an immutable Cartesian path, retaining its exact geometry and derivatives.
class RestrictedCartesianPath : public CartesianPath
{
public:
  RestrictedCartesianPath(std::shared_ptr<const CartesianPath> path, double u_start);
  double total_length() const override;
  CartesianSample eval_normalized(double u) const override;
  std::vector<double> piece_boundaries() const override;

private:
  std::shared_ptr<const CartesianPath> path_;
  double begin_;
  double span_;
};

// Structured non-fatal warning produced during geometric blend construction.
struct BlendWarning
{
  SourceLocation loc;
  std::string code;
  std::string message;
};

// Composite C^3 continuous Cartesian path built from one or more blended motion statements.
class CompositeCartesianPath : public CartesianPath
{
public:
  CompositeCartesianPath() = default;

  // Builds a composite Cartesian path from a sequence of MoveL / MoveC statements
  // starting from initial_pose, inserting C^3 Bernstein/Bezier corner blends (position and
  // orientation) wherever blend_radius > 0 between consecutive segments.
  bool build(
    const Eigen::Matrix4d & initial_pose,
    const std::vector<MotionStatement> & statements,
    SourceLocation & err_loc,
    std::string & err_code,
    std::string & err_msg,
    std::vector<BlendWarning> * out_warnings = nullptr);

  double total_length() const override { return total_length_; }

  CartesianSample eval_normalized(double u) const override;

  std::vector<double> piece_boundaries() const override;

  // Returns the sub-path [u_start, 1] of this path, re-normalized to [0, 1].
  CompositeCartesianPath restricted(double u_start) const;

private:
  std::vector<std::shared_ptr<CartesianCurvePiece>> pieces_;
  std::vector<double> s_cum_;
  double s_begin_ = 0.0;
  double total_length_ = 0.0;
};

// Smooth C^3 Cartesian spline through a sequence of SE(3) poses:
//   - Position: globally C^3 piecewise septic Hermite spline parameterized by pseudo arc length.
//   - Orientation: globally C^3 piecewise Lie-algebra septic Hermite spline matching continuous
//     world angular velocity, angular acceleration, and angular jerk across all interior waypoints
//     via exact first and second derivatives of the SO(3) left Jacobian.
class C3CartesianSpline : public CartesianPath
{
public:
  C3CartesianSpline() = default;

  bool build(
    const std::vector<Eigen::Matrix4d> & poses,
    const std::vector<SourceLocation> & seg_locs,
    const MotionOptions & opts,
    std::string & err_msg);

  double total_length() const override { return total_length_; }

  CartesianSample eval_normalized(double u) const override;

  std::vector<double> piece_boundaries() const override;

private:
  struct Segment
  {
    double u0 = 0.0;
    double h = 1.0;
    Eigen::Matrix3d R0 = Eigen::Matrix3d::Identity();
    std::array<Eigen::Vector3d, 8> p_c{};
    std::array<Eigen::Vector3d, 8> r_c{};
    SourceLocation loc;
  };
  std::vector<Segment> segments_;
  MotionOptions opts_;
  double total_length_ = 1e-6;
};

// Geometric joint-space path q(u), u in [0, 1], with analytic derivatives.
class JointPath
{
public:
  virtual ~JointPath() = default;
  virtual void eval(double u, Vec6 & q, Vec6 & q_u, Vec6 & q_uu, Vec6 & q_uuu) const = 0;
  // Normalized parameters in (0, 1) where geometric pieces meet.
  virtual std::vector<double> piece_boundaries() const { return {}; }
  // Per-u motion options and source location; returns false to use caller defaults.
  virtual bool options_at(double u, MotionOptions & opts, SourceLocation & loc) const
  {
    (void)u;
    (void)opts;
    (void)loc;
    return false;
  }
};

// Globally C^3-continuous piecewise septic Hermite joint spline q(u), u in [0, 1].
class C3JointSpline : public JointPath
{
public:
  C3JointSpline() = default;

  // Interpolating spline with estimated nodal derivatives (natural cubic spline for
  // first/second derivatives). An empty u_knots selects chord-length parameterization.
  void build(
    const std::vector<std::array<double, 6>> & knots,
    const std::vector<double> & u_knots = {});

  // Hermite spline through knots with exact nodal derivatives dq/du, d2q/du2, d3q/du3.
  // u_knots must be strictly increasing from 0 to 1 and match knots in size.
  void build_hermite(
    const std::vector<Vec6> & knots,
    const std::vector<Vec6> & m1,
    const std::vector<Vec6> & m2,
    const std::vector<Vec6> & m3,
    const std::vector<double> & u_knots);

  // Cumulative joint-space chord lengths normalized to [0, 1] (uniform if degenerate).
  static std::vector<double> chord_length_parameters(
    const std::vector<std::array<double, 6>> & knots);

  // Opt-in pinned TOPP collocation boundaries in (0, 1). Empty by default so dense Cartesian
  // IK splines and resume splines do not force excessive TOPP knots; populated only for
  // sparse user-waypoint splines such as pure-(P) joint MovePVA.
  void set_pinned_boundaries(std::vector<double> boundaries)
  {
    pinned_boundaries_ = std::move(boundaries);
  }

  void eval(double u, Vec6 & q, Vec6 & q_u, Vec6 & q_uu, Vec6 & q_uuu) const override;
  std::vector<double> piece_boundaries() const override { return pinned_boundaries_; }

private:
  struct Segment
  {
    double u0 = 0.0;
    double h = 1.0;
    std::array<Vec6, 8> c{};
  };
  std::vector<Segment> segments_;
  std::vector<double> pinned_boundaries_;
};

// Joint-space polyline through waypoints with explicit C^3 degree-7 Bernstein/Bezier
// corner blends whose control points lie inside each corner triangle [a, W, b]
// (guaranteeing the convex-hull property and preventing corner overshoot).
class JointCompositePath : public JointPath
{
public:
  struct Corner
  {
    double trim_in = 0.0;   // joint-space length removed before waypoint k [rad]
    double trim_out = 0.0;  // joint-space length removed after waypoint k [rad]
  };

  // waypoints.size() = n + 1 for n segments; segment_opts / segment_locs have n entries;
  // corners has n - 1 entries (corner k sits at waypoint k + 1).
  bool build(
    const std::vector<std::array<double, 6>> & waypoints,
    const std::vector<MotionOptions> & segment_opts,
    const std::vector<SourceLocation> & segment_locs,
    const std::vector<Corner> & corners,
    std::string & err_msg,
    std::vector<double> * out_corner_trims = nullptr);

  bool empty() const { return pieces_.empty(); }
  double total_length() const { return total_length_; }

  void eval(double u, Vec6 & q, Vec6 & q_u, Vec6 & q_uu, Vec6 & q_uuu) const override;
  std::vector<double> piece_boundaries() const override;
  bool options_at(double u, MotionOptions & opts, SourceLocation & loc) const override;

  // Returns the sub-path [u_start, 1], re-normalized to [0, 1].
  JointCompositePath restricted(double u_start) const;

private:
  struct Piece
  {
    bool blend = false;
    double length = 0.0;
    Vec6 start = Vec6::Zero();
    Vec6 dir = Vec6::Zero();
    std::array<Vec6, 8> bezier{};  // degree-7 Bernstein/Bezier control points (blend pieces)
    MotionOptions opts;
    SourceLocation loc;
  };
  size_t locate(double s, double & s_local) const;
  std::vector<Piece> pieces_;
  std::vector<double> s_cum_;
  double s_begin_ = 0.0;
  double total_length_ = 0.0;
};

}  // namespace lite6_studio_planner

#endif  // LITE6_STUDIO_PLANNER__CURVE_GEOMETRY_HPP_
