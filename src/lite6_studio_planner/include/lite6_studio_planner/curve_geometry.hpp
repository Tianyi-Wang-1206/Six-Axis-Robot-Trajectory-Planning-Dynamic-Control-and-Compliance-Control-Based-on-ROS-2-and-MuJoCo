#ifndef LITE6_STUDIO_PLANNER__CURVE_GEOMETRY_HPP_
#define LITE6_STUDIO_PLANNER__CURVE_GEOMETRY_HPP_

#include "lite6_studio_planner/dsl_ast.hpp"
#include "lite6_planner/kinematics_engine.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <memory>
#include <string>
#include <vector>

namespace lite6_studio_planner
{

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

// Composite G^2 / C^3 continuous Cartesian path built from one or more blended motion statements.
class CompositeCartesianPath
{
public:
  CompositeCartesianPath() = default;

  // Builds a composite Cartesian path from a sequence of MoveL / MoveC statements
  // starting from initial_pose, automatically inserting C^3 septic corner blends
  // wherever blend_radius > 0 between consecutive segments.
  bool build(
    const Eigen::Matrix4d & initial_pose,
    const std::vector<MotionStatement> & statements,
    SourceLocation & err_loc,
    std::string & err_code,
    std::string & err_msg);

  double total_length() const { return total_length_; }

  // Evaluates the composite path at normalized parameter u in [0, 1].
  // Derivatives in the returned CartesianSample are scaled with respect to u in [0, 1].
  CartesianSample eval_normalized(double u) const;

private:
  std::vector<std::shared_ptr<CartesianCurvePiece>> pieces_;
  std::vector<double> s_cum_;
  double total_length_ = 0.0;
};

// Globally C^3-continuous joint-space spline q(u) for u in [0, 1],
// providing exact analytical first, second, and third derivatives dq/du, d^2q/du^2, d^3q/du^3.
class C3JointSpline
{
public:
  C3JointSpline() = default;

  void build(
    const std::vector<std::array<double, 6>> & knots,
    const std::vector<double> & u_knots = {});

  void eval(
    double u,
    Vec6 & q,
    Vec6 & q_u,
    Vec6 & q_uu,
    Vec6 & q_uuu) const;

  const Vec6 & max_q1() const { return max_q1_; }
  const Vec6 & max_q2() const { return max_q2_; }
  const Vec6 & max_q3() const { return max_q3_; }

private:
  struct Segment
  {
    double u0 = 0.0;
    double h = 1.0;
    std::array<Vec6, 8> c{};
  };
  std::vector<Segment> segments_;
  Vec6 max_q1_ = Vec6::Zero();
  Vec6 max_q2_ = Vec6::Zero();
  Vec6 max_q3_ = Vec6::Zero();
};

}  // namespace lite6_studio_planner

#endif  // LITE6_STUDIO_PLANNER__CURVE_GEOMETRY_HPP_
