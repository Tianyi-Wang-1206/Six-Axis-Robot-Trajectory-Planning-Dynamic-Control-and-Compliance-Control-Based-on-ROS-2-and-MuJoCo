#ifndef LITE6_PLANNER_KINEMATICS_ENGINE_HPP_
#define LITE6_PLANNER_KINEMATICS_ENGINE_HPP_

#include <array>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include "lite6_planner/motion_tolerances.hpp"

namespace lite6_planner {

struct JointLimit {
  double min_pos = 0.0, max_pos = 0.0;
  double max_vel = 0.0, max_acc = 0.0, max_jerk = 0.0;
};
struct CartesianLimit {
  double max_trans_vel = 0.0, max_trans_acc = 0.0;
  double max_rot_vel = 0.0, max_rot_acc = 0.0;
};

enum class IKResult {
  SUCCESS = 1,
  ERR_NO_MATH_SOLUTION = -2,
  ERR_LIMIT_OR_COLLISION = -3,
  ERR_SINGULARITY = -4,
  ERR_QUADRANT_JUMP = -5
};

// Local planning diagnostics; the Action wire format and legacy codes stay stable.
struct IKDiagnostics {
  struct Rejection { size_t branch; std::string reason; std::string detail; };
  std::vector<Rejection> rejected;
  std::string input_error;
  std::string summary() const;
};

class KinematicsEngine {
public:
  struct CollisionWorkspace {
    pinocchio::Data data;
    pinocchio::GeometryData geometry;
    CollisionWorkspace(const pinocchio::Model& model,const pinocchio::GeometryModel& geoms)
      : data(model),geometry(geoms) {}
  };
  CollisionWorkspace collision_workspace() const {return {pin_model_,geom_model_};}
  KinematicsEngine();
  bool load_limits(const std::string& yaml_path);
  bool load_ik_config(const std::string& yaml_path);
  bool init_robot_model(const std::string& urdf_path, const std::string& srdf_path);
  IKResult solve_optimal_ik(const Eigen::Matrix4d& T,
      const std::array<double, 6>& q_ref,
      std::vector<std::array<double, 6>>& sorted,
      bool continuous, IKDiagnostics* diagnostics=nullptr) const;

  // Dimensionless, length-normalized minimum singular value of the 6x6 Jacobian.
  // Pinocchio orders spatial motions [linear; angular].
  double check_singularity(const std::array<double, 6>& q) const;
  // Geometry only (non-finite inputs are rejected). Callers validate joint limits
  // with the allowance appropriate to observations, targets or sampled trajectories.
  bool check_collision(const std::array<double, 6>& q, std::string* detail=nullptr) const;
  bool check_collision(const std::array<double,6>& q,CollisionWorkspace& workspace,
      std::string* detail=nullptr) const;
  bool joint_valid(const std::array<double,6>& q,
      double epsilon=tolerance::kJointPosition) const;
  Eigen::Matrix4d forward_pose(const std::array<double,6>& q) const;
  const pinocchio::Model& get_pin_model() const { return pin_model_; }
  const std::array<JointLimit,6>& get_joint_limits() const { return joint_limits_; }
  const CartesianLimit& get_cartesian_limits() const { return cart_limits_; }
  double get_singularity_threshold() const { return singularity_threshold_; }

private:
  // Original Maple-generated closed-form IK is retained unchanged.
  void raw_inverse_kinematics(const Eigen::Matrix4d& T, double solutions[8][6]) const;
  std::array<JointLimit,6> joint_limits_{};
  CartesianLimit cart_limits_{};
  double singularity_threshold_=0.001;
  double quadrant_jump_threshold_=0.5;
  std::vector<double> ground_size_{3.0,3.0,0.1};
  std::vector<double> ground_pos_{0.0,0.0,-0.05};
  std::array<double,6> ik_weights_{{1.5,1.5,1.2,1.0,1.0,0.8}};
  Eigen::Matrix4d flange_to_tcp_ = Eigen::Matrix4d::Identity();
  pinocchio::Model pin_model_;
  pinocchio::GeometryModel geom_model_;
};

} // namespace lite6_planner
#endif
