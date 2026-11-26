#ifndef LITE6_STUDIO_PLANNER__CARTESIAN_COLLOCATION_SOLVER_HPP_
#define LITE6_STUDIO_PLANNER__CARTESIAN_COLLOCATION_SOLVER_HPP_

#include "lite6_interfaces/msg/compile_diagnostic.hpp"
#include "lite6_kinematics/kinematics_engine.hpp"
#include "lite6_studio_planner/curve_geometry.hpp"
#include "lite6_studio_planner/dsl_ast.hpp"
#include "lite6_studio_planner/topp_jerk_solver.hpp"

#include <Eigen/Dense>
#include <array>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace lite6_studio_planner
{

struct CartesianCollocationWaypoint
{
  SourceLocation loc;
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  // LOCAL_WORLD_ALIGNED spatial twist [vx, vy, vz, wx, wy, wz] (optional).
  std::optional<std::array<double, 6>> velocity;
  // Classical linear + world angular acceleration [ax, ay, az, alphax, alphay, alphaz] (optional).
  std::optional<std::array<double, 6>> acceleration;
  // Exact duration [s] for the segment ending at this waypoint (optional).
  std::optional<double> dt;
};

struct CartesianCollocationResult
{
  SampledTrajectory1kHz trajectory;
  // Exact CartesianPath wrapper over the solved piecewise-cubic joint trajectory (for Pause->Resume).
  std::shared_ptr<const CartesianPath> cartesian_path;
  std::vector<double> segment_durations;
  double total_duration = 0.0;
  int total_ipm_iterations = 0;
  int mesh_refinements = 0;
  bool used_cartesian_warm_start = false;
  bool converged = false;
  bool kkt_optimal = false;
};

// Adaptive cubic joint-space collocation for explicit Cartesian PVA.
// Enforces joint/Cartesian bounds and singularity constraints via Pinocchio
// derivatives, banded SQD IPM and 1 ms snapping, with collision-guided fallback.
class CartesianCollocationSolver
{
public:
  static bool solve(
    const lite6_kinematics::KinematicsEngine & engine,
    const std::array<double, 6> & start_q,
    const std::vector<CartesianCollocationWaypoint> & waypoints,
    const std::array<lite6_kinematics::JointLimit, 6> & joint_limits,
    const lite6_kinematics::CartesianLimit & cart_limits,
    const MotionOptions & options,
    CartesianCollocationResult & out_result,
    std::vector<lite6_interfaces::msg::CompileDiagnostic> & out_diagnostics,
    const std::function<bool()> & is_cancelled = {});

  // Utility exposed for unit testing analytical Pinocchio derivatives against finite differences.
  struct TcpDerivatives
  {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d v_lin = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_ang = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_lin = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_ang = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 6, 6> J_lwa = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 3, 6> dv_lin_dq = Eigen::Matrix<double, 3, 6>::Zero();
    Eigen::Matrix<double, 3, 6> da_lin_dq = Eigen::Matrix<double, 3, 6>::Zero();
    Eigen::Matrix<double, 3, 6> da_lin_dv = Eigen::Matrix<double, 3, 6>::Zero();
    Eigen::Matrix<double, 3, 6> da_lin_da = Eigen::Matrix<double, 3, 6>::Zero();
    Eigen::Matrix<double, 3, 6> dv_ang_dq = Eigen::Matrix<double, 3, 6>::Zero();
    Eigen::Matrix<double, 3, 6> da_ang_dq = Eigen::Matrix<double, 3, 6>::Zero();
    Eigen::Matrix<double, 3, 6> da_ang_dv = Eigen::Matrix<double, 3, 6>::Zero();
    Eigen::Matrix<double, 3, 6> da_ang_da = Eigen::Matrix<double, 3, 6>::Zero();
    double sigma_smooth = 0.0;
    Eigen::Matrix<double, 6, 1> dsigma_dq = Eigen::Matrix<double, 6, 1>::Zero();
  };

  static void evaluate_tcp_derivatives(
    const pinocchio::Model & model,
    pinocchio::Data & data,
    pinocchio::FrameIndex tcp_frame_id,
    const Vec6 & q,
    const Vec6 & dq,
    const Vec6 & ddq,
    bool compute_derivatives,
    TcpDerivatives & out);
};

}  // namespace lite6_studio_planner

#endif  // LITE6_STUDIO_PLANNER__CARTESIAN_COLLOCATION_SOLVER_HPP_
