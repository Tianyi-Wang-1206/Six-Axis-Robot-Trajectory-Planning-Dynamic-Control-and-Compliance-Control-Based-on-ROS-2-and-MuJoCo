#ifndef LITE6_STUDIO_PLANNER__DSL_AST_HPP_
#define LITE6_STUDIO_PLANNER__DSL_AST_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <optional>
#include <string>
#include <vector>

namespace lite6_studio_planner
{

using Vec6 = Eigen::Matrix<double, 6, 1>;

enum class MotionCommandType
{
  MOVE_J,
  MOVE_P,
  MOVE_L,
  MOVE_C_THREE_POINT,
  MOVE_C_CENTER_ANGLE
};

enum class SpaceType
{
  JOINT,
  CARTESIAN
};

enum class GripperActionType
{
  OPEN = 1,
  CLOSE = 2,
  STOP = 3
};

struct SourceLocation
{
  int line = 1;
  int column = 1;
  int statement_index = -1;
  std::string text;
};

struct MotionOptions
{
  double vel_scale = 0.5;
  double acc_scale = 0.5;
  double jerk_scale = 1.0;
  double blend_radius = 0.0;
};

struct MotionStatement
{
  SourceLocation loc;
  MotionCommandType type = MotionCommandType::MOVE_J;
  std::array<double, 6> joint_target{};
  Eigen::Matrix4d pose_target = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d pose_via = Eigen::Matrix4d::Identity();
  bool via_has_orientation = false;
  Eigen::Vector3d circle_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d circle_axis = Eigen::Vector3d::UnitZ();
  double circle_angle_rad = 0.0;
  bool has_explicit_target_ori = false;
  Eigen::Matrix3d circle_target_rot = Eigen::Matrix3d::Identity();
  MotionOptions options;
};

struct PVAWaypoint
{
  SourceLocation loc;
  std::array<double, 6> joint_pos{};
  Eigen::Matrix4d cart_pose = Eigen::Matrix4d::Identity();
  std::optional<std::array<double, 6>> velocity;
  std::optional<std::array<double, 6>> acceleration;
  std::optional<double> dt;
};

struct MovePVAStatement
{
  SourceLocation loc;
  SpaceType space = SpaceType::JOINT;
  MotionOptions options;
  std::vector<PVAWaypoint> waypoints;
};

struct GripperStatement
{
  SourceLocation loc;
  GripperActionType action = GripperActionType::STOP;
  bool wait_for_completion = true;
  double timeout_sec = 6.0;
};

struct WaitStatement
{
  SourceLocation loc;
  double duration_sec = 0.5;
};

struct PayloadStatement
{
  SourceLocation loc;
  double mass_kg = 0.0;
  std::array<double, 3> com_tcp{{0.0, 0.0, 0.0}};
};

struct ImpedanceParams
{
  std::array<double, 3> kt{{250.0, 250.0, 250.0}};
  std::array<double, 3> dt{{0.8, 0.8, 0.8}};
  std::array<double, 3> kr{{10.0, 10.0, 10.0}};
  std::array<double, 3> dr{{0.8, 0.8, 0.8}};
  double null_damping = 0.15;
  double transition_time = 0.15;
  std::array<double, 6> wrench_ff{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  uint8_t end_behavior = 0;
};

enum class StatementKind
{
  MOTION,
  MOVE_PVA,
  GRIPPER,
  WAIT,
  SET_PAYLOAD,
  IMPEDANCE_BLOCK
};

struct ASTStatement
{
  StatementKind kind = StatementKind::MOTION;
  SourceLocation loc;
  std::optional<MotionStatement> motion;
  std::optional<MovePVAStatement> move_pva;
  std::optional<GripperStatement> gripper;
  std::optional<WaitStatement> wait_stmt;
  std::optional<PayloadStatement> payload_stmt;
  std::optional<ImpedanceParams> impedance_params;
  std::vector<ASTStatement> body;
};

struct ASTProgram
{
  std::vector<ASTStatement> statements;
  int total_leaf_statements = 0;
};

}  // namespace lite6_studio_planner

#endif  // LITE6_STUDIO_PLANNER__DSL_AST_HPP_
