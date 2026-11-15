cmake_minimum_required(VERSION 3.8)
project(lite6_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/IndustrialMotion.action"
  "action/FollowImpedanceTrajectory.action"
  "msg/ComplianceSettings.msg"
  "msg/ComplianceCommand.msg"
  "msg/ControllerStatus.msg"
  "msg/SysIdSample.msg"
  "msg/SimulationState.msg"
  "msg/GripperState.msg"
  "msg/MousePerturbation.msg"
  "msg/SupervisorState.msg"
  "msg/MotionRejection.msg"
  "msg/CompileDiagnostic.msg"
  "msg/ImpedanceTrajectoryPoint.msg"
  "msg/StudioState.msg"
  "srv/SupervisorCommand.srv"
  "srv/GripperCommand.srv"
  "srv/SubmitSysIdTrajectory.srv"
  "srv/CompileProgram.srv"
  "srv/StudioCommand.srv"
  DEPENDENCIES
    action_msgs
    builtin_interfaces
    geometry_msgs
    trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_package()
