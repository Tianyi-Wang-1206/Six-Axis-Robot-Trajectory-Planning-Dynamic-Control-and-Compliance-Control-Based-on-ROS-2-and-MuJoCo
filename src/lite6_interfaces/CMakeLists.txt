cmake_minimum_required(VERSION 3.8)
project(lite6_interfaces)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
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

add_library(lite6_shm_bridge SHARED src/shm_bridge.cpp)
target_include_directories(lite6_shm_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(lite6_shm_bridge rt)

install(TARGETS lite6_shm_bridge
  EXPORT export_lite6_shm_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_python_install_module(python/lite6_shm_client.py)

ament_export_targets(export_lite6_shm_bridge)
ament_export_libraries(lite6_shm_bridge)
ament_export_dependencies(rosidl_default_runtime)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_package()
