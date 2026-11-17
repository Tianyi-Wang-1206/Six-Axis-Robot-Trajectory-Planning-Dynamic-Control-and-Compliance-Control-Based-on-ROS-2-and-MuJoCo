cmake_minimum_required(VERSION 3.8)
project(lite6_interfaces)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(LITE6_ENABLE_NATIVE_TUNE "Enable -mtune=native scheduling on non-Debug builds without changing ABI" ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic
    "$<$<NOT:$<CONFIG:Debug>>:-O3>"
    "$<$<NOT:$<CONFIG:Debug>>:-DNDEBUG>"
    "$<$<NOT:$<CONFIG:Debug>>:-fno-math-errno>"
    "$<$<NOT:$<CONFIG:Debug>>:-ffp-contract=fast>")
  if(LITE6_ENABLE_NATIVE_TUNE)
    include(CheckCXXCompilerFlag)
    check_cxx_compiler_flag("-mtune=native" COMPILER_SUPPORTS_MTUNE_NATIVE)
    if(COMPILER_SUPPORTS_MTUNE_NATIVE)
      add_compile_options("$<$<NOT:$<CONFIG:Debug>>:-mtune=native>")
    endif()
  endif()
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
  "srv/ControllerCommand.srv"
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
install(FILES python/lite6_motion_contract.py DESTINATION "${PYTHON_INSTALL_DIR}")

ament_export_targets(export_lite6_shm_bridge)
ament_export_libraries(lite6_shm_bridge)
ament_export_dependencies(rosidl_default_runtime)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_package()
