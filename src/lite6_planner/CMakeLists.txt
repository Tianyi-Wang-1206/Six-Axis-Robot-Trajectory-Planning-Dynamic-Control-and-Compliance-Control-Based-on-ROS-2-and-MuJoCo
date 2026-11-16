cmake_minimum_required(VERSION 3.8)
project(lite6_planner)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(lite6_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ruckig REQUIRED)
find_package(pinocchio REQUIRED)
find_package(coal REQUIRED)

add_library(kinematics_engine SHARED src/kinematics_engine.cpp)
target_include_directories(kinematics_engine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(kinematics_engine ament_index_cpp)
target_link_libraries(kinematics_engine pinocchio::pinocchio coal::coal yaml-cpp)

add_executable(lite6_planner_node
  src/lite6_planner_node.cpp
  src/motion_planner.cpp
  src/supervisor.cpp)
target_include_directories(lite6_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(lite6_planner_node
  rclcpp rclcpp_action std_msgs trajectory_msgs
  control_msgs ament_index_cpp lite6_interfaces)
target_link_libraries(lite6_planner_node ruckig::ruckig kinematics_engine)

install(TARGETS kinematics_engine lite6_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
ament_export_include_directories(include)
ament_export_libraries(kinematics_engine)
ament_export_dependencies(rclcpp rclcpp_action std_msgs
  trajectory_msgs control_msgs ament_index_cpp lite6_interfaces pinocchio coal yaml-cpp ruckig)

ament_package()
