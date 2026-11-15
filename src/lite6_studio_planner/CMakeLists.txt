cmake_minimum_required(VERSION 3.8)
project(lite6_studio_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(lite6_interfaces REQUIRED)
find_package(lite6_planner REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(coal REQUIRED)

add_library(studio_planner_core SHARED
  src/dsl_parser.cpp
  src/s_curve_solver.cpp
  src/curve_geometry.cpp
  src/topp_jerk_solver.cpp
  src/program_compiler.cpp
)
target_include_directories(studio_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(studio_planner_core
  rclcpp
  trajectory_msgs
  ament_index_cpp
  lite6_interfaces
  lite6_planner
)
target_link_libraries(studio_planner_core
  pinocchio::pinocchio
  coal::coal
  yaml-cpp
)

install(TARGETS studio_planner_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(studio_planner_core)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  trajectory_msgs
  control_msgs
  ament_index_cpp
  lite6_interfaces
  lite6_planner
  pinocchio
  coal
  yaml-cpp
)

add_executable(lite6_studio_planner_node
  src/studio_supervisor.cpp
  src/studio_planner_node.cpp
)

target_include_directories(lite6_studio_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(lite6_studio_planner_node
  studio_planner_core
)

ament_target_dependencies(lite6_studio_planner_node
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  trajectory_msgs
  control_msgs
  ament_index_cpp
  lite6_interfaces
  lite6_planner
  pinocchio
  coal
  yaml-cpp
)

install(TARGETS lite6_studio_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
