cmake_minimum_required(VERSION 3.8)
project(lite6_hardware)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(lite6_interfaces REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(Threads REQUIRED)
find_package(glfw3 REQUIRED)
find_package(sensor_msgs REQUIRED)

set(MUJOCO_DIR $ENV{MUJOCO_DIR})
if(NOT MUJOCO_DIR)
  set(MUJOCO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco-3.9.0")
endif()
get_filename_component(MUJOCO_DIR "${MUJOCO_DIR}" ABSOLUTE)
set(CMAKE_INSTALL_RPATH "${MUJOCO_DIR}/lib")
include_directories(include ${MUJOCO_DIR}/include)
link_directories(${MUJOCO_DIR}/lib)

add_library(${PROJECT_NAME} SHARED src/lite6_mujoco_system.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  ament_index_cpp
  diagnostic_msgs
  lite6_interfaces
  realtime_tools
)
target_link_libraries(${PROJECT_NAME} mujoco Threads::Threads)

add_executable(mujoco_viewer src/mujoco_viewer.cpp)
ament_target_dependencies(mujoco_viewer rclcpp ament_index_cpp sensor_msgs lite6_interfaces)
target_link_libraries(mujoco_viewer mujoco glfw Threads::Threads)
install(TARGETS mujoco_viewer DESTINATION lib/${PROJECT_NAME})

pluginlib_export_plugin_description_file(hardware_interface lite6_hardware.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp hardware_interface pluginlib ament_index_cpp diagnostic_msgs lite6_interfaces realtime_tools)

ament_package()
