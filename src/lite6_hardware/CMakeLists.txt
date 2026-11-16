cmake_minimum_required(VERSION 3.8)
project(lite6_hardware)

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
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(lite6_interfaces REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(Threads REQUIRED)
find_package(glfw3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

set(MUJOCO_DIR $ENV{MUJOCO_DIR})
if(NOT MUJOCO_DIR)
  set(MUJOCO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco-3.9.0")
endif()
get_filename_component(MUJOCO_DIR "${MUJOCO_DIR}" ABSOLUTE)
set(CMAKE_INSTALL_RPATH "${MUJOCO_DIR}/lib")
include_directories(include ${MUJOCO_DIR}/include)
link_directories(${MUJOCO_DIR}/lib)

add_library(${PROJECT_NAME} SHARED src/lite6_mujoco_system.cpp src/gripper_control.cpp)
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
  std_msgs
)
target_link_libraries(${PROJECT_NAME} mujoco Threads::Threads yaml-cpp)

add_executable(mujoco_viewer src/mujoco_viewer.cpp)
ament_target_dependencies(mujoco_viewer rclcpp ament_index_cpp lite6_interfaces)
target_link_libraries(mujoco_viewer mujoco glfw Threads::Threads)

add_executable(robot_description_publisher src/robot_description_publisher.cpp)
ament_target_dependencies(robot_description_publisher rclcpp std_msgs)

install(TARGETS mujoco_viewer robot_description_publisher DESTINATION lib/${PROJECT_NAME})

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
ament_export_dependencies(rclcpp hardware_interface pluginlib ament_index_cpp diagnostic_msgs lite6_interfaces realtime_tools std_msgs)

ament_package()
