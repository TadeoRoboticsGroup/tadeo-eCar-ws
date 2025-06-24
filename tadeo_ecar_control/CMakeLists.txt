cmake_minimum_required(VERSION 3.8)
project(tadeo_ecar_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tadeo_ecar_msgs REQUIRED)
find_package(tadeo_ecar_interfaces REQUIRED)

# Include directories
include_directories(include)

# Wheel Controller Node
add_executable(wheel_controller_node src/wheel_controller_node.cpp)
ament_target_dependencies(wheel_controller_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tadeo_ecar_msgs
  tadeo_ecar_interfaces
)

# 4WS Controller Node
add_executable(four_wheel_steering_controller_node src/four_wheel_steering_controller_node.cpp)
ament_target_dependencies(four_wheel_steering_controller_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tadeo_ecar_msgs
  tadeo_ecar_interfaces
)

# Vehicle Dynamics Node
add_executable(vehicle_dynamics_node src/vehicle_dynamics_node.cpp)
ament_target_dependencies(vehicle_dynamics_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tadeo_ecar_msgs
)

# Install executables
install(TARGETS
  wheel_controller_node
  four_wheel_steering_controller_node
  vehicle_dynamics_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  include/
  launch/
  config/
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()