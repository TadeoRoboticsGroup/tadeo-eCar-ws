cmake_minimum_required(VERSION 3.8)
project(tadeo_ecar_localization)

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
find_package(Eigen3 REQUIRED)
find_package(tadeo_ecar_msgs REQUIRED)

# Include directories
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

# EKF Localization Node
add_executable(ekf_localization_node src/ekf_localization_node.cpp)
ament_target_dependencies(ekf_localization_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  tadeo_ecar_msgs
)

# UKF Localization Node
add_executable(ukf_localization_node src/ukf_localization_node.cpp)
ament_target_dependencies(ukf_localization_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  tadeo_ecar_msgs
)

# Sensor Fusion Node
add_executable(sensor_fusion_localization_node src/sensor_fusion_localization_node.cpp)
ament_target_dependencies(sensor_fusion_localization_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  tadeo_ecar_msgs
)

# Map Matcher Node
add_executable(map_matcher_node src/map_matcher_node.cpp)
ament_target_dependencies(map_matcher_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  tadeo_ecar_msgs
)

# Install executables
install(TARGETS
  ekf_localization_node
  ukf_localization_node
  sensor_fusion_localization_node
  map_matcher_node
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