cmake_minimum_required(VERSION 3.8)
project(tadeo_ecar_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(angles REQUIRED)
find_package(tadeo_ecar_msgs REQUIRED)
find_package(tadeo_ecar_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

# Navigation Controller Node
add_executable(navigation_controller_node src/navigation_controller_node.cpp)
ament_target_dependencies(navigation_controller_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  nav2_util
  Eigen3
  angles
  tadeo_ecar_msgs
  tadeo_ecar_interfaces
)

# Mission Executor Node
add_executable(mission_executor_node src/mission_executor_node.cpp)
ament_target_dependencies(mission_executor_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  Eigen3
  tadeo_ecar_msgs
  tadeo_ecar_interfaces
)
target_link_libraries(mission_executor_node yaml-cpp)

# Waypoint Manager Node
add_executable(waypoint_manager_node src/waypoint_manager_node.cpp)
ament_target_dependencies(waypoint_manager_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  tadeo_ecar_msgs
  tadeo_ecar_interfaces
)
target_link_libraries(waypoint_manager_node yaml-cpp)

# Navigation Monitor Node
add_executable(navigation_monitor_node src/navigation_monitor_node.cpp)
ament_target_dependencies(navigation_monitor_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  nav2_costmap_2d
  Eigen3
  tadeo_ecar_msgs
)

# Install executables
install(TARGETS
  navigation_controller_node
  mission_executor_node
  waypoint_manager_node
  navigation_monitor_node
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