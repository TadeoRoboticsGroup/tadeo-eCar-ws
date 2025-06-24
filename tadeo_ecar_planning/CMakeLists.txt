cmake_minimum_required(VERSION 3.8)
project(tadeo_ecar_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(tadeo_ecar_msgs REQUIRED)
find_package(tadeo_ecar_interfaces REQUIRED)

# Include directories
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OMPL_INCLUDE_DIRS})

# Global Path Planner Node
add_executable(global_path_planner_node src/global_path_planner_node.cpp)
ament_target_dependencies(global_path_planner_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  ompl
  tadeo_ecar_msgs
  tadeo_ecar_interfaces
)
target_link_libraries(global_path_planner_node ${OMPL_LIBRARIES})

# Local Path Planner Node
add_executable(local_path_planner_node src/local_path_planner_node.cpp)
ament_target_dependencies(local_path_planner_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  tadeo_ecar_msgs
  tadeo_ecar_interfaces
)

# Trajectory Optimizer Node
add_executable(trajectory_optimizer_node src/trajectory_optimizer_node.cpp)
ament_target_dependencies(trajectory_optimizer_node
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

# Path Manager Node
add_executable(path_manager_node src/path_manager_node.cpp)
ament_target_dependencies(path_manager_node
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

# Install executables
install(TARGETS
  global_path_planner_node
  local_path_planner_node
  trajectory_optimizer_node
  path_manager_node
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