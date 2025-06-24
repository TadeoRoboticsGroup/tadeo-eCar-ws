cmake_minimum_required(VERSION 3.8)
project(tadeo_ecar_slam)

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
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tadeo_ecar_msgs REQUIRED)
find_package(tadeo_ecar_interfaces REQUIRED)

# Include directories
include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

# Grid SLAM Node
add_executable(grid_slam_node src/grid_slam_node.cpp)
ament_target_dependencies(grid_slam_node
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
  tadeo_ecar_interfaces
)

# Graph SLAM Node
add_executable(graph_slam_node src/graph_slam_node.cpp)
ament_target_dependencies(graph_slam_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  PCL
  pcl_conversions
  tadeo_ecar_msgs
  tadeo_ecar_interfaces
)
target_link_libraries(graph_slam_node ${PCL_LIBRARIES})

# Map Manager Node
add_executable(map_manager_node src/map_manager_node.cpp)
ament_target_dependencies(map_manager_node
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

# Loop Detector Node
add_executable(loop_detector_node src/loop_detector_node.cpp)
ament_target_dependencies(loop_detector_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  PCL
  pcl_conversions
  tadeo_ecar_msgs
)
target_link_libraries(loop_detector_node ${PCL_LIBRARIES})

# Install executables
install(TARGETS
  grid_slam_node
  graph_slam_node
  map_manager_node
  loop_detector_node
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