cmake_minimum_required(VERSION 3.8)
project(tadeo_ecar_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tadeo_ecar_msgs REQUIRED)
find_package(tadeo_ecar_interfaces REQUIRED)

# Include directories
include_directories(include)

# Emergency Stop Node
add_executable(emergency_stop_node src/emergency_stop_node.cpp)
ament_target_dependencies(emergency_stop_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tadeo_ecar_msgs
  tadeo_ecar_interfaces
)

# Collision Avoidance Node
add_executable(collision_avoidance_node src/collision_avoidance_node.cpp)
ament_target_dependencies(collision_avoidance_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tadeo_ecar_msgs
  tadeo_ecar_interfaces
)

# Safety Monitor Node
add_executable(safety_monitor_node src/safety_monitor_node.cpp)
ament_target_dependencies(safety_monitor_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tadeo_ecar_msgs
  tadeo_ecar_interfaces
)

# Watchdog Node
add_executable(watchdog_node src/watchdog_node.cpp)
ament_target_dependencies(watchdog_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tadeo_ecar_msgs
  tadeo_ecar_interfaces
)

# Install executables
install(TARGETS
  emergency_stop_node
  collision_avoidance_node
  safety_monitor_node
  watchdog_node
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