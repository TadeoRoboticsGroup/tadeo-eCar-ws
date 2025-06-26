cmake_minimum_required(VERSION 3.8)
project(tadeo_ecar_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# nav2 packages removed for compilation compatibility
# find_package(nav2_msgs REQUIRED)
# find_package(nav2_behavior_tree REQUIRED)
# find_package(nav2_util REQUIRED)
# find_package(nav2_core REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(tadeo_ecar_msgs REQUIRED)
find_package(tadeo_ecar_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

# Behavior Manager Node
add_executable(behavior_manager_node src/behavior_manager_node.cpp)
ament_target_dependencies(behavior_manager_node
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  # nav2_msgs
  # nav2_behavior_tree
  # nav2_util
  # nav2_core
  behaviortree_cpp_v3
  Eigen3
  tadeo_ecar_msgs
  tadeo_ecar_interfaces
)
target_link_libraries(behavior_manager_node yaml-cpp)

# Behavior Tree Action Nodes Library
add_library(tadeo_bt_nodes SHARED
  src/bt_action_nodes/navigation_actions.cpp
  src/bt_action_nodes/patrol_actions.cpp
  src/bt_action_nodes/exploration_actions.cpp
  src/bt_action_nodes/safety_conditions.cpp
  src/bt_action_nodes/utility_actions.cpp
)
ament_target_dependencies(tadeo_bt_nodes
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  # nav2_msgs
  # nav2_behavior_tree
  # nav2_util
  behaviortree_cpp_v3
  Eigen3
  tadeo_ecar_msgs
  tadeo_ecar_interfaces
)
target_link_libraries(tadeo_bt_nodes yaml-cpp)

# State Machine Node (optional advanced behavior management)
add_executable(state_machine_node src/state_machine_node.cpp)
ament_target_dependencies(state_machine_node
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  Eigen3
  tadeo_ecar_msgs
  tadeo_ecar_interfaces
)
target_link_libraries(state_machine_node yaml-cpp)

# Behavior Monitor Node
add_executable(behavior_monitor_node src/behavior_monitor_node.cpp)
ament_target_dependencies(behavior_monitor_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  Eigen3
  tadeo_ecar_msgs
)
target_link_libraries(behavior_monitor_node yaml-cpp)

# Install executables
install(TARGETS
  behavior_manager_node
  state_machine_node
  behavior_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install shared libraries
install(TARGETS
  tadeo_bt_nodes
  DESTINATION lib
)

# Install directories
install(DIRECTORY
  include/
  launch/
  config/
  behavior_trees/
  DESTINATION share/${PROJECT_NAME}/
)

# Install behavior tree XML files specifically
install(FILES
  behavior_trees/main_behavior.xml
  behavior_trees/patrol_behavior.xml
  behavior_trees/exploration_behavior.xml
  DESTINATION share/${PROJECT_NAME}/behavior_trees/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # Add unit tests
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(test_behavior_types test/test_behavior_types.cpp)
  ament_target_dependencies(test_behavior_types rclcpp)
  
  ament_add_gtest(test_behavior_manager test/test_behavior_manager.cpp)
  ament_target_dependencies(test_behavior_manager rclcpp behaviortree_cpp_v3)
  target_link_libraries(test_behavior_manager tadeo_bt_nodes)
endif()

ament_package()