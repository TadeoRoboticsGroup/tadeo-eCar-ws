cmake_minimum_required(VERSION 3.8)
project(tadeo_ecar_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tadeo_ecar_msgs REQUIRED)

# Generate interfaces
set(srv_files
  "srv/SetRobotMode.srv"
  "srv/CalibrateWheels.srv"
  "srv/GetSystemStatus.srv"
  "srv/EmergencyStop.srv"
  "srv/SaveMap.srv"
  "srv/ExecuteBehavior.srv"
)

set(action_files
  "action/NavigateToGoal.action"
  "action/FollowPath.action"
  "action/Dock.action"
  "action/Calibrate.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs tadeo_ecar_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
