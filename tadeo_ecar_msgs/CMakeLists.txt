cmake_minimum_required(VERSION 3.8)
project(tadeo_ecar_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Generate interfaces
set(msg_files
  "msg/RobotStatus.msg"
  "msg/WheelStates.msg"
  "msg/SystemHealth.msg"
  "msg/SafetyStatus.msg"
  "msg/NavigationState.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
