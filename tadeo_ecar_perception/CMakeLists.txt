cmake_minimum_required(VERSION 3.8)
project(tadeo_ecar_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O1 -fno-exceptions)
endif()

# Reduce memory usage during compilation
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g0 -fno-rtti -pipe")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g0 -pipe")

# Limit parallel compilation to reduce memory pressure
set(CMAKE_BUILD_PARALLEL_LEVEL 1)

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
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tadeo_ecar_msgs REQUIRED)

# Include directories
include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

# Camera Processing Node
add_executable(camera_processor_node src/camera_processor_node.cpp)
ament_target_dependencies(camera_processor_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  image_transport
  OpenCV
  tadeo_ecar_msgs
)

# LiDAR Processing Node
add_executable(lidar_processor_node src/lidar_processor_node.cpp)
ament_target_dependencies(lidar_processor_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  PCL
  pcl_conversions
  tadeo_ecar_msgs
)
target_link_libraries(lidar_processor_node ${PCL_LIBRARIES})

# IMU Processing Node  
add_executable(imu_processor_node src/imu_processor_node.cpp)
ament_target_dependencies(imu_processor_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tadeo_ecar_msgs
)

# Sensor Fusion Node
add_executable(sensor_fusion_node src/sensor_fusion_node.cpp)
ament_target_dependencies(sensor_fusion_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  PCL
  pcl_conversions
  tadeo_ecar_msgs
)
target_link_libraries(sensor_fusion_node ${PCL_LIBRARIES})

# Install executables
install(TARGETS
  camera_processor_node
  lidar_processor_node
  imu_processor_node
  sensor_fusion_node
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