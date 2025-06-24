cmake_minimum_required(VERSION 3.8)
project(tadeo_ecar_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install configuration files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
