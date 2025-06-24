cmake_minimum_required(VERSION 3.8)
project(tadeo_ecar_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Método más explícito - instalar cada directorio individualmente
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME}) 
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

ament_package()