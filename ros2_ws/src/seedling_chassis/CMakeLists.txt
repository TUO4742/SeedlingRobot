cmake_minimum_required(VERSION 3.16)
project(seedling_chassis)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(chassis_node
  src/chassis_node.cpp
  src/mecanum.cpp
  src/serial_port.cpp
)

target_include_directories(chassis_node PRIVATE include)

ament_target_dependencies(chassis_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

install(TARGETS chassis_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()
