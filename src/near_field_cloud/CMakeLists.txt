cmake_minimum_required(VERSION 3.8)
project(near_field_cloud)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_executable(near_field_cloud_node src/near_field_cloud_node.cpp)
ament_target_dependencies(near_field_cloud_node rclcpp sensor_msgs livox_ros_driver2)

install(TARGETS near_field_cloud_node DESTINATION lib/${PROJECT_NAME})

ament_package()
