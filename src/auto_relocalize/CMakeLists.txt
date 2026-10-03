cmake_minimum_required(VERSION 3.8)
project(auto_relocalize)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

# 全局似然场搜索方案(移植自 wheeltec_fastlio), 不再需要 PCL/ICP。
add_executable(auto_relocalize src/auto_relocalize.cpp)
ament_target_dependencies(auto_relocalize
  rclcpp nav_msgs sensor_msgs geometry_msgs std_msgs std_srvs tf2_ros)

install(TARGETS auto_relocalize DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
