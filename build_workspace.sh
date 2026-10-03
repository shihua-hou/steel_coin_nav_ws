#!/bin/bash
# 一键编译「钢镚」整合工作区（主控 Ubuntu 22.04 aarch64 / ROS2 Humble）
# 用法: bash build_workspace.sh
set -e
cd "$(dirname "$0")"

source /opt/ros/humble/setup.bash

# 先装依赖（首次）
sudo apt-get update -y && sudo apt-get install -y \
  libpcl-dev libeigen3-dev libopenblas-dev \
  ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
  ros-humble-nav2-bringup ros-humble-nav2-amcl \
  ros-humble-nav2-msgs ros-humble-nav2-core ros-humble-nav2-costmap-2d \
  ros-humble-nav2-navfn-planner ros-humble-nav2-dwb-controller \
  ros-humble-nav2-behaviors ros-humble-nav2-bt-navigator \
  ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-waypoint-follower ros-humble-nav2-velocity-smoother \
  ros-humble-nav2-recoveries ros-humble-nav2-rviz-plugins \
  ros-humble-sensor-msgs-py ros-humble-geometry-msgs ros-humble-navigation2 || true

# 依赖顺序：livox(消息) → LIO → 桥 → 导航集成
colcon build \
  --packages-select livox_ros_driver2 FAST_LIO SUPER_LIO \
                    genisom_bridge lio_tf_bridge nav2_tools \
                    isaac_go2_nav2 \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
echo "=== BUILD OK ==="
echo "启动建图: ros2 launch isaac_go2_nav2 steel_coin_nav2.launch.py mode:=mapping"
echo "启动导航: ros2 launch isaac_go2_nav2 steel_coin_nav2.launch.py mode:=nav"
