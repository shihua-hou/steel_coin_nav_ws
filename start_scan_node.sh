#!/bin/bash
# start_scan_node.sh — /lio/cloud_world → /scan（nav_scan_node）
# 与建图页 3D 点云同源：按标定重力矢量校平，切 z=[0.10,1.0]，发布到 base_footprint
source /opt/ros/humble/setup.bash
source /home/linaro/steel_coin_nav_ws/install/setup.bash
pkill -f pointcloud_to_laserscan_node 2>/dev/null || true
pkill -f nav_scan_node.py 2>/dev/null || true
pkill -f 'nav2_tools.nav_scan_node' 2>/dev/null || true
sleep 0.3
nohup python3 -u /home/linaro/steel_coin_nav_ws/src/nav2_tools/nav2_tools/nav_scan_node.py \
  --ros-args \
  -p z_min:=0.10 -p z_max:=1.00 \
  -p range_min:=0.40 -p range_max:=8.0 \
  -p angle_increment:=0.01 \
  > /tmp/scan_node.log 2>&1 &
echo "nav_scan_node pid=$!"
