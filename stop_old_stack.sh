#!/bin/bash
# stop_old_stack.sh — 停止 robot_ws 全部 ROS/UI 进程（不删任何文件，可随时用旧脚本重启）
# 在跑新工程之前必须执行一次（否则两套 livox/genisom/Nav2 抢雷达、狗、话题）
set -e

echo "[1/6] 停止传感器/建图/导航节点..."
pkill -f livox_ros_driver2_node || true
pkill -f super_lio_node || true
pkill -f map_building_node || true
pkill -f nav_scan_node || true
pkill -f nav2_container || true
pkill -f auto_relocalize || true
pkill -f lio_tf_bridge || true
pkill -f genisom_bridge || true
pkill -f 'ros2 launch nav2_bringup' || true
pkill -f 'ros2 launch super_lio' || true
pkill -f 'ros2 launch livox_ros_driver2' || true
pkill -f 'ros2 launch genisom_bridge' || true
pkill -f 'ros2 launch lio_tf_bridge' || true

echo "[2/6] 停止 UI 栈（网页控制台）..."
pkill -f web_ops_node.py || true
pkill -f rosbridge_websocket || true
pkill -f web_console_watchdog.py || true
pkill -f web_http_nocache.py || true

echo "[3/6] 重置 ros2 daemon..."
ros2 daemon stop 2>/dev/null || true
sleep 2

echo "[4/6] 确认残留（应只有本命令自身）..."
pgrep -af 'livox|super_lio|nav2|genisom|rosbridge|web_ops|nav_scan|map_building|auto_relocalize|lio_tf' | grep -v grep || echo "  ✅ 全部已停止"
echo ""
echo "robot_ws 栈已停止（文件未动）。要恢复旧工程：cd /home/linaro/robot_ws && bash start_all.sh"
