#!/bin/bash
# 修复导航：强制重启 LIO/TF（避免僵尸进程无 /lio/odom），校平 /scan，贴墙重定位。
# 必须在板子本机跑，不要用 Cursor Agent 终端。
set -e
WS=/home/linaro/robot_ws
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export ROS_HOME=/tmp/ros_home
mkdir -p /tmp/ros_home/log

MAP="${1:-map_live.yaml}"
MAP_PATH="$WS/maps/$MAP"
[ -f "$MAP_PATH" ] || MAP_PATH="$WS/maps/map_live.yaml"
[ -f "$MAP_PATH" ] || MAP_PATH="$WS/maps/map.yaml"
echo "[nav] map=$MAP_PATH"

echo "[nav] kill Nav2 / reloc / scan..."
pkill -9 -f component_container_isolated 2>/dev/null || true
pkill -9 -f bringup_launch.py 2>/dev/null || true
pkill -9 -f 'nav2_bringup' 2>/dev/null || true
pkill -9 -f auto_relocalize 2>/dev/null || true
pkill -9 -f pointcloud_to_laserscan_node 2>/dev/null || true
pkill -9 -f nav_scan_node.py 2>/dev/null || true
sleep 1

echo "[nav] FORCE restart Super-LIO + lio_tf (clear stale TF / missing odom)..."
pkill -9 -f 'super_lio Livox_mid360' 2>/dev/null || true
pkill -9 -f super_lio_node 2>/dev/null || true
pkill -9 -f 'lio_tf_bridge bridge.launch' 2>/dev/null || true
pkill -9 -f 'lio_tf_bridge/lio_tf_bridge' 2>/dev/null || true
pkill -9 -f imu_to_base_link_static 2>/dev/null || true
sleep 2
bash "$WS/start_all.sh" localization
sleep 5

echo "[nav] check /lio/odom /lio/cloud_world / TF..."
for i in 1 2 3 4 5 6 7 8; do
  if timeout 2 ros2 topic hz /lio/odom 2>&1 | grep -q 'average rate'; then
    break
  fi
  echo "  wait lio/odom ($i)..."
  sleep 1
done
timeout 5 ros2 topic hz /lio/odom 2>&1 | head -4 || echo "WARN: no /lio/odom"
timeout 5 ros2 topic hz /lio/cloud_world 2>&1 | head -4 || echo "WARN: no /lio/cloud_world"
# 等到 odom→base_footprint（lio_tf 收到里程计后才会发）
for i in 1 2 3 4 5 6 7 8 9 10; do
  if timeout 2 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | grep -q 'Translation'; then
    echo "  TF odom→base_footprint OK"
    break
  fi
  echo "  wait TF odom→base_footprint ($i)..."
  sleep 1
done

echo "[nav] start leveled /scan (z=0.15~1.0m)..."
bash "$WS/start_scan_node.sh"
sleep 3
timeout 6 ros2 topic echo /scan --once 2>&1 | head -14 || echo "WARN: no /scan"
timeout 4 ros2 topic hz /scan 2>&1 | head -5 || true
timeout 4 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | head -12 || true

# web_ops
ps -eo pid,args | awk '/python3 .*web_ui\/web_ops_node\.py/ && !/awk/ {print $1}' | while read -r p; do
  kill "$p" 2>/dev/null || true
done
sleep 1
nohup python3 "$WS/web_ui/web_ops_node.py" >/tmp/webops.log 2>&1 &
echo "web_ops pid=$!"
sleep 2

PARAMS="$WS/src/nav2_tools/nav2_params.yaml"
echo "[nav] start Nav2..."
nohup ros2 launch nav2_bringup bringup_launch.py \
  params_file:="$PARAMS" map:="$MAP_PATH" use_sim_time:=False autostart:=True \
  >/tmp/nav2.log 2>&1 &
echo "nav2 pid=$!"
sleep 6

echo "[nav] start Nav2..."
nohup ros2 launch nav2_bringup bringup_launch.py \
  params_file:="$PARAMS" map:="$MAP_PATH" use_sim_time:=False autostart:=True \
  >/tmp/nav2.log 2>&1 &
echo "nav2 pid=$!"
sleep 6

echo "[nav] containers=$(pgrep -c -f component_container_isolated || echo 0) super_lio=$(pgrep -c -f super_lio_node || echo 0) scan=$(pgrep -c -f nav_scan_node || echo 0)"
echo "OK — 刷新网页后："
echo "  1) 确认 Nav2 / LIO / 扫描都在跑"
echo "  2) 点「设初始位姿」对准位置与朝向（Go2W：/initialpose）"
echo "  3) 「确认定位」后「设目标」"
