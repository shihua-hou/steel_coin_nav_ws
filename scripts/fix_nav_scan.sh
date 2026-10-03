#!/bin/bash
# Fix Mid360 /scan: leveled height band (pcd2pgm-matched) + base_footprint for Nav2.
set -x
WS=/home/linaro/robot_ws
exec > /tmp/fix_nav_scan.out 2>&1
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

pkill -f pointcloud_to_laserscan_node || true
pkill -f nav_scan_node.py || true
sleep 1
bash "$WS/start_scan_node.sh"
sleep 3
tail -20 /tmp/scan_node.log

# Restart reloc with new binary
sleep 1
echo reloc=$!
sleep 2

# Restart web_ops
pkill -f 'web_ui/web_ops_node.py' || true
sleep 1
nohup bash -c "source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && python3 $WS/web_ui/web_ops_node.py" \
  > /tmp/webops.log 2>&1 &
echo webops=$!
sleep 2

# Restart Nav2 with base_footprint params
MAP="$WS/maps/map_live.yaml"
[ -f "$MAP" ] || MAP=$(ls -t "$WS"/maps/*.yaml 2>/dev/null | head -1)
[ -n "$MAP" ] || MAP="$WS/maps/map.yaml"
echo "MAP=$MAP"
bash "$WS/nav2_start.sh" "$MAP"

sleep 8
echo '==== PROCS ===='
echo '==== SCAN LOG ===='
tail -30 /tmp/scan_node.log || true
echo '==== VERIFY SCAN ===='
timeout 6 ros2 topic echo /scan --once | head -20 || true
timeout 4 ros2 topic hz /scan || true
timeout 3 ros2 run tf2_ros tf2_echo odom base_footprint | head -15 || true
echo DONE
