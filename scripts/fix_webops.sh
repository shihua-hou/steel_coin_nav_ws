#!/bin/bash
# 仅拉起 / 重启 web_ops HTTP API (:8090)。在「真机主机」上跑，不要在 Cursor 沙箱里跑。
set -e
WS=/home/linaro/robot_ws
export ROS_HOME=/tmp/ros_home
mkdir -p /tmp/ros_home/log

# 清掉所有 web_ops（含误跑进沙箱网卡命名空间、主机访问不到的僵尸进程）
ps -eo pid,args | awk '/python3 .*web_ui\/web_ops_node\.py/ && !/awk/ {print $1}' | while read -r p; do
  kill "$p" 2>/dev/null || true
done
sleep 1
ps -eo pid,args | awk '/python3 .*web_ui\/web_ops_node\.py/ && !/awk/ {print $1}' | while read -r p; do
  kill -9 "$p" 2>/dev/null || true
done
sleep 0.5

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
nohup python3 "$WS/web_ui/web_ops_node.py" >/tmp/webops.log 2>&1 &
echo "web_ops pid=$!"
sleep 2
if curl -sf --connect-timeout 2 http://127.0.0.1:8090/api/maps >/dev/null; then
  echo "OK :8090 /api/maps"
  curl -s http://127.0.0.1:8090/api/maps
  echo
else
  echo "FAIL: 8090 still down — see /tmp/webops.log"
  tail -30 /tmp/webops.log || true
  exit 1
fi
