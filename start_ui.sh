#!/bin/bash
# start_ui.sh — 新工程控制台界面
#   静态页(:8080) + web_ops API(:8090) + rosbridge(:9090)
#
# 注意：不要用 pgrep -f 'rosbridge_websocket' / 'http.server 8080' 判断是否已启动——
# Cursor/agent 调试命令行里常带同名字符串，会误判「已在跑」导致端口实际 DOWN。
set -e
WS_NEW=/home/linaro/steel_coin_nav_ws
UI_DIR=/home/linaro/steel_coin_nav_ws/web_ui
export ROS_HOME=/tmp/ros_home
mkdir -p /tmp/ros_home/log

source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/linaro/steel_coin_nav_ws/config/fastdds_udp_only.xml  # FastDDS 只走 UDP（SHM 端口会被误删导致进程互相收不到数据，2026-10-01）
source "$WS_NEW/install/setup.bash"

port_up() {
  timeout 1 bash -c "echo >/dev/tcp/127.0.0.1/$1" >/dev/null 2>&1
}

if ! port_up 9090; then
  nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
    use_compression:=true > /tmp/rosbridge_new.log 2>&1 &
  echo "rosbridge :9090 pid=$!"
  sleep 2
else
  echo "rosbridge :9090 already up"
fi

if ! port_up 8090; then
  nohup python3 "$UI_DIR/web_ops_node.py" > /tmp/webops_new.log 2>&1 &
  echo "web_ops :8090 pid=$!"
  sleep 2
else
  echo "web_ops :8090 already up"
fi

if ! port_up 8080; then
  nohup python3 -m http.server 8080 --directory "$UI_DIR" > /tmp/http_new.log 2>&1 &
  echo "静态页 :8080 pid=$!"
  sleep 1
else
  echo "静态页 :8080 already up"
fi

WLAN_IP=$(ip -4 -o addr show dev wlan0 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -1)
UI_IP="${WLAN_IP:-$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -v '^127\.' | grep -v '^192\.168\.168\.' | head -1)}"

echo ""
echo "控制台就绪：浏览器打开 http://${UI_IP:-<主控IP>}:8080"
for p in 8080 8090 9090; do
  if port_up "$p"; then
    echo "  :$p OK"
  else
    echo "  :$p DOWN"
  fi
done
