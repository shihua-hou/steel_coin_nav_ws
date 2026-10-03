#!/bin/bash
# 一次性拉起网页三件套（看门狗也会做同样的事；本脚本便于手动）
set -u
WS=/home/linaro/steel_coin_nav_ws
export ROS_HOME=/tmp/ros_home
mkdir -p /tmp/ros_home/log

source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/linaro/steel_coin_nav_ws/config/fastdds_udp_only.xml  # FastDDS 只走 UDP（SHM 端口会被误删导致进程互相收不到数据，2026-10-01）
source "$WS/install/setup.bash"

# 按真实进程 cmdline 判断（勿用裸 pgrep -f，易命中调试 shell）
_wd_alive() {
  local p c
  for p in $(pgrep -f 'scripts/web_console_watchdog.py' 2>/dev/null); do
    c=$(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null || true)
    case "$c" in
      *extglob*|*cursorsandbox*|*COMMAND_EXIT_CODE*) continue ;;
    esac
    return 0
  done
  return 1
}

if ! _wd_alive; then
  nohup python3 "$WS/scripts/web_console_watchdog.py" >/tmp/web_console_watchdog.log 2>&1 &
  echo "[web-console] watchdog pid=$!"
  sleep 3
else
  echo "[web-console] watchdog already running"
fi

WLAN_IP=$(ip -4 -o addr show dev wlan0 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -1)
UI_IP="${WLAN_IP:-$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -v '^127\.' | grep -v '^192\.168\.168\.' | head -1)}"
echo "[web-console] → http://${UI_IP:-<ip>}:8080/  (API :8090  rosbridge :9090)"
for p in 8080 8090 9090; do
  if timeout 1 bash -c "echo >/dev/tcp/127.0.0.1/$p" 2>/dev/null; then
    echo "  :$p OK"
  else
    echo "  :$p DOWN"
  fi
done
