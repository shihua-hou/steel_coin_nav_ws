#!/bin/bash
# 安装并启用开机自启：sudo bash scripts/install_web_console_service.sh
set -e
WS=/home/linaro/steel_coin_nav_ws
UNIT=dog-web-console.service
SRC="$WS/scripts/$UNIT"
DST="/etc/systemd/system/$UNIT"

if [ "$(id -u)" -ne 0 ]; then
  echo "请用 root 安装： sudo bash $0"
  exit 1
fi

chmod +x \
  "$WS/scripts/start_web_console.sh" \
  "$WS/scripts/stop_web_console.sh" \
  "$WS/scripts/web_console_watchdog.py" \
  "$WS/scripts/install_web_console_service.sh" \
  "$WS/start_ui.sh"

cp "$SRC" "$DST"
systemctl daemon-reload
systemctl enable "$UNIT"
systemctl stop "$UNIT" 2>/dev/null || true
# 清掉可能残留的旧三件套，交给 watchdog 统一拉起
bash "$WS/scripts/stop_web_console.sh" || true
sleep 1
systemctl start "$UNIT"
sleep 4
systemctl --no-pager --full status "$UNIT" || true
echo ""
echo "已启用常驻看门狗：$UNIT  (Restart=always, 工作区=$WS)"
echo "  状态: systemctl status $UNIT"
echo "  日志: journalctl -u $UNIT -f"
echo "  心跳: cat /tmp/web_console_heartbeat"
WLAN_IP=$(ip -4 -o addr show dev wlan0 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -1)
echo "  浏览器: http://${WLAN_IP:-<wlan-ip>}:8080/"
for p in 8080 8090 9090; do
  if timeout 1 bash -c "echo >/dev/tcp/127.0.0.1/$p" 2>/dev/null; then
    echo "  :$p OK"
  else
    echo "  :$p DOWN"
  fi
done
