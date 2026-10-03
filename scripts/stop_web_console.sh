#!/bin/bash
# 停止网页控制台相关进程（不影响 FAST-LIO / Nav2 / genisom_bridge）
set +e

_kill_match() {
  local needle="$1" sig="${2:-TERM}" p c
  for p in /proc/[0-9]*; do
    c=$(tr '\0' ' ' < "$p/cmdline" 2>/dev/null || true)
    [ -z "$c" ] && continue
    case "$c" in
      *extglob*|*cursorsandbox*|*COMMAND_EXIT_CODE*|*dump_bash_state*) continue ;;
    esac
    case "$c" in
      *"$needle"*) kill -s "$sig" "${p#/proc/}" 2>/dev/null || true ;;
    esac
  done
}

# 先停看门狗，避免立刻又拉起来
_kill_match 'scripts/web_console_watchdog.py' TERM
sleep 0.5
_kill_match 'http.server 8080' TERM
_kill_match 'web_http_nocache.py' TERM
_kill_match 'steel_coin_nav_ws/web_ui/web_ops_node.py' TERM
_kill_match 'lib/rosbridge_server/rosbridge_websocket' TERM
_kill_match 'rosbridge_websocket_launch' TERM
sleep 1
_kill_match 'scripts/web_console_watchdog.py' KILL
_kill_match 'http.server 8080' KILL
_kill_match 'web_http_nocache.py' KILL
_kill_match 'steel_coin_nav_ws/web_ui/web_ops_node.py' KILL
_kill_match 'lib/rosbridge_server/rosbridge_websocket' KILL
echo "[web-console] stopped"
exit 0
