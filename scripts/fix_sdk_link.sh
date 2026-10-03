#!/bin/bash
# 狗 SDK 链路一键诊断 / 恢复
#
#   bash scripts/fix_sdk_link.sh          诊断 + 按需修复（动狗端运控前会询问）
#   bash scripts/fix_sdk_link.sh --check  只诊断，不做任何改动
#   bash scripts/fix_sdk_link.sh -y       不询问，直接修复（无人值守用；狗可能趴下）
#
# 判断顺序（2026-10-01 实测教训）：
#   1) 先看 bridge 是否真的在读狗（/odom_dog、/robot_ctrl_mode 有数据）。
#      有数据 = 狗 SDK 本身是通的；此时网页/首页「SDK 未通」只是 web_ops / rosbridge
#      没收到 bridge 的话题（运行久了的进程与新拉起的 bridge 没建立 DDS 连接），
#      只需重启 web_ops + rosbridge —— 不要去重启狗端 mc_ctrl（会让狗趴下）。
#   2) bridge 读不到狗：先只重启 bridge；仍不行再重启狗端 mc_ctrl（狗会掉回趴下阻尼）。
WS=/home/linaro/steel_coin_nav_ws
API=http://127.0.0.1:8090
MODE=fix; YES=0
for a in "$@"; do
  case "$a" in --check) MODE=check;; -y|--yes) YES=1;; esac
done
# ROS 的 setup.bash 不兼容 set -u，加载完再开
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export ROS_HOME=/tmp/ros_home

ok(){ echo -e "  \e[32m✔\e[0m $*"; }
bad(){ echo -e "  \e[31m✘\e[0m $*"; }
step(){ echo -e "\n\e[1m$*\e[0m"; }

api_sdk(){ curl -s -m 3 "$API/api/battery" | python3 -c 'import sys,json
try:
    d=json.load(sys.stdin); pc=d.get("percentage")
    print("1" if d.get("sdk") else "0", d.get("age_sec"), ("%.0f%%" % (pc*100)) if isinstance(pc,(int,float)) else "-")
except Exception: print("x - -")'; }
dog_data(){ timeout 8 ros2 topic echo --once --no-daemon /robot_ctrl_mode std_msgs/msg/Int32 2>/dev/null | grep -m1 -oE "data: -?[0-9]+"; }
webops_pid(){ pgrep -f "^python3 $WS/web_ui/web_ops_node.py" | head -1; }
bridge_pid(){ pgrep -f "lib/genisom_bridge/genisom_bridge --ros-args" | head -1; }

restart_webops_rosbridge(){
  local wp; wp=$(webops_pid)
  echo "  → 重启 web_ops（pid ${wp:-无}，看门狗会自动拉起）"
  [ -n "$wp" ] && kill "$wp"
  for i in $(seq 1 40); do
    sleep 1
    local np; np=$(webops_pid)
    if [ -n "$np" ] && [ "$np" != "$wp" ] && curl -s -m 2 "$API/api/battery" >/dev/null; then
      echo "    web_ops 已起来（pid $np，${i}s）"; break
    fi
  done
  echo "  → 重启 rosbridge"
  curl -s -m 60 -X POST "$API/api/restart_rosbridge" -H 'Content-Type: application/json' -d '{}' >/dev/null
  sleep 5
}

wait_api_sdk(){  # $1 秒
  for i in $(seq 1 "$1"); do
    read -r s age pct < <(api_sdk)
    [ "$s" = "1" ] && return 0
    sleep 1
  done
  return 1
}

# ---------------- 诊断 ----------------
step "1. 网络"
if ping -c 3 -W 1 -i 0.3 192.168.168.168 >/dev/null 2>&1; then ok "狗 192.168.168.168 可 ping"; else bad "狗 ping 不通：检查网线 / 狗是否开机（以下步骤无意义）"; exit 1; fi

step "2. bridge 进程"
BP=$(bridge_pid)
if [ -n "$BP" ]; then ok "genisom_bridge 在运行（pid $BP）"; else bad "genisom_bridge 没在运行"; fi

step "3. bridge 是否在读狗（/robot_ctrl_mode）"
DD=$(dog_data)
if [ -n "$DD" ]; then ok "能读到狗状态：$DD（0=趴下阻尼 1=站立 18=运动）→ 狗 SDK 本身是通的"; DOG_OK=1
else bad "读不到狗状态 → 狗 SDK 真不通"; DOG_OK=0; fi

step "4. 网页后端 web_ops 看到的 SDK"
read -r S AGE PCT < <(api_sdk)
case "$S" in
  1) ok "web_ops: SDK 正常（电量 $PCT）";;
  0) bad "web_ops: SDK 未通（状态数据 ${AGE}s 未更新）";;
  *) bad "web_ops API :8090 连不上";;
esac

# ---------------- 结论 / 修复 ----------------
step "结论"
if [ "$DOG_OK" = "1" ] && [ "$S" = "1" ]; then
  ok "一切正常。若网页仍显示未通：刷新网页；仍不行再执行本脚本时会重启 rosbridge。"
  if [ "$MODE" = "fix" ]; then
    echo "  → 顺手重启 rosbridge（只影响网页数据，不碰狗）"
    curl -s -m 60 -X POST "$API/api/restart_rosbridge" -H 'Content-Type: application/json' -d '{}' >/dev/null
  fi
  exit 0
fi

if [ "$DOG_OK" = "1" ]; then
  echo "  狗 SDK 是通的，问题在 web_ops / rosbridge 收不到 bridge 的话题。"
  echo "  修复：重启 web_ops + rosbridge（不碰 bridge，不碰狗端运控，狗不会动）。"
  [ "$MODE" = "check" ] && exit 2
  restart_webops_rosbridge
  if wait_api_sdk 15; then ok "已恢复。网页刷新一下即可；狗趴着的话点「站立」。"; exit 0; fi
  bad "web_ops 仍看不到 SDK，请把以上输出发给开发排查。"; exit 3
fi

echo "  bridge 读不到狗。先只重启 bridge（狗不会动）。"
[ "$MODE" = "check" ] && exit 2
curl -s -m 40 -X POST "$API/api/ctrl" -H 'Content-Type: application/json' -d '{"mode":"SDK","force_restart":true}' >/dev/null
sleep 8
if [ -n "$(dog_data)" ]; then
  ok "bridge 重启后已能读到狗"
  wait_api_sdk 10 || restart_webops_rosbridge
  wait_api_sdk 15 && ok "已恢复。狗趴着的话点「站立」。" || bad "web_ops 仍看不到，请把输出发给开发"
  exit 0
fi

echo
echo "  仍读不到狗：需要重启狗端运控 mc_ctrl —— 狗会掉回「趴下阻尼」，之后要重新站立。"
if [ "$YES" != "1" ]; then
  read -r -p "  确认狗周围安全、可以趴下，继续？[y/N] " ans
  [ "$ans" = "y" ] || [ "$ans" = "Y" ] || { echo "  已取消。"; exit 4; }
fi
bash "$WS/scripts/ensure_dog_mc_ctrl.sh" --restart
echo "  等运控起来（约 40 s，起来前不要重启 bridge）…"
sleep 40
curl -s -m 40 -X POST "$API/api/ctrl" -H 'Content-Type: application/json' -d '{"mode":"SDK","force_restart":true}' >/dev/null
sleep 8
if [ -n "$(dog_data)" ]; then
  ok "bridge 已能读到狗"
  wait_api_sdk 10 || restart_webops_rosbridge
  wait_api_sdk 15 && ok "已恢复。网页刷新，点「站立」。" || bad "web_ops 仍看不到，请把输出发给开发"
else
  bad "仍读不到狗：给狗断电重启（或用遥控器 / APP 检查是否处于故障保护），再执行本脚本。"
  exit 5
fi
