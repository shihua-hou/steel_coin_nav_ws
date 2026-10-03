#!/bin/bash
# start motion control — single-instance guarded (steel_coin_nav_ws)
# 多个 mc_ctrl 会让主机 SDK checkConnect 一直超时，网页显示「狗 SDK 未连通」。

echo "start motion control"

SHM_FILE="/dev/shm/spline_shm"
while true; do
    if [ -e "$SHM_FILE" ]; then
        echo "共享内存文件 $SHM_FILE 已存在。"
        break
    else
        echo "共享内存文件 $SHM_FILE 不存在，等待 1 秒后重试..."
        sleep 1
    fi
done

echo "共享内存文件已准备好，可以执行后续操作。"

LOCK_FILE=/var/lock/mc_ctrl.start.lock
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
    echo "mc_ctrl 已在启动/运行（flock 占用），本实例退出"
    pgrep -x mc_ctrl >/dev/null && exit 0
    exit 1
fi

if pgrep -x mc_ctrl >/dev/null 2>&1; then
    echo "发现已有 mc_ctrl，先清理孤儿再拉起"
    pkill -x mc_ctrl 2>/dev/null || true
    sleep 0.8
fi

sudo ifconfig lo multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo 2>/dev/null || true

export LD_LIBRARY_PATH=/opt/export/mc/bin
export ROBOT_TYPE=XG
export SDK_CLIENT_IP="192.168.168.168"

cd /opt/export/mc/bin && taskset -c 7 ./mc_ctrl r
