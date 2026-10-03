#!/bin/bash
# 用 scp 安全覆盖狗端 start_motion_control.sh（避免 tee+密码误写入文件）。
set -euo pipefail
DOG_HOST="${DOG_HOST:-192.168.168.168}"
DOG_USER="${DOG_USER:-firefly}"
DOG_PASS="${DOG_PASS:-firefly}"
SRC="$(cd "$(dirname "$0")" && pwd)/dog_start_motion_control.sh"

[[ -f "$SRC" ]] || { echo "missing $SRC" >&2; exit 1; }

ssh_dog() {
  sshpass -p "$DOG_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
    "${DOG_USER}@${DOG_HOST}" "$@"
}

echo "[install] backup + deploy $SRC -> ${DOG_HOST}:/opt/app_launch/start_motion_control.sh"
ssh_dog "echo ${DOG_PASS} | sudo -S cp -a /opt/app_launch/start_motion_control.sh /opt/app_launch/start_motion_control.sh.bak.\$(date +%Y%m%d%H%M%S) 2>/dev/null || true"

sshpass -p "$DOG_PASS" scp -o StrictHostKeyChecking=no "$SRC" \
  "${DOG_USER}@${DOG_HOST}:/tmp/start_motion_control.sh.new"

ssh_dog "echo ${DOG_PASS} | sudo -S install -m 755 /tmp/start_motion_control.sh.new /opt/app_launch/start_motion_control.sh && wc -c /opt/app_launch/start_motion_control.sh && grep -n flock /opt/app_launch/start_motion_control.sh && head -5 /opt/app_launch/start_motion_control.sh"
echo "[ok] dog start_motion_control.sh guarded"
