#!/bin/bash
# 确保狗端只有 1 个 mc_ctrl（多实例会让 SDK checkConnect 一直超时）。
# 用法：ensure_dog_mc_ctrl.sh [--restart]
set -u
DOG_HOST="${DOG_HOST:-192.168.168.168}"
DOG_USER="${DOG_USER:-firefly}"
DOG_PASS="${DOG_PASS:-firefly}"
RESTART=0
[[ "${1:-}" == "--restart" ]] && RESTART=1

if ! command -v sshpass >/dev/null 2>&1; then
  echo "sshpass missing" >&2
  exit 2
fi

ssh_dog() {
  sshpass -p "$DOG_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
    -o PreferredAuthentications=password -o PubkeyAuthentication=no \
    "${DOG_USER}@${DOG_HOST}" "$@"
}

echo "[ensure_dog_mc_ctrl] check ${DOG_USER}@${DOG_HOST}"

# 统计并去重：保留挂在 start_motion_control 下的 mc_ctrl，其余杀掉
out=$(ssh_dog 'bash -s' <<EOF
echo ${DOG_PASS} | sudo -S true >/dev/null
# cmdline 通常是 "./mc_ctrl r"，不能只匹配绝对路径
PIDS=\$(pgrep -x mc_ctrl || pgrep -f '[./]mc_ctrl( |$)' || true)
N=\$(echo "\$PIDS" | wc -w)
echo "COUNT \$N"
if [ "\$N" -le 1 ]; then
  [ -n "\$PIDS" ] && echo "PROC \$PIDS"
  exit 0
fi
KEEP=""
for p in \$PIDS; do
  ppid=\$(awk '{print \$4}' /proc/\$p/stat 2>/dev/null || echo 0)
  pcmd=\$(tr '\\0' ' ' < /proc/\$ppid/cmdline 2>/dev/null || true)
  echo "PROC \$p ppid=\$ppid \$pcmd"
  case "\$pcmd" in *start_motion_control*) KEEP=\$p ;; esac
done
if [ -z "\$KEEP" ]; then
  KEEP=\$(echo "\$PIDS" | tr ' ' '\\n' | sort -n | head -1)
fi
echo "KEEP \$KEEP"
for p in \$PIDS; do
  if [ "\$p" != "\$KEEP" ]; then
    echo ${DOG_PASS} | sudo -S kill -9 "\$p" 2>/dev/null || true
    echo "KILLED \$p"
  fi
done
EOF
) || {
  echo "[ensure_dog_mc_ctrl] SSH failed" >&2
  echo "$out" >&2
  exit 1
}
echo "$out"

if [[ "$RESTART" -eq 1 ]]; then
  echo "[ensure_dog_mc_ctrl] restart motion control"
  ssh_dog 'bash -s' <<EOF
echo ${DOG_PASS} | sudo -S pkill -x mc_ctrl || true
echo ${DOG_PASS} | sudo -S pkill -f '[./]mc_ctrl( |$)' || true
echo ${DOG_PASS} | sudo -S pkill -f start_motion_control.sh || true
sleep 1
echo ${DOG_PASS} | sudo -S bash -c 'nohup bash /opt/app_launch/start_motion_control.sh >/tmp/mc_restart.log 2>&1 &'
sleep 3
pgrep -ax mc_ctrl || pgrep -af mc_ctrl || true
EOF
fi

echo "[ensure_dog_mc_ctrl] done"
