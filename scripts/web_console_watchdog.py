#!/usr/bin/env python3
"""网页控制台常驻看门狗：保证 :8080 静态页 + :8090 web_ops + :9090 rosbridge 一直在。

- 每 HEARTBEAT_SEC 秒检查**端口**，挂了就拉起（勿仅靠模糊 pgrep）
- 写心跳文件 /tmp/web_console_heartbeat（供排查）
- 作为 systemd Type=simple 主进程；挂掉由 systemd Restart=always 拉回
"""
from __future__ import annotations

import os
os.environ.setdefault("FASTRTPS_DEFAULT_PROFILES_FILE", "/home/linaro/steel_coin_nav_ws/config/fastdds_udp_only.xml")  # FastDDS 只走 UDP（SHM 端口会被误删导致进程互相收不到数据，2026-10-01）；子进程全部继承
import signal
import socket
import subprocess
import sys
import time

WS = "/home/linaro/steel_coin_nav_ws"
ROS_SETUP = "source /opt/ros/humble/setup.bash; source /home/linaro/steel_coin_nav_ws/install/setup.bash"
ROS_HOME = "/tmp/ros_home"
HEARTBEAT_SEC = 5.0
HEARTBEAT_FILE = "/tmp/web_console_heartbeat"
LOG = "/tmp/web_console_watchdog.log"

_last_start: dict[str, float] = {}
MIN_RESTART_GAP = 8.0
_running = True


def log(msg: str) -> None:
    line = time.strftime("%Y-%m-%d %H:%M:%S") + " " + msg
    try:
        with open(LOG, "a") as f:
            f.write(line + "\n")
    except Exception:
        pass
    print(line, flush=True)


def cmdline(pid: int) -> str:
    try:
        return open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\0", b" ").decode()
    except Exception:
        return ""


def find_pids(*needles: str) -> list[int]:
    out = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        c = cmdline(int(name))
        if not c:
            continue
        # 跳过 Cursor/agent 包装 shell（命令行里常含同名子串，会误伤）
        if any(
            x in c
            for x in (
                "extglob",
                "web_console_watchdog",
                "dump_bash_state",
                "cursorsandbox",
                "COMMAND_EXIT_CODE",
            )
        ):
            continue
        if all(n in c for n in needles):
            out.append(int(name))
    return out


def port_open(port: int, host: str = "127.0.0.1") -> bool:
    try:
        with socket.create_connection((host, port), timeout=1.0):
            return True
    except OSError:
        return False


def bash_bg(cmd: str, log_path: str) -> None:
    env = dict(os.environ)
    env["ROS_HOME"] = ROS_HOME
    env["PYTHONUNBUFFERED"] = "1"
    full = f"{ROS_SETUP}; export ROS_HOME={ROS_HOME}; {cmd}"
    subprocess.Popen(
        ["bash", "-lc", full],
        stdout=open(log_path, "a"),
        stderr=subprocess.STDOUT,
        env=env,
        start_new_session=True,
    )


def can_start(key: str) -> bool:
    now = time.time()
    last = _last_start.get(key, 0.0)
    if now - last < MIN_RESTART_GAP:
        return False
    _last_start[key] = now
    return True


def ensure_http() -> bool:
    if port_open(8080):
        return True
    if not can_start("http"):
        return False
    for p in find_pids("http.server", "8080") + find_pids("web_http_nocache.py"):
        try:
            os.kill(p, signal.SIGKILL)
        except Exception:
            pass
    time.sleep(0.3)
    log("restart static HTTP :8080")
    nocache = f"{WS}/scripts/web_http_nocache.py"
    if os.path.isfile(nocache):
        bash_bg(f"exec python3 {nocache}", "/tmp/web_http.log")
    else:
        bash_bg(
            f"exec python3 -m http.server 8080 --directory {WS}/web_ui",
            "/tmp/web_http.log",
        )
    return False


def ensure_webops() -> bool:
    if port_open(8090):
        return True
    if not can_start("webops"):
        return False
    for p in find_pids("steel_coin_nav_ws/web_ui/web_ops_node.py"):
        try:
            os.kill(p, signal.SIGKILL)
        except Exception:
            pass
    time.sleep(0.3)
    log("restart web_ops :8090")
    bash_bg(f"exec python3 {WS}/web_ui/web_ops_node.py", "/tmp/webops_new.log")
    return False


def ensure_rosbridge() -> bool:
    if port_open(9090):
        return True
    if not can_start("rosbridge"):
        return False
    for p in find_pids("lib/rosbridge_server/rosbridge_websocket"):
        try:
            os.kill(p, signal.SIGKILL)
        except Exception:
            pass
    for p in find_pids("rosbridge_websocket_launch"):
        try:
            os.kill(p, signal.SIGKILL)
        except Exception:
            pass
    time.sleep(0.5)
    log("restart rosbridge :9090")
    # 手机浏览器对 bson/cbor 压缩支持差，压缩开会连上又立刻断开，页面一直「连接中」
    bash_bg(
        "exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml "
        "use_compression:=false",
        "/tmp/rosbridge_new.log",
    )
    return False


def write_heartbeat(ok_http: bool, ok_api: bool, ok_rb: bool) -> None:
    try:
        with open(HEARTBEAT_FILE, "w") as f:
            f.write(
                f"ts={time.time():.0f} http8080={int(ok_http)} "
                f"api8090={int(ok_api)} rosbridge9090={int(ok_rb)}\n"
            )
    except Exception:
        pass


def on_signal(signum, _frame):
    global _running
    log(f"signal {signum}, shutting down watchdog (children kept unless ExecStop)")
    _running = False


def main() -> int:
    os.makedirs(ROS_HOME + "/log", exist_ok=True)
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    log("web_console_watchdog start (steel_coin_nav_ws)")

    ensure_rosbridge()
    ensure_webops()
    ensure_http()
    time.sleep(2.0)

    while _running:
        ensure_rosbridge()
        ensure_webops()
        ensure_http()
        ok_http, ok_api, ok_rb = port_open(8080), port_open(8090), port_open(9090)
        write_heartbeat(ok_http, ok_api, ok_rb)
        if not (ok_http and ok_api and ok_rb):
            log(f"health http={ok_http} api={ok_api} rosbridge={ok_rb}")
        deadline = time.time() + HEARTBEAT_SEC
        while _running and time.time() < deadline:
            time.sleep(0.2)
    log("web_console_watchdog exit")
    return 0


if __name__ == "__main__":
    sys.exit(main())
