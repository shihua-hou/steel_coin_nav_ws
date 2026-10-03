#!/usr/bin/env python3
"""web_ops_node - 网页上位机后端代理节点（v3：ROS + 管理 HTTP API）

ROS 订阅(前端 -> 主控):
  /web/nav_cmd     geometry_msgs/Twist   linear.x/y=目标 angular.z=yaw(rad) -> Nav2
  /web/nav_cancel  std_msgs/Empty        取消当前导航目标
  /web/init_pose   geometry_msgs/Twist   linear.x=x linear.y=y angular.z=yaw(rad) -> /initialpose
  /web/start_nav2  std_msgs/Empty        启动 Nav2（默认 map.yaml）
  /web/stop_nav2   std_msgs/Empty        停止 Nav2
  /web/mapping     std_msgs/Empty        确保 Super-LIO 建图链运行

ROS 发布(主控 -> 前端):
  /web/nav_status  std_msgs/String  idle|navigating|reached|aborted|canceled[:detail]
  /web/sys_status  std_msgs/String  JSON services + nav
  /web/robot_pose  geometry_msgs/PoseStamped  map→base_footprint（网页画激光/机身用）

HTTP API (端口 8090, CORS 开放):
  GET  /api/maps
  GET  /api/map/pgm?name=x.pgm
  GET  /api/map/preview?name=x.pgm   -> PNG 缩略图（浏览器可直接显示）
  POST /api/map/edit
  POST /api/map/delete
  POST /api/mapping/save     -> {name} 保存建图（可先 stop 再 save，或运行中直接 save）
  POST /api/mapping/stop     -> 停止建图并落盘 PCD（不转换）
  POST /api/mapping/discard  -> 停止建图并丢弃（不写地图）
  POST /api/nav2/start       -> {map} 启动定位栈 + 单例 Nav2（Go2W：初姿用 /initialpose）
  POST /api/nav2/stop
  POST /api/nav2/accept_pose -> 确认当前位姿，放行导航目标
  POST /api/nav2/reload_map  -> {map} 热换图（换图后请重新设初姿）
  GET  /api/sysinfo
  GET  /api/log?file=xxx&n=50
  GET  /api/video/mjpeg      -> RTSP→MJPEG 代理（浏览器可直接 <img>）
  GET  /api/video/snapshot   -> 单帧 JPEG
  POST /api/restart_rosbridge
  POST /api/ctrl            -> {mode: APP|SDK} 停止/启动 genisom_bridge
  POST /api/cmd_vel         -> {vx,vy,wz} 网页遥控（绕过 rosbridge）
  POST /api/video/stop      -> 停止 ffmpeg 图传拉流（省 CPU）
"""
import array
import base64
import json
import math
import os
import re
import shutil
import signal
import struct
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
from urllib.parse import unquote
from PIL import Image
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import Twist, Quaternion, PoseWithCovarianceStamped, PoseStamped, Pose
from std_msgs.msg import String, Empty, Float32MultiArray
from sensor_msgs.msg import LaserScan, BatteryState, NavSatFix
from nav2_msgs.action import FollowPath, NavigateToPose, NavigateThroughPoses
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from rclpy.time import Time
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

ROS_SETUP = '/opt/ros/humble/setup.bash'
WS_SETUP = '/home/linaro/steel_coin_nav_ws/install/setup.bash'
MAPS_DIR = '/home/linaro/steel_coin_nav_ws/maps'
NAV_PICK_FILE = os.path.join(MAPS_DIR, '.selected_nav')
SUPERLIO_PCD = '/home/linaro/steel_coin_nav_ws/src/SUPER_LIO/super_lio/map/map.pcd'
SUPERLIO_PCD_DIR = '/home/linaro/steel_coin_nav_ws/src/SUPER_LIO/super_lio/map'
PCD2PGM = '/home/linaro/steel_coin_nav_ws/src/nav2_tools/nav2_tools/pcd2pgm.py'
if not os.path.isfile(PCD2PGM):
    PCD2PGM = '/home/linaro/steel_coin_nav_ws/src/nav2_tools/pcd2pgm.py'
NAV2_PARAMS = '/home/linaro/steel_coin_nav_ws/src/isaac_go2_nav2/config/steel_coin_nav2_params.yaml'
if not os.path.isfile(NAV2_PARAMS):
    NAV2_PARAMS = '/home/linaro/steel_coin_nav_ws/src/nav2_tools/nav2_params.yaml'
START_ALL = '/home/linaro/steel_coin_nav_ws/start_quad.sh'
START_SCAN = '/home/linaro/steel_coin_nav_ws/start_scan_node.sh'
# 狗身广角相机（AgiBot 文档）：有线网段 RTSP H.264
DOG_RTSP = os.environ.get('DOG_RTSP', 'rtsp://192.168.168.168:8554/test')
DOG_FRAME_JPG = '/tmp/dog_live.jpg'
# 默认档（无负载时的高清）；实际以 /api/video/profile 为准
VIDEO_WIDTH = int(os.environ.get('DOG_VIDEO_WIDTH', '1280'))
VIDEO_FPS = int(os.environ.get('DOG_VIDEO_FPS', '15'))
VIDEO_Q = int(os.environ.get('DOG_VIDEO_Q', '3'))
VIDEO_IDLE_SEC = float(os.environ.get('DOG_VIDEO_IDLE_SEC', '45'))
# smooth=导航/建图时保流畅；balanced/best=空闲时按网络探测
VIDEO_PROFILES = {
    'smooth': {'width': 640, 'fps': 12, 'q': 5, 'label': '流畅'},
    'balanced': {'width': 960, 'fps': 12, 'q': 4, 'label': '均衡'},
    'best': {'width': 1280, 'fps': 15, 'q': 3, 'label': '高清'},
}
_video_feeder = {'proc': None, 'lock': threading.Lock(), 'last_use': 0.0}
_video_profile = 'best'
_video_cfg = dict(VIDEO_PROFILES['best'])
# 运控页右上角的实际出帧率：按最近约 1 秒里真正写出的 MJPEG 帧计
_mjpeg_meter = {'t': 0.0, 'n': 0, 'fps': 0.0}
# 遥控优先：stop 后一段时间内禁止再拉起 ffmpeg
_video_block_until = 0.0


def _ffmpeg_dog_live_pids(max_age=0):
    """所有写 /tmp/dog_live.jpg 的 ffmpeg（含 web_ops 重启后的孤儿）。默认实时扫描（只在杀/拉 ffmpeg 时调）。"""
    out = []
    for pid, cmd in _proc_table(max_age):
        if 'ffmpeg' in cmd and 'dog_live.jpg' in cmd:
            out.append(pid)
    return out


def stop_video_feeder(block_sec=20.0):
    global _video_block_until
    with _video_feeder['lock']:
        if block_sec and block_sec > 0:
            _video_block_until = max(_video_block_until, time.monotonic() + float(block_sec))
        proc = _video_feeder['proc']
        _video_feeder['proc'] = None
        for pid in _ffmpeg_dog_live_pids():
            try:
                _kill(pid, signal.SIGTERM)
            except Exception:
                pass
        try:
            if proc is not None and proc.poll() is None:
                _killpg(proc.pid, signal.SIGTERM)
        except Exception:
            pass
    time.sleep(0.05)
    for pid in _ffmpeg_dog_live_pids():
        try:
            _kill(pid, signal.SIGKILL)
        except Exception:
            pass


def video_busy_profile():
    """占核时图传强制档位：建图（map_building / 建图会话 / Super-LIO）→ 'smooth'；
    Nav2 在跑 → 'balanced'；只剩后台 FAST-LIO，或都没跑 → None（按客户端/网速选，可到高清）。
    关导航后定位链按设计留着。FAST-LIO 约半核，8 核上再加高清软解（约 1 核）仍有余量；
    Nav2（代价图 / AMCL / 规划）叠上才维持均衡。前端运控页 motionVideoCap() 与此对齐。"""
    if proc_alive('map_building_node'):
        return 'smooth'
    try:
        if _mapping_session:
            return 'smooth'
    except NameError:
        pass
    # Super-LIO 很重（建图用），运控只能走流畅档
    if proc_alive('super_lio_node'):
        return 'smooth'
    if proc_alive('component_container_isolated'):
        return 'balanced'
    return None


def note_mjpeg_frame():
    now = time.monotonic()
    st = _mjpeg_meter
    if st['t'] <= 0 or (now - st['t']) >= 1.0:
        dt = (now - st['t']) if st['t'] > 0 else 1.0
        st['fps'] = (st['n'] / dt) if st['t'] > 0 else 0.0
        st['n'] = 0
        st['t'] = now
    st['n'] += 1


def mjpeg_fps():
    """最近一秒实际送给浏览器的帧率。超过 2 秒没出帧则记 0。"""
    now = time.monotonic()
    st = _mjpeg_meter
    if st['t'] <= 0 or (now - st['t']) > 2.0:
        return 0.0
    dt = now - st['t']
    if dt >= 0.4 and st['n'] > 0:
        return st['n'] / dt
    return float(st['fps'] or 0.0)


def video_meter():
    cfg = _video_cfg or VIDEO_PROFILES['best']
    return {
        'ok': True,
        'profile': _video_profile,
        'label': VIDEO_PROFILES.get(_video_profile, {}).get('label', ''),
        'width': int(cfg.get('width', VIDEO_WIDTH)),
        'fps_set': int(cfg.get('fps', VIDEO_FPS)),
        'fps': round(mjpeg_fps(), 1),
    }


def video_stack_busy():
    """建图或 Nav2 占核时，禁止高清图传把网页拖死。后台只剩 FAST-LIO 不算忙。"""
    return video_busy_profile() is not None


def set_video_profile(name='best', rtt_ms=None, start_feeder=True):
    """切换图传档位。smooth=流畅；balanced=均衡；best/auto=按 rtt_ms 选最优。
    start_feeder=False：档位变了只在 ffmpeg 本来就在跑时重启（定时器用，没人看时不白拉 ffmpeg）。"""
    global _video_profile, _video_cfg
    raw = (name or 'best').strip().lower()
    if raw in ('fluent', 'fluid', 'low', 'lite'):
        raw = 'smooth'
    if raw in ('high', 'hd', 'quality'):
        raw = 'best'
    if raw in ('mid', 'medium', 'normal'):
        raw = 'balanced'
    forced = False
    busy_prof = video_busy_profile()
    if busy_prof:
        # 负载中强制档位（建图=流畅，Nav2=均衡），忽略客户端要高清
        raw = busy_prof
        forced = True
    elif raw in ('auto', 'best'):
        pick = 'best'
        if rtt_ms is not None:
            try:
                rtt = float(rtt_ms)
            except (TypeError, ValueError):
                rtt = None
            if rtt is not None:
                if rtt > 1100:
                    pick = 'smooth'
                elif rtt > 600:
                    pick = 'balanced'
                else:
                    pick = 'best'
        if raw == 'auto':
            raw = pick
        else:
            raw = 'balanced' if pick == 'smooth' else pick
    if raw not in VIDEO_PROFILES:
        raw = 'best'
    new_cfg = dict(VIDEO_PROFILES[raw])
    changed = (raw != _video_profile) or (new_cfg != _video_cfg)
    _video_profile = raw
    _video_cfg = new_cfg
    restarted = False
    if changed:
        proc = _video_feeder['proc']
        was_running = proc is not None and proc.poll() is None
        stop_video_feeder(block_sec=0)
        if start_feeder or was_running:
            restarted = bool(ensure_video_feeder())
    return {
        'ok': True,
        'profile': _video_profile,
        'label': VIDEO_PROFILES[_video_profile]['label'],
        'width': _video_cfg['width'],
        'fps': _video_cfg['fps'],
        'q': _video_cfg['q'],
        'restarted': bool(changed),
        'feeder': restarted,
        'rtt_ms': rtt_ms,
        'forced': forced,
        'forced_smooth': forced and _video_profile == 'smooth',
    }


def ensure_video_feeder():
    """Keep one ffmpeg writing latest JPEG；仅在有人看视频且未被遥控暂停时。"""
    global _video_block_until
    with _video_feeder['lock']:
        if time.monotonic() < _video_block_until:
            return False
        _video_feeder['last_use'] = time.monotonic()
        proc = _video_feeder['proc']
        # MJPEG 每帧都会调到这里：自己的 ffmpeg 还活着且 5 s 内核对过「只有它一个」就直接返回，
        # 不再每帧整扫 /proc（2026-10-02：开着图传时这一项占 web_ops CPU 的 1/5 以上）
        if (proc is not None and proc.poll() is None
                and time.monotonic() - _video_feeder.get('verified', 0.0) < 5.0):
            return True
        live = _ffmpeg_dog_live_pids()
        if proc is not None and proc.poll() is None and len(live) == 1 and proc.pid in live:
            _video_feeder['verified'] = time.monotonic()
            return True
        # 清理全部残留，保证只留一个
        for pid in live:
            try:
                _kill(pid, signal.SIGTERM)
            except Exception:
                pass
        time.sleep(0.2)
        for pid in _ffmpeg_dog_live_pids():
            try:
                _kill(pid, signal.SIGKILL)
            except Exception:
                pass
        try:
            if proc is not None and proc.poll() is None:
                _killpg(proc.pid, signal.SIGKILL)
        except Exception:
            pass
        try:
            if os.path.isfile(DOG_FRAME_JPG):
                os.remove(DOG_FRAME_JPG)
        except Exception:
            pass
        if time.monotonic() < _video_block_until:
            return False
        cfg = _video_cfg or VIDEO_PROFILES['best']
        w = max(320, min(1280, int(cfg.get('width', VIDEO_WIDTH))))
        fps = max(5, min(20, int(cfg.get('fps', VIDEO_FPS))))
        q = max(2, min(8, int(cfg.get('q', VIDEO_Q))))
        cmd = [
            'ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'error',
            '-rtsp_transport', 'tcp',
            '-fflags', 'nobuffer', '-flags', 'low_delay',
            '-probesize', '32', '-analyzeduration', '0',
            '-i', DOG_RTSP,
            '-an',
            '-r', str(fps),
            '-vf', f'scale={w}:-2',
            '-q:v', str(q),
            '-f', 'image2', '-update', '1',
            DOG_FRAME_JPG,
        ]
        try:
            _video_feeder['proc'] = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                stdin=subprocess.DEVNULL, preexec_fn=os.setsid)
            proc_cache_invalidate()
            _video_feeder['last_use'] = time.monotonic()
            _video_feeder['verified'] = 0.0  # 下一次调用实打实核对一次（确认没叠出第二个）
            return True
        except Exception:
            _video_feeder['proc'] = None
            return False


def video_idle_reaper():
    """无人看视频超过 VIDEO_IDLE_SEC 则停 ffmpeg，省 CPU。"""
    while True:
        time.sleep(5.0)
        try:
            with _video_feeder['lock']:
                last = _video_feeder['last_use']
                proc = _video_feeder['proc']
            if last <= 0:
                continue
            if (time.monotonic() - last) < VIDEO_IDLE_SEC:
                continue
            if proc is None and not _ffmpeg_dog_live_pids():
                continue
            stop_video_feeder()
        except Exception:
            pass


def read_live_frame(wait_sec=2.5):
    """Return latest complete JPEG bytes from feeder file."""
    ensure_video_feeder()
    deadline = time.time() + wait_sec
    while time.time() < deadline:
        try:
            if os.path.isfile(DOG_FRAME_JPG) and os.path.getsize(DOG_FRAME_JPG) > 2000:
                with open(DOG_FRAME_JPG, 'rb') as f:
                    data = f.read()
                # 完整 JPEG：SOI + EOI，避免读到半帧
                if data[:2] == b'\xff\xd8' and data[-2:] == b'\xff\xd9':
                    return data
        except Exception:
            pass
        time.sleep(0.025)
    return None


def run_bg(cmd, logfile):
    full = f'source {ROS_SETUP} && source {WS_SETUP} && {cmd}'
    with open(logfile, 'a') as f:
        f.write(f'\n==== {time.strftime("%H:%M:%S")} ====\n')
    subprocess.Popen(['setsid', 'bash', '-c', full],
                     stdout=open(logfile, 'a'), stderr=subprocess.STDOUT,
                     stdin=subprocess.DEVNULL, start_new_session=True)
    proc_cache_invalidate()


# 进程表快照：一次扫 /proc 供 PROC_CACHE_TTL 内所有 proc_alive/proc_pids 共用。
# 2026-10-02 实测：每次 proc_alive 都整扫 /proc（~260 进程，≈2.6 ms），20 Hz 位姿定时器、
# 每帧 /scan、1 Hz 状态/自愈（几十次）、MJPEG 循环（每帧）叠加，web_ops 常驻 ~85% 单核。
# 杀进程（_kill/_killpg/kill_pattern）和拉进程（run_bg/ffmpeg）后立即作废快照，复查仍是实时结果。
PROC_CACHE_TTL = 0.5
_proc_cache = {'ts': -1e9, 'rows': (), 'gen': 0}


def proc_cache_invalidate():
    _proc_cache['gen'] += 1
    _proc_cache['ts'] = -1e9


def _proc_table(max_age=PROC_CACHE_TTL):
    """[(pid, cmdline)]，不含本进程。max_age=0 强制重扫。"""
    if max_age > 0 and time.monotonic() - _proc_cache['ts'] < max_age:
        return _proc_cache['rows']
    t0 = time.monotonic()
    gen = _proc_cache['gen']
    rows = []
    me = os.getpid()
    for name in os.listdir('/proc'):
        if not name.isdigit():
            continue
        pid = int(name)
        if pid == me:
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode('utf-8', 'ignore')
        except Exception:
            continue
        rows.append((pid, cmd))
    rows = tuple(rows)
    # 扫描期间有人杀/拉了进程（gen 变了）：本次结果照常返回，但不进缓存
    if _proc_cache['gen'] == gen:
        _proc_cache['rows'] = rows
        _proc_cache['ts'] = t0
    return rows


def _kill(pid, sig):
    try:
        os.kill(pid, sig)
    finally:
        proc_cache_invalidate()


def _killpg(pgid, sig):
    try:
        os.killpg(pgid, sig)
    finally:
        proc_cache_invalidate()


def proc_pids(pattern):
    """Return PIDs of real processes matching pattern (not agent/shell noise)."""
    pids = []
    for pid, cmd in _proc_table():
        if pattern not in cmd:
            continue
        # Ignore Cursor/agent shells, pgrep, and this helper text
        if any(x in cmd for x in (
                'extglob', 'pgrep', 'cursor-agent', 'start_all.sh',
                'COMMAND_EXIT_CODE', 'dump_bash_state')):
            continue
        try:
            exe = os.readlink(f'/proc/{pid}/exe')
        except Exception:
            exe = ''
        # Prefer real binaries / installed node entrypoints
        ok = False
        if pattern in exe:
            ok = True
        elif f'/{pattern}' in cmd or f' {pattern} ' in f' {cmd} ':
            # python entrypoints e.g. .../lib/nav2_tools/map_building_node
            if 'ros2 launch' in cmd and pattern in cmd:
                ok = True
            if f'lib/' in cmd and pattern in cmd:
                ok = True
            exe_base = os.path.basename(exe)
            if exe_base.startswith('python3') or exe_base.startswith('python'):
                # only if argv looks like the node, not a random script quoting the name
                # (strip .py so script-style launches like nav_scan_node.py still match)
                parts = cmd.split()
                stems = [os.path.basename(p.rstrip('/')) for p in parts]
                stems = [s[:-3] if s.endswith('.py') else s for s in stems]
                if any(s == pattern or s.endswith(pattern) for s in stems):
                    ok = True
        if ok:
            pids.append(pid)
    return pids


def proc_alive(pattern):
    return bool(proc_pids(pattern))


def kill_pattern(pattern, sig=signal.SIGTERM):
    for pid in proc_pids(pattern):
        try:
            _kill(pid, sig)
        except Exception:
            pass


# ---------------- HTTP API ----------------
class ApiHandler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _cors(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET,POST,OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')

    def _send_json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self._cors()
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(200)
        self._cors()
        self.end_headers()

    def do_GET(self):
        path = self.path.split('?')[0]
        query = {}
        if '?' in self.path:
            for kv in self.path.split('?')[1].split('&'):
                if '=' in kv:
                    k, v = kv.split('=', 1)
                    query[k] = v
        try:
            if path == '/api/maps':
                self._send_json(list_maps())
            elif path == '/api/map/pgm':
                name = query.get('name', '')
                self._serve_pgm(name)
            elif path == '/api/map/preview':
                name = query.get('name', '')
                self._serve_map_preview(name)
            elif path == '/api/sysinfo':
                self._send_json(sysinfo())
            elif path == '/api/battery':
                self._send_json(battery_json())
            elif path == '/api/mapping/status':
                self._send_json(mapping_status())
            elif path == '/api/lidar_extrinsic':
                self._send_json(load_lidar_extrinsic())
            elif path == '/api/nav2/map':
                self._send_json(current_map_json(query.get('map', '')))
            elif path == '/api/cruise/list':
                self._send_json(cruise_list(query.get('map', '')))
            elif path == '/api/nav2/speed':
                self._send_json(api_nav_speed_get())
            elif path == '/api/nav2/reload_status':
                self._send_json(reload_job_status())
            elif path == '/api/log':
                file = query.get('file', 'webops.log')
                n = int(query.get('n', '50'))
                self._send_json({'file': file, 'log': tail_log(file, n)})
            elif path == '/api/video/mjpeg':
                self._serve_mjpeg(query.get('url') or DOG_RTSP)
            elif path == '/api/video/snapshot':
                self._serve_snapshot(query.get('url') or DOG_RTSP)
            elif path == '/api/video/meter':
                self._send_json(video_meter())
            else:
                self._send_json({'error': 'not found'}, 404)
        except Exception as e:
            self._send_json({'error': str(e)}, 500)

    def do_POST(self):
        path = self.path.split('?')[0]
        try:
            ln = int(self.headers.get('Content-Length', 0))
            raw = self.rfile.read(ln) if ln else b''
            body = json.loads(raw.decode('utf-8')) if raw else {}
            if path == '/api/map/edit':
                self._send_json(save_map_edit(body))
            elif path == '/api/map/delete':
                self._send_json(delete_map(body))
            elif path == '/api/mapping/save':
                self._send_json(save_mapping(body))
            elif path == '/api/mapping/stop':
                self._send_json(stop_mapping_keep_pcd())
            elif path == '/api/mapping/discard':
                self._send_json(discard_mapping())
            elif path == '/api/mapping/start':
                self._send_json(start_mapping_session(body))
            elif path == '/api/navmap':
                self._send_json(set_nav_pick(body.get('name', '')))
            elif path == '/api/nav2/start':
                self._send_json(start_nav2(body))
            elif path == '/api/nav2/stop':
                self._send_json(stop_nav2_api())
            elif path == '/api/nav2/accept_pose':
                self._send_json(accept_reloc_pose())
            elif path == '/api/nav2/speed':
                self._send_json(api_nav_speed_set(body))
            elif path == '/api/nav2/init_pose':
                self._send_json(api_init_pose(body))
            elif path == '/api/nav2/goal':
                self._send_json(api_nav_goal(body))
            elif path == '/api/nav2/goals':
                self._send_json(api_nav_goals(body))
            elif path == '/api/cruise/save':
                self._send_json(cruise_save(body))
            elif path == '/api/cruise/delete':
                self._send_json(cruise_delete(body))
            elif path == '/api/cruise/start_route':
                self._send_json(api_cruise_start_route(body))
            elif path == '/api/nav2/cancel':
                if _web_ops_node is None:
                    self._send_json({'ok': False, 'error': 'web_ops 节点未就绪'})
                else:
                    _web_ops_node.cb_nav_cancel(Empty())
                    self._send_json({'ok': True})
            elif path in ('/api/nav2/relocalize', '/api/nav2/global_relocalize',
                          '/api/nav2/reloc_mode'):
                # 已移除 2D 盲搜 auto_relocalize 与 3D-BBS；请用地图拖拽设初姿
                self._send_json({
                    'ok': False,
                    'removed': True,
                    'error': '已改为 Go2W 初姿流程：请点「设初始位姿」在地图上拖出朝向',
                })
            elif path == '/api/nav2/reload_map':
                self._send_json(reload_nav_map(body))
            elif path == '/api/restart_rosbridge':
                self._send_json(restart_rosbridge())
            elif path == '/api/localization/restart':
                self._send_json(restart_localization_stack())
            elif path == '/api/ctrl':
                self._send_json(set_ctrl_mode(
                    body.get('mode', ''),
                    force_restart=bool(body.get('force_restart', False))))
            elif path == '/api/sdk/heal':
                # 同步做会堵住 HTTP（SSH+重启 >3s）；后台跑，前端轮询 /api/battery
                def _bg():
                    try:
                        # 手动点恢复：直接重启狗端运控，覆盖「单个僵死 mc_ctrl」
                        ensure_dog_single_mc_ctrl(restart=True)
                    except Exception:
                        pass
                    try:
                        set_ctrl_mode('SDK', force_restart=True, restart_mc_ctrl=False)
                    except Exception:
                        pass
                threading.Thread(target=_bg, daemon=True).start()
                self._send_json({'ok': True, 'started': True,
                                 'msg': '后台恢复 SDK：重启狗端 mc_ctrl + 重启 bridge'})
            elif path == '/api/cmd_vel':
                self._send_json(publish_cmd_vel(body))
            elif path == '/api/video/stop':
                # 运动页遥控短暂停图；建图页前端不再调此接口
                stop_video_feeder(block_sec=6.0)
                self._send_json({'ok': True, 'stopped': True, 'blocked_sec': 6})
            elif path == '/api/video/resume':
                global _video_block_until
                _video_block_until = 0.0
                self._send_json({'ok': True, 'resumed': True})
            elif path == '/api/video/profile':
                self._send_json(set_video_profile(
                    body.get('profile', 'best'), body.get('rtt_ms')))
            else:
                self._send_json({'error': 'not found'}, 404)
        except Exception as e:
            self._send_json({'error': str(e)}, 500)

    def _serve_pgm(self, name):
        safe = os.path.basename(name)
        p = os.path.join(MAPS_DIR, safe)
        if not safe.endswith('.pgm') or not os.path.isfile(p):
            self._send_json({'error': 'no such map'}, 404)
            return
        with open(p, 'rb') as f:
            data = f.read()
        self.send_response(200)
        self.send_header('Content-Type', 'image/x-portable-graymap')
        self._cors()
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _serve_map_preview(self, name):
        """Serve PGM as PNG — browsers cannot render raw PGM in <img>."""
        safe = os.path.basename(unquote(name))
        if not safe.endswith('.pgm'):
            safe += '.pgm'
        p = os.path.join(MAPS_DIR, safe)
        if not os.path.isfile(p):
            self._send_json({'error': 'no such map'}, 404)
            return
        try:
            png = pgm_file_to_png(p)
        except Exception as e:
            self._send_json({'error': str(e)}, 500)
            return
        self.send_response(200)
        self.send_header('Content-Type', 'image/png')
        self.send_header('Cache-Control', 'no-cache')
        self._cors()
        self.send_header('Content-Length', str(len(png)))
        self.end_headers()
        self.wfile.write(png)

    def _serve_mjpeg(self, rtsp_url):
        """Serve multipart MJPEG from live frame cache (browser-friendly)."""
        if time.monotonic() < _video_block_until:
            self._send_json({'error': 'video paused for teleop', 'blocked': True}, 503)
            return
        ensure_video_feeder()
        try:
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Access-Control-Allow-Origin', '*')
            self._cors()
            self.end_headers()
            last = b''
            last_sig = None
            idle = 0
            interval = max(0.04, 1.0 / max(8, min(30, int((_video_cfg or {}).get('fps', VIDEO_FPS)))))
            while idle < 100:
                if time.monotonic() < _video_block_until:
                    break
                # 文件没变（mtime+大小相同）就不必整帧读进来再比较
                try:
                    st = os.stat(DOG_FRAME_JPG)
                    sig = (st.st_mtime_ns, st.st_size)
                except OSError:
                    sig = None
                if sig is not None and sig == last_sig:
                    ensure_video_feeder()  # 保持 last_use 刷新（原来每轮 read_live_frame 都会调）
                    time.sleep(0.02)
                    continue
                data = read_live_frame(wait_sec=0.5)
                if not data:
                    idle += 1
                    time.sleep(0.05)
                    continue
                idle = 0
                if data == last:
                    last_sig = sig
                    time.sleep(0.02)
                    continue
                last = data
                last_sig = sig
                part = (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n'
                    b'Content-Length: ' + str(len(data)).encode() + b'\r\n\r\n'
                    + data + b'\r\n'
                )
                self.wfile.write(part)
                self.wfile.flush()
                note_mjpeg_frame()
                time.sleep(interval * 0.5)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception:
            pass

    def _serve_snapshot(self, rtsp_url):
        # 遥控 block 期间不要再起 ffmpeg（哪怕单帧），否则建图页轮询会打满 CPU
        if time.monotonic() < _video_block_until:
            self._send_json({'error': 'video paused for teleop', 'blocked': True}, 503)
            return
        data = read_live_frame(wait_sec=1.5)
        if not data:
            if time.monotonic() < _video_block_until:
                self._send_json({'error': 'video paused for teleop', 'blocked': True}, 503)
                return
            cmd = [
                'ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'error',
                '-rtsp_transport', 'tcp', '-fflags', 'nobuffer', '-flags', 'low_delay',
                '-y', '-i', rtsp_url or DOG_RTSP,
                '-frames:v', '1', '-q:v', str(max(2, min(8, int((_video_cfg or {}).get('q', VIDEO_Q))))),
                '-vf', f'scale={max(320, min(1280, int((_video_cfg or {}).get("width", VIDEO_WIDTH))))}:-2',
                '-f', 'image2pipe', '-vcodec', 'mjpeg', '-',
            ]
            try:
                r = subprocess.run(cmd, capture_output=True, timeout=8)
                data = r.stdout if r.returncode == 0 else b''
            except Exception:
                data = b''
        if not data:
            self._send_json({'error': 'snapshot failed'}, 502)
            return
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Cache-Control', 'no-cache, no-store')
        self._cors()
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def pgm_file_to_png(path, max_side=256):
    """Read a P5 PGM and return PNG bytes suitable for <img> thumbnails."""
    with open(path, 'rb') as f:
        raw = f.read()
    i = 0
    n = len(raw)

    def tok():
        nonlocal i
        while i < n:
            if raw[i:i + 1] == b'#':
                while i < n and raw[i:i + 1] != b'\n':
                    i += 1
                continue
            if raw[i] in (9, 10, 13, 32):
                i += 1
                continue
            break
        s = b''
        while i < n and raw[i] not in (9, 10, 13, 32) and raw[i:i + 1] != b'#':
            s += raw[i:i + 1]
            i += 1
        return s.decode('ascii', 'replace')

    if tok() != 'P5':
        raise ValueError('not a P5 PGM')
    w, h, _mv = int(tok()), int(tok()), int(tok())
    while i < n and raw[i] in (9, 10, 13, 32):
        i += 1
    pix = raw[i:i + w * h]
    if len(pix) < w * h:
        raise ValueError('truncated PGM')
    # ROS PGM: 0 障碍 / 254 自由 / 205 未知 —— 不可用 >200，否则 205 会被当成自由
    out = bytearray(w * h)
    for k, v in enumerate(pix):
        out[k] = 0 if v <= 50 else (255 if v >= 250 else 128)
    img = Image.frombytes('L', (w, h), bytes(out)).convert('RGB')
    if max(w, h) > max_side:
        img.thumbnail((max_side, max_side), Image.NEAREST)
    buf = BytesIO()
    img.save(buf, format='PNG', optimize=True)
    return buf.getvalue()


def list_maps():
    out = []
    if not os.path.isdir(MAPS_DIR):
        return {'maps': out}
    for f in sorted(os.listdir(MAPS_DIR)):
        if not f.endswith('.pgm'):
            continue
        if f.startswith('__') or '.bak.' in f or f.endswith('.bak.pgm'):
            continue
        yaml_path = os.path.join(MAPS_DIR, f[:-4] + '.yaml')
        meta = {'resolution': 0.05, 'origin': [0.0, 0.0, 0.0]}
        if os.path.isfile(yaml_path):
            for line in open(yaml_path):
                m = re.match(r'resolution:\s*([\d.eE+-]+)', line)
                if m:
                    meta['resolution'] = float(m.group(1))
                m = re.match(r'origin:\s*\[([^,]+),\s*([^,\]]+)', line)
                if m:
                    meta['origin'] = [float(m.group(1)), float(m.group(2)), 0.0]
        try:
            with open(os.path.join(MAPS_DIR, f), 'rb') as fh:
                head = fh.read(64)
            m = re.match(rb'P5\n(\d+) (\d+)\n', head)
            w, h = (int(m.group(1)), int(m.group(2))) if m else (0, 0)
        except Exception:
            w = h = 0
        out.append({'name': f, 'width': w, 'height': h,
                    'resolution': meta['resolution'], 'origin': meta['origin'],
                    'mtime': time.strftime('%m-%d %H:%M', time.localtime(
                        os.path.getmtime(os.path.join(MAPS_DIR, f))))})
    return {'maps': out, 'selected': read_nav_pick()}


def _pgm_name(name):
    name = os.path.basename(str(name or '').strip())
    if name.endswith('.yaml'):
        name = name[:-5] + '.pgm'
    elif name and not name.endswith('.pgm'):
        name += '.pgm'
    return name


def read_nav_pick():
    """地图页「设为导航地图」写在机器人上，导航页下拉框读这份，不靠浏览器本地记录。"""
    try:
        name = _pgm_name(open(NAV_PICK_FILE, encoding='utf-8').read())
    except Exception:
        return ''
    if name and os.path.isfile(os.path.join(MAPS_DIR, name)):
        return name
    return ''


def set_nav_pick(name):
    name = _pgm_name(name)
    if not name or not os.path.isfile(os.path.join(MAPS_DIR, name)):
        return {'ok': False, 'error': '地图不存在'}
    os.makedirs(MAPS_DIR, exist_ok=True)
    with open(NAV_PICK_FILE, 'w', encoding='utf-8') as handle:
        handle.write(name + '\n')
    return {'ok': True, 'name': name}


def save_map_edit(body):
    name = os.path.basename(body.get('name', ''))
    if not name.endswith('.pgm'):
        name += '.pgm'
    data = base64.b64decode(body['data'])
    pgm_path = os.path.join(MAPS_DIR, name)
    with open(pgm_path, 'wb') as f:
        f.write(data)
    res = float(body.get('resolution', 0.05))
    origin = body.get('origin', [0.0, 0.0])
    yaml_path = pgm_path[:-4] + '.yaml'
    with open(yaml_path, 'w') as f:
        f.write('image: %s\n' % name)
        f.write('resolution: %f\n' % res)
        f.write('origin: [%f, %f, 0.0]\n' % (float(origin[0]), float(origin[1])))
        f.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.15\n')
    return {'ok': True, 'name': name}


def delete_map(body):
    """删除地图库中的 .pgm+.yaml；允许删当前选中/正在导航的图。

    若 Nav2 正挂着这张图：先停导航并清空选中，避免库空了还挂旧图。
    之后点「启动导航」会因无图报「没有可用导航地图」。
    """
    global _nav2_current_map
    name = os.path.basename(body.get('name', ''))
    if not name.endswith('.pgm'):
        return {'ok': False, 'error': 'name must end .pgm'}
    stem = name[:-4]
    was_nav = False
    cur = _nav2_current_map or ''
    if cur and (os.path.basename(cur) == stem + '.yaml'
                or os.path.basename(cur) == stem + '.pgm'):
        was_nav = True
        stop_nav2_procs()
        _nav2_current_map = None
    removed = []
    for suffix in ('.pgm', '.yaml'):
        p = os.path.join(MAPS_DIR, stem + suffix)
        if os.path.isfile(p):
            os.remove(p)
            removed.append(suffix)
    if not removed:
        return {'ok': False, 'error': '地图文件不存在'}
    try:
        picked = _pgm_name(open(NAV_PICK_FILE, encoding='utf-8').read())
    except Exception:
        picked = ''
    if picked == name:
        try:
            os.remove(NAV_PICK_FILE)
        except OSError:
            pass
    return {
        'ok': True,
        'removed': removed,
        'stopped_nav': was_nav,
        'hint': '已删除；若无其它图，启动导航会提示缺少地图' if was_nav else None,
    }


def stop_relocation_node(sig=signal.SIGTERM):
    """停掉先验重定位节点（与 super_lio_node 互斥，同发 /lio/odom）。"""
    killed = []
    for needle in ('relocation_node', 'dog_reloc.py'):
        for pid in proc_pids(needle):
            try:
                _kill(pid, sig)
                killed.append(pid)
            except Exception:
                pass
    return killed


def collapse_map_building(keep_one=True):
    """叠了多个 map_building_node 时只留最新一个（或全停）。"""
    pids = sorted(proc_pids('map_building_node'))
    launches = sorted(proc_pids('map_building.launch'))
    if keep_one and len(pids) <= 1 and len(launches) <= 1:
        return {'ok': True, 'kept': pids[:1], 'killed': []}
    killed = []
    # 先杀多余 launch，再杀多余 node
    drop_l = launches[:-1] if (keep_one and launches) else launches
    drop_n = pids[:-1] if (keep_one and pids) else pids
    if not keep_one:
        drop_l, drop_n = launches, pids
    for pid in drop_l + drop_n:
        try:
            _kill(pid, signal.SIGTERM)
            killed.append(pid)
        except Exception:
            pass
    time.sleep(0.4)
    for pid in drop_l + drop_n:
        try:
            _kill(pid, signal.SIGKILL)
        except Exception:
            pass
    return {'ok': True, 'kept': ([] if not keep_one else sorted(proc_pids('map_building_node'))[-1:]),
            'killed': killed}


def stop_super_lio(sig=signal.SIGINT):
    pids = proc_pids('super_lio_node')
    for pid in pids:
        try:
            _kill(pid, sig)
        except Exception:
            pass
    return pids


PCD_DIR = os.path.dirname(SUPERLIO_PCD)
LIDAR_EXTRINSIC = '/home/linaro/steel_coin_nav_ws/config/lidar_extrinsic.yaml'
OBSTACLE_HEIGHT_BAND = '/home/linaro/steel_coin_nav_ws/config/obstacle_height_band.yaml'


def load_obstacle_height_band():
    """stage2 /scan 与 pcd2pgm 共用高度带（相对水平系 z）。"""
    out = {'z_min': 0.05, 'z_max': 0.85}
    try:
        with open(OBSTACLE_HEIGHT_BAND, 'r') as f:
            for line in f:
                line = line.split('#', 1)[0].strip()
                if not line or ':' not in line:
                    continue
                k, v = line.split(':', 1)
                k, v = k.strip(), v.strip()
                if k in ('z_min', 'z_max'):
                    out[k] = float(v)
    except Exception:
        pass
    if out['z_max'] <= out['z_min']:
        out = {'z_min': 0.05, 'z_max': 0.85}
    return out

def load_lidar_extrinsic():
    """Read pitch_down / roll / lidar_z for UI + pcd2pgm."""
    out = {
        'ok': True,
        'pitch_down': 0.342586,  # dog-head Mid360 looking down ~19.6°
        'roll': 0.0,
        'lidar_z': 0.45,
        'lidar_pitch': -0.342586,
        'mount_pitch_down': 0.342586,
        'source': 'default',
    }
    if not os.path.isfile(LIDAR_EXTRINSIC):
        return out
    try:
        with open(LIDAR_EXTRINSIC, 'r') as f:
            for ln in f:
                ln = ln.split('#', 1)[0].strip()
                if ':' not in ln:
                    continue
                k, v = ln.split(':', 1)
                k, v = k.strip(), v.strip()
                try:
                    out[k] = float(v)
                except ValueError:
                    out[k] = v
        if 'pitch_down' not in out and 'lidar_pitch' in out:
            out['pitch_down'] = abs(float(out['lidar_pitch']))
        out['ok'] = True
        out['source'] = 'file'
    except Exception as e:
        out['ok'] = False
        out['error'] = str(e)
    return out


def clear_pcd_cache():
    """删除 Super-LIO 上次留下的 PCD（map.pcd + map/PCD/scans_*.pcd），避免旧云污染新图。"""
    removed = []
    try:
        if os.path.isdir(SUPERLIO_PCD_DIR):
            for fn in os.listdir(SUPERLIO_PCD_DIR):
                p = os.path.join(SUPERLIO_PCD_DIR, fn)
                if fn == os.path.basename(SUPERLIO_PCD) or fn == 'PCD':
                    try:
                        if os.path.isdir(p):
                            for sub in os.listdir(p):
                                sp = os.path.join(p, sub)
                                if sub.startswith('scans') and sub.endswith('.pcd'):
                                    try:
                                        os.remove(sp)
                                        removed.append('PCD/' + sub)
                                    except Exception:
                                        pass
                        else:
                            os.remove(p)
                            removed.append(fn)
                    except Exception:
                        pass
    except Exception as e:
        return {'ok': False, 'error': str(e), 'removed': removed}
    return {'ok': True, 'removed': removed}


def pcd_ready():
    """是否有可转换的 map.pcd（体积足够）。"""
    try:
        return os.path.isfile(SUPERLIO_PCD) and os.path.getsize(SUPERLIO_PCD) > 1024
    except Exception:
        return False


def list_pcd_fragments():
    """Super-LIO save_interval>0 时写的 map/PCD/scans_*.pcd 分片。"""
    frag_dir = os.path.join(SUPERLIO_PCD_DIR, 'PCD')
    out = []
    if not os.path.isdir(frag_dir):
        return out
    for fn in sorted(os.listdir(frag_dir)):
        if fn.startswith('scans_') and fn.endswith('.pcd'):
            p = os.path.join(frag_dir, fn)
            try:
                if os.path.isfile(p) and os.path.getsize(p) > 1024:
                    out.append(p)
            except Exception:
                pass
    return out


def _write_pcd_xyz_binary(path, pts):
    n = len(pts)
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z\n'
        'SIZE 4 4 4\n'
        'TYPE F F F\n'
        'COUNT 1 1 1\n'
        f'WIDTH {n}\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {n}\n'
        'DATA binary\n'
    )
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        for x, y, z in pts:
            f.write(struct.pack('<fff', float(x), float(y), float(z)))


def merge_pcd_fragments(voxel=0.05):
    """把 scans_*.pcd 合并成 map.pcd（体素降采样，避免 RK3588 OOM）。

    Super-LIO 在 save_interval>0 时 SIGINT→ProcessCaceMap 常要数分钟且
    if_filter:false 易把整图载入内存；旧 stop 逻辑约 17s 就 SIGKILL，
    导致无 map.pcd、网页「保存地图」一直灰。
    """
    frags = list_pcd_fragments()
    if not frags:
        return {'ok': False, 'error': '没有 PCD 分片可合并', 'fragments': 0}
    # 复用 pcd2pgm 的流式读取
    try:
        import importlib.util
        spec = importlib.util.spec_from_file_location('pcd2pgm_merge', PCD2PGM)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        read_pcd = mod.read_pcd
    except Exception as e:
        return {'ok': False, 'error': '无法加载 pcd2pgm.read_pcd: %s' % e, 'fragments': len(frags)}

    kept = {}
    inv = 1.0 / max(1e-3, float(voxel))
    total_in = 0
    used, skipped = [], []
    for fp in frags:
        try:
            pts = read_pcd(fp, voxel=0.0)
        except Exception as e:
            # 强杀时最后一个 scans_*.pcd 常半截，跳过即可
            skipped.append({'file': os.path.basename(fp), 'error': str(e)})
            continue
        if not pts:
            skipped.append({'file': os.path.basename(fp), 'error': 'empty'})
            continue
        used.append(os.path.basename(fp))
        total_in += len(pts)
        for x, y, z in pts:
            key = (int(math.floor(x * inv)), int(math.floor(y * inv)),
                   int(math.floor(z * inv)))
            if key not in kept:
                kept[key] = (x, y, z)
    if not kept:
        return {'ok': False, 'error': '分片合并后为空（可能全损坏）',
                'fragments': len(frags), 'skipped': skipped}
    pts_out = list(kept.values())
    try:
        _write_pcd_xyz_binary(SUPERLIO_PCD, pts_out)
    except Exception as e:
        return {'ok': False, 'error': '写 map.pcd 失败: %s' % e,
                'fragments': len(frags), 'skipped': skipped}
    return {
        'ok': True,
        'fragments': len(frags),
        'used': used,
        'skipped': skipped,
        'points_in': total_in,
        'points_out': len(pts_out),
        'voxel': float(voxel),
        'pcd_path': SUPERLIO_PCD,
        'pcd_size': os.path.getsize(SUPERLIO_PCD),
    }


def ensure_map_pcd(wait_super_lio_sec=90.0):
    """确保有可保存的 map.pcd：先等 Super-LIO 落盘，不行再合并分片。"""
    deadline = time.monotonic() + max(0.0, float(wait_super_lio_sec))
    prev_mtime = os.path.getmtime(SUPERLIO_PCD) if os.path.isfile(SUPERLIO_PCD) else 0
    while time.monotonic() < deadline:
        if os.path.isfile(SUPERLIO_PCD):
            try:
                mt = os.path.getmtime(SUPERLIO_PCD)
                sz = os.path.getsize(SUPERLIO_PCD)
            except Exception:
                mt, sz = 0, 0
            if sz > 1024 and (mt > prev_mtime or (time.time() - mt) < 120):
                # 文件仍在增长则再等一会
                time.sleep(1.0)
                try:
                    sz2 = os.path.getsize(SUPERLIO_PCD)
                except Exception:
                    sz2 = sz
                if sz2 == sz:
                    return {'ok': True, 'pcd_ready': True, 'source': 'super_lio',
                            'pcd_size': sz2}
        # 进程已退出且仍无 map.pcd → 不必空等
        if not proc_alive('super_lio_node') and not pcd_ready():
            break
        time.sleep(0.5)
    if pcd_ready():
        return {'ok': True, 'pcd_ready': True, 'source': 'super_lio',
                'pcd_size': os.path.getsize(SUPERLIO_PCD)}
    frags = list_pcd_fragments()
    if not frags:
        return {'ok': False, 'pcd_ready': False, 'fragments': 0,
                'error': '未生成 map.pcd，且没有 scans_*.pcd 分片；请重新建图后再停止'}
    merged = merge_pcd_fragments(voxel=0.05)
    if merged.get('ok'):
        return {'ok': True, 'pcd_ready': True, 'source': 'merged_fragments',
                'pcd_size': merged.get('pcd_size', 0), 'merge': merged}
    return {'ok': False, 'pcd_ready': False, 'fragments': len(frags),
            'error': merged.get('error') or '分片合并失败', 'merge': merged}


def mapping_status():
    global _mapping_session, _mapping_session_known
    running = proc_alive('super_lio_node')
    building = proc_alive('map_building_node')
    nav2 = proc_alive('component_container_isolated')
    # 仅 web_ops 重启后、尚无显式 start/stop 时，用 LIO+预览推断「正在建图」
    if (not _mapping_session_known and running and building and not nav2
            and not _mapping_stopping):
        _mapping_session = True
        _mapping_session_known = True
    if nav2 or _mapping_stopping:
        session = False
    else:
        session = bool(_mapping_session)
    ready = pcd_ready()
    frags = list_pcd_fragments()
    size = 0
    mtime = None
    if os.path.isfile(SUPERLIO_PCD):
        try:
            size = os.path.getsize(SUPERLIO_PCD)
            mtime = time.strftime('%H:%M:%S', time.localtime(os.path.getmtime(SUPERLIO_PCD)))
        except Exception:
            pass
    # 有分片、LIO 已停、尚无 map.pcd：也可进入「可保存」流程（save 时会先合并）
    can_save = (not running) and (ready or bool(frags)) and not _mapping_stopping
    return {
        'ok': True,
        'session': session,
        'stopping': bool(_mapping_stopping),
        'fastlio': running,
        'map_building': building,
        'pcd_ready': (not running) and ready,
        'pcd_exists': ready,
        'pcd_fragments': len(frags),
        'pcd_size': size,
        'pcd_mtime': mtime,
        'can_start': not session and not running and not nav2 and not _mapping_stopping,
        'can_stop': (session or running) and not _mapping_stopping,
        'can_save': can_save,
    }


def stop_mapping_keep_pcd():
    """停止 Super-LIO：SIGINT 触发 saveMap()，超时则合并 PCD 分片为 map.pcd。"""
    global _mapping_stopping
    set_mapping_session(False)
    _mapping_stopping = True
    force_killed = False
    try:
        pids = proc_pids('super_lio_node')
        if pids:
            stop_super_lio(signal.SIGINT)
            # 先给 ProcessCaceMap 足够时间（大图合并可达数分钟）
            ensured = ensure_map_pcd(wait_super_lio_sec=120.0)
            kill_pattern('map_building_node')
            kill_pattern('Livox_mid360.py')
            still = proc_pids('super_lio_node')
            if still and not ensured.get('pcd_ready'):
                # 再宽限一段时间；仍无结果再强杀并用分片兜底
                time.sleep(5.0)
                ensured = ensure_map_pcd(wait_super_lio_sec=5.0)
            still = proc_pids('super_lio_node')
            for pid in still:
                try:
                    _kill(pid, signal.SIGKILL)
                    force_killed = True
                except Exception:
                    pass
            if not ensured.get('pcd_ready'):
                ensured = ensure_map_pcd(wait_super_lio_sec=0.0)
        else:
            ensured = ensure_map_pcd(wait_super_lio_sec=0.0)
            if ensured.get('ok') and ensured.get('source') == 'merged_fragments':
                pass  # 上次被强杀留下的分片，现在补合并
            elif not ensured.get('pcd_ready') and not list_pcd_fragments():
                ensured = {
                    'ok': True, 'already_stopped': True,
                    'pcd_ready': pcd_ready(),
                    'pcd_size': os.path.getsize(SUPERLIO_PCD) if os.path.isfile(SUPERLIO_PCD) else 0,
                }

        # 关键外壳必须杀：否则 mode:=mapping launch 残留会挡导航
        orphans = stop_mapping_launch_orphans()
        set_mapping_session(False)
        ready = bool(ensured.get('pcd_ready') or pcd_ready())
        return {
            'ok': True,
            'pcd_ready': ready,
            'pcd_saved': ready,
            'pcd_size': ensured.get('pcd_size') or (
                os.path.getsize(SUPERLIO_PCD) if os.path.isfile(SUPERLIO_PCD) else 0),
            'pcd_source': ensured.get('source'),
            'pcd_fragments': len(list_pcd_fragments()),
            'force_killed': force_killed,
            'orphans_killed': orphans.get('killed', 0),
            'merge': ensured.get('merge'),
            'error': None if ready else (
                ensured.get('error')
                or '未生成 map.pcd，请确认 livox_360.yaml 开启 lio.map.save_map 后重新建图'),
        }
    finally:
        _mapping_stopping = False
        set_mapping_session(False)
        stop_mapping_launch_orphans()


def start_mapping_session(body=None):
    """清理旧 PCD 后启动/确保建图链（幂等，不会叠多个 Livox）。

    真正的幂等靠 super_lio_node/map_building_node 是否已存活来判断，但
    run_bg() 是异步的——从"没看到进程"到"进程真的起来"之间有好几秒空窗。
    两次几乎同时的 /api/mapping/start 请求（双击、页面重试、脚本和网页撞车）
    都会在这个空窗里各自判定"没在跑"然后各起一份 start_all.sh mapping，
    而 start_all.sh 自己的 ensure_one() 也是同样的检查-再启动，两边一起
    竞态就会叠出两个 super_lio_node + 两个 map_building_node 抢同一路雷达，
    建图直接乱掉（这次排查到的真实故障)。用一把进程内锁把"决定要不要启动"
    这段收窄到互斥执行，第二个请求要么等第一个判定完直接复用，要么在第一个
    还没判定完时立刻被拒绝，不会再有两边同时通过检查的窗口。
    """
    global _mapping_starting
    with _mapping_lock:
        if _mapping_stopping:
            return {'ok': False, 'error': '正在停止建图，请稍候'}
        if _mapping_starting:
            return {'ok': False, 'error': '建图正在启动中，请稍候'}
        if proc_alive('component_container_isolated'):
            return {'ok': False, 'error': '请先关闭导航'}
        # 与 relocation_node 互斥，否则 /lio/odom 双源导致建图页狗位姿跳变
        if proc_alive('relocation_node'):
            stop_relocation_node()
            time.sleep(0.8)
        # 若已在建图，仅确保预览（并压扁叠出来的多个 map_building）
        if proc_alive('super_lio_node'):
            set_mapping_session(True)
            collapse_map_building(keep_one=True)
            if not proc_alive('map_building_node'):
                run_bg(
                    'ros2 launch nav2_tools map_building.launch.py',
                    '/tmp/map_building.log')
            return {'ok': True, 'already_running': True, 'cleared': [], 'session': True}
        _mapping_starting = True
    try:
        cleared = clear_pcd_cache()
        # 导航定位链（关闭导航只停 Nav2 容器，它还在跑）必须先停：否则建图再起一个 livox 驱动，
        # 两个驱动抢雷达/IMU（2026-10-01 两次建图都这样，FAST-LIO「IMU and LiDAR not Synced」）
        stop_localization_chain()
        # 再清一次，防止锁外窗口又被 start_reloc 拉起
        stop_relocation_node()
        collapse_map_building(keep_one=False)
        # start_all.sh is idempotent: collapses duplicate Livox/LIO first
        run_bg(f'bash {START_ALL} mapping', '/tmp/mapping.log')
        # 等 start_all.sh 里的 ensure_one() 真正把 super_lio_node 跑起来再放锁，
        # 覆盖掉 run_bg() 异步返回到进程实际存活之间的那段窗口。
        for _ in range(30):
            if proc_alive('super_lio_node'):
                break
            time.sleep(0.5)
        set_mapping_session(True)
        collapse_map_building(keep_one=True)
        return {'ok': True, 'started': True, 'cleared': cleared.get('removed', []), 'session': True}
    finally:
        _mapping_starting = False


def discard_mapping():
    """Stop Super-LIO without converting PCD to a map."""
    set_mapping_session(False)
    pids = stop_super_lio(signal.SIGINT)
    # Also stop map_building preview node so next session starts clean
    kill_pattern('map_building_node')
    time.sleep(0.5)
    if pids:
        # Force if still alive
        still = proc_pids('super_lio_node')
        for pid in still:
            try:
                _kill(pid, signal.SIGKILL)
            except Exception:
                pass
    # 同 stop：清 mapping launch 外壳与其孤儿 livox 驱动，避免导航时双驱动抢雷达
    stop_mapping_launch_orphans()
    return {'ok': True, 'stopped': len(pids), 'discarded': True}


def save_mapping(body):
    """保存建图：SIGINT super_lio -> 等 map.pcd -> pcd2pgm -> maps/<name>.pgm+yaml"""
    name = os.path.basename(body.get('name', 'map'))
    if name in ('__discard__', 'discard', ''):
        return discard_mapping()
    if name.endswith(('.pgm', '.yaml')):
        name = name.rsplit('.', 1)[0]
    name = re.sub(r'[^\w\-]+', '_', name) or 'map'

    set_mapping_session(False)
    pids = proc_pids('super_lio_node')
    if not pids:
        if not pcd_ready() and list_pcd_fragments():
            merged = merge_pcd_fragments(voxel=0.05)
            if not merged.get('ok'):
                return {'ok': False, 'error': merged.get('error') or '分片合并失败'}
        if os.path.isfile(SUPERLIO_PCD):
            return run_pcd2pgm(name)
        return {'ok': False, 'error': 'super_lio 未运行且无 PCD 可转换'}

    # Remember mtime before kill so we wait for a fresh PCD
    prev_mtime = os.path.getmtime(SUPERLIO_PCD) if os.path.isfile(SUPERLIO_PCD) else 0
    stop_super_lio(signal.SIGINT)

    pcd_path = SUPERLIO_PCD
    saved = False
    for _ in range(20):
        if os.path.isfile(pcd_path):
            mt = os.path.getmtime(pcd_path)
            if mt > prev_mtime or (time.time() - mt) < 30:
                # Wait a bit more for flush
                time.sleep(1.0)
                if os.path.getsize(pcd_path) > 100:
                    saved = True
                    break
        time.sleep(1)

    kill_pattern('map_building_node')
    if not saved and not os.path.isfile(pcd_path):
        return {'ok': False, 'error': 'Super-LIO 已停止但未生成 map.pcd'}
    return run_pcd2pgm(name, pcd_path)


def run_pcd2pgm(name, pcd_path=SUPERLIO_PCD):
    import tempfile
    import shutil
    os.makedirs(MAPS_DIR, exist_ok=True)
    out_base = os.path.join(MAPS_DIR, name)
    overwritten = os.path.isfile(out_base + '.pgm') or os.path.isfile(out_base + '.yaml')
    # 大 PCD（建图走久会到 GB 级）必须体素降采样，否则 python 列表 OOM → 被杀 →「pgm not written」
    try:
        pcd_sz = os.path.getsize(pcd_path) if os.path.isfile(pcd_path) else 0
    except OSError:
        pcd_sz = 0
    # PCD 过旧时多半是上一次建图残留：仍允许转换，但打标给前端提示
    pcd_age = None
    try:
        pcd_age = time.time() - os.path.getmtime(pcd_path)
    except OSError:
        pass
    voxel = '0.05' if pcd_sz > 200 * 1024 * 1024 else '0.04'
    timeout = 600 if pcd_sz > 400 * 1024 * 1024 else 300
    band = load_obstacle_height_band()
    # 先写到临时目录再原子替换，避免转换失败时留下半截/旧图混用
    tmp_dir = tempfile.mkdtemp(prefix='pcd2pgm_', dir=MAPS_DIR)
    tmp_base = os.path.join(tmp_dir, name)
    cmd = [
        'python3', PCD2PGM, pcd_path, tmp_base,
        # Super-LIO 的 map.pcd 在重力对齐的 world 系（z 向上），
        # 期望地平面法向量为 +Z，故 pitch_down/roll/lidar_z 都取 0（由 RANSAC 精修高度）。
        # 障碍高度带与导航 stage2 /scan 共用 config/obstacle_height_band.yaml
        '--pitch-down', '0',
        '--roll', '0',
        '--lidar-z', '0',
        '--z-min', '%.3f' % band['z_min'],
        '--z-max', '%.3f' % band['z_max'],
        '--occ-min', '0.12', '--occ-pts', '8',
        '--free-pts', '1', '--ground-min-z', '-0.15',
        '--free-dilate', '8', '--free-erode', '2',
        '--despeckle', '3',
        '--voxel', voxel,
    ]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        log = (r.stdout + r.stderr).strip()
        tmp_pgm = tmp_base + '.pgm'
        tmp_yaml = tmp_base + '.yaml'
        ok = os.path.isfile(tmp_pgm) and os.path.isfile(tmp_yaml)
        if ok:
            for suf in ('.pgm', '.yaml'):
                os.replace(tmp_base + suf, out_base + suf)
            # 若 Nav2 正挂着同名图，热加载新文件，避免界面仍显示内存里的旧栅格
            reloaded = None
            cur = _nav2_current_map or ''
            if cur and os.path.basename(cur) == name + '.yaml':
                reloaded = reload_nav_map({'map': name + '.yaml', 'reloc': False})
            return {
                'ok': True,
                'name': name + '.pgm',
                'overwritten': overwritten,
                'pcd_age_sec': pcd_age,
                'reloaded': reloaded,
                'log': log[-2000:],
                'error': None,
            }
        err = log[-500:] if log else ''
        if r.returncode == -9 or r.returncode == 137:
            err = (err + ' | ').lstrip(' |') + (
                f'转换进程被系统杀掉(OOM)。PCD≈{pcd_sz/1e9:.2f}GB，已用 voxel={voxel}m；'
                '请缩短单次建图或再试一次保存')
        elif not err:
            err = f'pgm not written (exit={r.returncode})'
        return {'ok': False, 'name': name + '.pgm', 'log': log[-2000:], 'error': err}
    except subprocess.TimeoutExpired:
        return {'ok': False, 'error': f'PCD→PGM 超时(>{timeout}s)，点云过大，请缩短建图再保存'}
    except Exception as e:
        return {'ok': False, 'error': str(e)}
    finally:
        try:
            shutil.rmtree(tmp_dir, ignore_errors=True)
        except Exception:
            pass


def _scan_proc_alive():
    """Go2W stage2（默认）或 nav_scan / 旧 laserscan。"""
    return (
        proc_alive('stage2_pointcloud_to_laserscan')
        or proc_alive('nav_scan_node')
        or proc_alive('pointcloud_to_laserscan')
    )


def _go2w_odom_alive():
    """默认导航里程计：FAST-LIO + odom_filter（发布 /odom_nav + TF）。"""
    return proc_alive('lib/fast_lio/fastlio_mapping') and proc_alive('fastlio_nav2_odom_filter')


_scan_start_lock = threading.Lock()


def ensure_scan_node():
    # 默认 Go2W：localization 已起 stage2；有 FAST-LIO 时不要再起 nav_scan（抢 /scan、帧不对）
    # 必须全局锁：heal / start_nav2 / 定时巡检并发时会叠多个 stage2 → /scan 双发、costmap stale
    with _scan_start_lock:
        collapse_go2w_duplicates(keep_one=True)
        if proc_alive('stage2_pointcloud_to_laserscan'):
            return True
        if _go2w_odom_alive() or proc_alive('fastlio_mapping'):
            # stage2 应由 start_quad.sh localization 拉起；这里再兜底一次
            # stamp_mode:=tf：戳=最新 odom→base_footprint TF 时间-0.1s。now 戳总比 TF 新，Nav2 的 tf2 MessageFilter
            # 每帧都走 waitForTransform，Humble 下与 TF 监听线程互锁 → AMCL/costmap 永久收不到 /scan
            # /scan 必须 BEST_EFFORT（节点内 qos_profile_sensor_data），否则 Nav2 对不上
            band = load_obstacle_height_band()
            run_bg(
                'ros2 run isaac_go2_nav2 stage2_pointcloud_to_laserscan --ros-args '
                '-r __node:=stage2_pointcloud_to_laserscan '
                '-r cloud_in:=/cloud_registered -r scan:=/scan '
                '-p target_frame:=body -p output_frame:=base_footprint '
                '-p body_extrinsic_x:=-0.084303 -p body_extrinsic_y:=0.006341 '
                '-p body_extrinsic_z:=-0.507792 '
                '-p body_extrinsic_qx:=0.006523 -p body_extrinsic_qy:=-0.170463 '
                '-p body_extrinsic_qz:=0.000000 -p body_extrinsic_qw:=0.985343 '
                '-p use_tf:=true -p stamp_mode:=tf -p stamp_tf_frame:=odom -p stamp_tf_lag:=0.10 '
                '-p min_height:=%.3f -p max_height:=%.3f '
                '-p angle_increment:=0.00872664626 '
                '-p range_min:=0.25 -p range_max:=12.0 -p self_filter_enabled:=false '
                '-p isolate_filter_enabled:=false '
                '-p transform_tolerance:=0.50' % (band['z_min'], band['z_max']),
                '/tmp/scan_node.log')
            time.sleep(1.5)
            collapse_go2w_duplicates(keep_one=True)
            return _scan_proc_alive()
    if proc_alive('nav_scan_node'):
        return True
    if proc_alive('pointcloud_to_laserscan'):
        run_bg("pkill -f pointcloud_to_laserscan_node || true", '/tmp/scan_node.log')
        time.sleep(0.4)
    if os.path.isfile(START_SCAN):
        run_bg(f'bash {START_SCAN}', '/tmp/scan_node.log')
        time.sleep(1.5)
    return _scan_proc_alive()


_nav2_lock = threading.Lock()
_nav2_starting = False
_nav2_current_map = None  # 幂等启动用: 同一张图已在跑就别重新盲搜, 见 _start_nav2_impl
_reload_job = {
    'busy': False, 'ok': None, 'error': None,
    'map': None, 'map_pgm': None, 'hint': None, 'ts': 0.0,
}

_mapping_lock = threading.Lock()
_mapping_starting = False
# 跨设备共享的「建图会话」：任一端点「开始建图」置位，停止/保存/丢弃清掉。
# 另一端只靠本地 mappingSession 会永远显示空闲。
_mapping_session = False
# False=进程启动后尚未被 start/stop 显式写过；仅此时才允许用进程存活推断会话。
# 否则停止过程中 LIO 还在落盘时 status 轮询会把 session 又推断成 True，前端再同步/自动恢复，表现为「关不掉」。
_mapping_session_known = False
_mapping_stopping = False


def set_mapping_session(active):
    global _mapping_session, _mapping_session_known
    _mapping_session = bool(active)
    _mapping_session_known = True


_loc_heal_ts = 0.0


def _cmdline_pids(substr):
    """PIDs whose cmdline contains substr (skip agent/shell noise)."""
    out = []
    for pid, cmd in _proc_table():
        if 'extglob' in cmd or 'COMMAND_EXIT_CODE' in cmd or 'pgrep' in cmd:
            continue
        if substr in cmd:
            out.append(pid)
    return out


def collapse_lio_tf_bridge(keep_one=True):
    """多个 lio_tf_bridge 会抢发 odom→base_footprint，网页箭头/激光必晃。"""
    bins, launches = [], []
    for pid, cmd in _proc_table():
        if 'extglob' in cmd or 'COMMAND_EXIT_CODE' in cmd:
            continue
        if 'lio_tf_bridge/lio_tf_bridge' in cmd:
            bins.append(pid)
        elif 'ros2 launch' in cmd and 'lio_tf_bridge' in cmd:
            launches.append(pid)
    if not bins and not launches:
        return {'ok': True, 'kept': 0}
    if keep_one and len(bins) <= 1 and len(launches) <= 1:
        return {'ok': True, 'kept': len(bins)}
    # 只留最新的一个 node + 其 launch；其余杀掉
    bins.sort()
    launches.sort()
    keep_bin = bins[-1] if bins else None
    keep_launch = launches[-1] if launches else None
    for pid in bins + launches:
        if pid in (keep_bin, keep_launch):
            continue
        try:
            _kill(pid, signal.SIGKILL)
        except Exception:
            pass
    time.sleep(0.4)
    return {'ok': True, 'kept': 1 if keep_bin else 0, 'killed': True}


def collapse_go2w_duplicates(keep_one=True):
    """叠两套 FAST-LIO/odom_filter/stage2 → 非单调 stamp + TF 断树，Nav2 必挂。"""
    groups = {
        'fastlio_mapping': _cmdline_pids('fastlio_mapping'),
        'fastlio_nav2_odom_filter': _cmdline_pids('fastlio_nav2_odom_filter'),
        'stage2_pointcloud_to_laserscan': _cmdline_pids('stage2_pointcloud_to_laserscan'),
        'body_to_base_link_static': _cmdline_pids('body_to_base_link_static'),
        'odom_to_camera_init_static': _cmdline_pids('odom_to_camera_init_static'),
    }
    killed = 0
    for name, pids in groups.items():
        # ros2 run 会留下外层 python + 内层节点，只按可执行路径收紧
        real = []
        for pid in pids:
            try:
                with open(f'/proc/{pid}/cmdline', 'rb') as f:
                    cmd = f.read().replace(b'\0', b' ').decode('utf-8', 'ignore')
            except Exception:
                continue
            if 'ros2 run' in cmd or 'ros2 launch' in cmd:
                continue
            if name in cmd:
                real.append(pid)
        if keep_one and len(real) <= 1:
            continue
        real.sort()
        for pid in real[:-1] if keep_one and real else real:
            try:
                _kill(pid, signal.SIGKILL)
                killed += 1
            except Exception:
                pass
    # 多余的 localization 壳只留一个
    loc_shells = []
    for pid in _cmdline_pids('start_quad.sh'):
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode('utf-8', 'ignore')
        except Exception:
            continue
        if 'localization' in cmd:
            loc_shells.append(pid)
    if keep_one and len(loc_shells) > 1:
        loc_shells.sort()
        for pid in loc_shells[:-1]:
            try:
                _kill(pid, signal.SIGKILL)
                killed += 1
            except Exception:
                pass
    if killed:
        time.sleep(0.5)
    return {'ok': True, 'killed': killed}


def _mapping_launch_pids():
    """start_quad mapping / steel_coin mode:=mapping 外壳 PID。"""
    out = []
    for pid in _cmdline_pids('mode:=mapping'):
        out.append(pid)
    for pid in _cmdline_pids('start_quad.sh'):
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode('utf-8', 'ignore')
        except Exception:
            continue
        if 'localization' in cmd:
            continue
        if ' mapping' in cmd or cmd.rstrip().endswith('mapping'):
            out.append(pid)
    return out


def mapping_stack_busy():
    """建图会话是否占用雷达/LIO（含 launch 还在但预览节点已挂的情况）。"""
    if proc_alive('map_building_node') or proc_alive('super_lio_node'):
        return True
    return bool(_mapping_launch_pids())


def stop_mapping_launch_orphans():
    """清掉停止建图后残留的 mapping launch（不碰 localization / genisom）。"""
    killed = 0
    for pid in _mapping_launch_pids():
        try:
            _kill(pid, signal.SIGKILL)
            killed += 1
        except Exception:
            pass
    for pat in ('map_building_node', 'super_lio_node', 'Livox_mid360.py'):
        before = proc_pids(pat) if pat != 'Livox_mid360.py' else _cmdline_pids(pat)
        for pid in before:
            try:
                _kill(pid, signal.SIGKILL)
                killed += 1
            except Exception:
                pass
    # mapping launch 被 SIGKILL 后，它拉起的 livox 驱动会变成孤儿（父进程=init）继续占雷达；
    # 之后 localization 再起一个驱动 → 两个驱动抢 IMU，FAST-LIO「IMU and LiDAR not Synced」卡死
    # 且内存持续上涨（2026-10-01 实测）。只杀孤儿，不碰 localization 正在用的驱动。
    time.sleep(0.3)
    for pid in _livox_driver_pids():
        if _ppid(pid) == 1:
            try:
                _kill(pid, signal.SIGKILL)
                killed += 1
            except Exception:
                pass
    if killed:
        time.sleep(0.8)
    return {'ok': True, 'killed': killed}


def stop_localization_chain():
    """停导航定位链（livox / FAST-LIO / odom_filter / stage2 / 静态 TF / start_quad localization 外壳）。
    按可执行路径匹配；绝不碰 genisom_bridge（断 SDK 狗会趴下）。返回被杀进程数。"""
    killed = 0
    pats = ('start_quad.sh localization', 'lib/livox_ros_driver2/livox_ros_driver2_node',
            'msg_MID360s_launch', 'lib/fast_lio/fastlio_mapping', 'fast_lio mapping.launch',
            'lib/isaac_go2_nav2/fastlio_nav2_odom_filter', 'lib/isaac_go2_nav2/stage2_pointcloud_to_laserscan',
            'body_to_base_link_static')
    for pat in pats:
        for pid in proc_pids(pat):
            try:
                cmd = open(f'/proc/{pid}/cmdline', 'rb').read().replace(b'\0', b' ').decode('utf-8', 'ignore')
            except Exception:
                continue
            if 'genisom' in cmd or 'web_ops_node' in cmd:
                continue
            try:
                _kill(pid, signal.SIGTERM)
                killed += 1
            except Exception:
                pass
    if killed:
        time.sleep(1.5)
        for pat in pats:          # 还活着的补 SIGKILL
            for pid in proc_pids(pat):
                try:
                    _kill(pid, signal.SIGKILL)
                except Exception:
                    pass
    shutil.rmtree('/tmp/steel_coin_localization.lockdir', ignore_errors=True)
    if killed:
        _log_warn('mapping start: stopped localization chain (%d procs)' % killed)
    return killed


def restart_localization_stack():
    """手动重启定位：先停掉已发散的 FAST-LIO 链，再拉起一套。不碰 genisom_bridge。"""
    def _bg():
        try:
            if _web_ops_node is not None:
                _web_ops_node.cb_nav_cancel(Empty())
        except Exception:
            pass
        try:
            stop_localization_chain()
        except Exception as e:
            _log_warn('manual localization stop failed: %s' % e)
        run_bg(f'bash {START_ALL} localization', '/tmp/localization.log')
        _log_warn('manual localization restart')
    threading.Thread(target=_bg, daemon=True).start()
    return {'ok': True, 'started': True,
            'msg': '正在重启定位。起来后请重新设一次初始位姿'}


def _livox_driver_pids():
    return proc_pids('lib/livox_ros_driver2/livox_ros_driver2_node')


def _ppid(pid):
    try:
        with open(f'/proc/{pid}/stat') as f:
            return int(f.read().rsplit(')', 1)[1].split()[1])
    except Exception:
        return -1


# 本进程（web_ops_tf_listener 节点，raw 订阅不反序列化）最近一次收到 /Odometry 的时间
_odom_rx = {'ts': 0.0}


def _on_odom_raw(_msg):
    _odom_rx['ts'] = time.monotonic()


def _fastlio_output_alive(timeout=5.0):
    """新进程 echo 一次 /Odometry：FAST-LIO 进程在 ≠ 在出数据（IMU 不同步时会活着但卡死）。

    快速路径：本进程 3 s 内收到过 /Odometry 就算在出数据，不再起 ros2 CLI（每次单核满载 1~2 s，
    看门狗 30 s 一次）。本进程没收到时仍走原来的新进程探测，避免本进程「聋了」时误判卡死去重建定位链。"""
    if time.monotonic() - _odom_rx['ts'] < 3.0:
        return True
    try:
        r = subprocess.run(
            ['bash', '-c',
             'source /opt/ros/humble/setup.bash >/dev/null 2>&1; '
             'timeout %d ros2 topic echo --once --no-daemon /Odometry nav_msgs/msg/Odometry --field header.stamp.sec'
             % int(timeout)],
            capture_output=True, text=True, timeout=timeout + 4)
        return any(ch.isdigit() for ch in (r.stdout or ''))
    except Exception:
        return False


def ensure_localization_stack(force=False):
    """Nav2 默认依赖 Go2W：FAST-LIO + odom_filter(/odom_nav) + stage2(/scan)。

    钢镚机构/雷达安装≠Go2：localization 脚本里用本机 19.6° 静态 TF，stage2 关腿滤。
    start_quad.sh localization 会清掉 Super-LIO/lio_tf，避免双里程计抢 TF。
    """
    global _loc_heal_ts
    ensure_oem700_driver()
    collapse_lio_tf_bridge(keep_one=True)
    collapse_go2w_duplicates(keep_one=True)
    has = (_go2w_odom_alive() and proc_alive('livox_ros_driver2_node')
           and _scan_proc_alive())
    # 进程都在 ≠ 健康：双 livox 驱动（建图残留孤儿）或 FAST-LIO 卡死不出 /Odometry 都要重建
    if has and len(_livox_driver_pids()) > 1:
        _log_warn('localization: %d livox drivers running — rebuild' % len(_livox_driver_pids()))
        has = False
    if has and not force and not _fastlio_output_alive():
        _log_warn('localization: FAST-LIO alive but no /Odometry — rebuild')
        has = False
    # 重复实例即使 has=True 也要塌缩；塌缩后若断了再拉
    if has and not force:
        return {
            'fastlio': True,
            'lio_tf': True,
            'livox': True,
            'go2w': True,
            'scan': True,
        }
    need = (not has) or force
    if need:
        now = time.time()
        if force or (now - _loc_heal_ts) > 12.0:
            _loc_heal_ts = now
            stop_relocation_node()
            run_bg(f'bash {START_ALL} localization', '/tmp/localization.log')
            for _ in range(50):
                if (_go2w_odom_alive() and proc_alive('livox_ros_driver2_node')
                        and _scan_proc_alive()):
                    break
                time.sleep(0.5)
            collapse_go2w_duplicates(keep_one=True)
    return {
        'fastlio': _go2w_odom_alive() or proc_alive('fastlio_mapping')
                   or proc_alive('super_lio_node') or proc_alive('relocation_node'),
        'lio_tf': proc_alive('fastlio_nav2_odom_filter') or proc_alive('lio_tf_bridge'),
        'livox': proc_alive('livox_ros_driver2_node'),
        'go2w': _go2w_odom_alive(),
        'scan': _scan_proc_alive(),
    }


def heal_nav_deps():
    """If Nav2 is up but LIO/scan died, bring them back (no dog motion)."""
    if not proc_alive('component_container_isolated'):
        return
    collapse_lio_tf_bridge(keep_one=True)
    if not _go2w_odom_alive():
        ensure_localization_stack()
    else:
        _fastlio_stall_watchdog()
    if not _scan_proc_alive():
        ensure_scan_node()
    stop_legacy_reloc_procs()
    _nav_lifecycle_watchdog()


# Nav2 半启动：lifecycle_manager 激活途中某节点 bond 超时就整体 Abort，排在后面的
# bt_navigator / velocity_smoother 停在 inactive → 目标一律被拒（网页「导航失败」）。
# 2026-10-01：开导航时 FAST-LIO 卡死无 odom TF，controller 激活卡 4 min，随后 smoother bond 超时。
_NAV_LATE_NODES = ('bt_navigator', 'velocity_smoother')
_nav_lc = {'ts': 0.0}


def _container_age_sec():
    for pid in proc_pids('component_container_isolated'):
        try:
            with open(f'/proc/{pid}/stat') as f:
                start_ticks = int(f.read().rsplit(')', 1)[1].split()[19])
            with open('/proc/uptime') as f:
                up = float(f.read().split()[0])
            return up - start_ticks / os.sysconf('SC_CLK_TCK')
        except Exception:
            continue
    return 0.0


def ensure_nav_lifecycle_active():
    """把停在 inactive 的 bt_navigator / velocity_smoother 激活（不动狗、不丢 AMCL 定位）。返回被激活的节点名。"""
    fixed = []
    for nd in _NAV_LATE_NODES:
        try:
            r = subprocess.run(
                ['bash', '-c', 'source /opt/ros/humble/setup.bash >/dev/null 2>&1; '
                               'timeout 8 ros2 lifecycle get --no-daemon /%s' % nd],
                capture_output=True, text=True, timeout=12)
            if 'inactive' not in (r.stdout or ''):
                continue
            r = subprocess.run(
                ['bash', '-c', 'source /opt/ros/humble/setup.bash >/dev/null 2>&1; '
                               'timeout 20 ros2 lifecycle set --no-daemon /%s activate' % nd],
                capture_output=True, text=True, timeout=25)
            if 'successful' in (r.stdout or ''):
                fixed.append(nd)
        except Exception:
            pass
    if fixed:
        _log_warn('Nav2 half-started: activated %s' % ', '.join(fixed))
    return fixed


def _nav_lifecycle_watchdog():
    now = time.time()
    if now - _nav_lc['ts'] < 30.0:
        return
    age = _container_age_sec()
    # 只在 bringup 窗口（启动后 90 s~10 min）查；之后节点不会自己掉 inactive，目标被拒时另有即时检查。
    # 每次查要起 ros2 CLI（单核满载 1~2 s），常驻轮询太费 CPU。
    if age < 90.0 or age > 600.0:
        return
    _nav_lc['ts'] = now
    ensure_nav_lifecycle_active()


_fastlio_stall = {'ts': 0.0, 'fails': 0}


def _fastlio_stall_watchdog():
    """FAST-LIO 进程在但不出 /Odometry（IMU 不同步卡死、内存持续涨）→ 连续 2 次（约 60 s）后重建定位链。
    导航中重建会让里程计归零，需重设初姿；但卡死时导航本来就不可用。"""
    now = time.time()
    if now - _fastlio_stall['ts'] < 30.0:
        return
    _fastlio_stall['ts'] = now
    if _fastlio_output_alive():
        _fastlio_stall['fails'] = 0
        return
    _fastlio_stall['fails'] += 1
    if _fastlio_stall['fails'] < 2:
        return
    _fastlio_stall['fails'] = 0
    _log_warn('FAST-LIO stalled (no /Odometry ~60s) — rebuild localization stack; re-set initial pose')
    ensure_localization_stack(force=True)


def _log_warn(msg):
    try:
        if _web_ops_node is not None:
            _web_ops_node.get_logger().warn(msg)
    except Exception:
        pass


def stop_nav2_procs():
    for pat in ['component_container_isolated', 'bringup_launch.py',
                'nav2_bringup', 'bt_navigator', 'controller_server',
                'planner_server', 'behavior_server', 'waypoint_follower',
                'velocity_smoother', 'lifecycle_manager', 'auto_relocalize',
                'bbs3d_ros2_node', 'bbs_global_pose_bridge',
                'global_pose_arbiter']:
        kill_pattern(pat, signal.SIGKILL)
    time.sleep(1.2)


def ensure_oem700_driver():
    """NMEA only. Does not publish map→odom."""
    if proc_alive('oem700_driver'):
        return
    run_bg(
        'ros2 run oem700_rtk oem700_driver --ros-args -r __node:=oem700_driver',
        '/tmp/oem700.log')


def _rtk_nav2_params(map_path):
    """Return (params_yaml, fusion_on). Fusion requires maps/<stem>.datum.yaml."""
    try:
        from oem700_rtk.fusion_launch import fusion_requested, write_nav2_params
    except Exception as exc:
        _log_warn('oem700_rtk import failed (%s); AMCL keeps map→odom' % exc)
        return NAV2_PARAMS, False
    fuse = fusion_requested(map_path, 'auto')
    try:
        return write_nav2_params(NAV2_PARAMS, fuse), fuse
    except Exception as exc:
        _log_warn('RTK params rewrite failed: %s' % exc)
        return NAV2_PARAMS, False


def ensure_rtk_fusion(map_path, fuse):
    """Start projection + arbiter only when this nav session owns map→odom."""
    ensure_oem700_driver()
    if not fuse:
        if proc_alive('global_pose_arbiter'):
            kill_pattern('global_pose_arbiter', signal.SIGKILL)
        return
    if proc_alive('rtk_to_map'):
        kill_pattern('rtk_to_map', signal.SIGKILL)
    if proc_alive('global_pose_arbiter'):
        kill_pattern('global_pose_arbiter', signal.SIGKILL)
    time.sleep(0.3)
    run_bg(
        'ros2 run oem700_rtk rtk_to_map --ros-args -r __node:=rtk_to_map '
        '-p map_yaml:=%s' % map_path,
        '/tmp/oem700_map.log')
    run_bg(
        'ros2 run oem700_rtk global_pose_arbiter --ros-args '
        '-r __node:=global_pose_arbiter',
        '/tmp/oem700_arbiter.log')


def stop_legacy_reloc_procs():
    """停掉已废弃的 2D auto_relocalize / 3D-BBS（若仍在跑）。"""
    for pat in ('auto_relocalize', 'bbs3d_ros2_node', 'bbs_global_pose_bridge',
                'livox_cloud_to_pc2'):
        kill_pattern(pat, signal.SIGKILL)


def ensure_auto_relocalize():
    """已废弃：不再启动 2D 盲搜。旧调用只清理残留。"""
    stop_legacy_reloc_procs()
    return False


_web_ops_node = None  # set in main(); HTTP handlers use it to gate nav
_cmd_vel_lock = threading.Lock()
_cmd_vel_seq = 0  # 已接受的最大 seq；用于丢弃乱序晚到的非零指令
_cmd_vel_stop_mono = 0.0  # 最近一次零速的 monotonic 时间


def _read_p5_pgm(path):
    """Return (width, height, raw_bytes) for a P5 PGM."""
    with open(path, 'rb') as f:
        raw = f.read()
    i = 0
    n = len(raw)

    def tok():
        nonlocal i
        while i < n:
            if raw[i:i + 1] == b'#':
                while i < n and raw[i:i + 1] != b'\n':
                    i += 1
                continue
            if raw[i] in (9, 10, 13, 32):
                i += 1
                continue
            break
        s = b''
        while i < n and raw[i] not in (9, 10, 13, 32) and raw[i:i + 1] != b'#':
            s += raw[i:i + 1]
            i += 1
        return s.decode('ascii', 'replace')

    if tok() != 'P5':
        raise ValueError('not a P5 PGM')
    w, h, _mv = int(tok()), int(tok()), int(tok())
    while i < n and raw[i] in (9, 10, 13, 32):
        i += 1
    pix = raw[i:i + w * h]
    if len(pix) < w * h:
        raise ValueError('truncated PGM')
    return w, h, pix


def _parse_map_yaml_meta(yaml_path):
    """Parse nav map yaml → resolution/origin/thresholds/image stem."""
    meta = {
        'resolution': 0.05,
        'origin': [0.0, 0.0, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
        'image': None,
    }
    if not os.path.isfile(yaml_path):
        return meta
    for line in open(yaml_path, encoding='utf-8', errors='replace'):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        m = re.match(r'image:\s*(\S+)', line)
        if m:
            meta['image'] = os.path.basename(m.group(1).strip().strip('"\''))
            continue
        m = re.match(r'resolution:\s*([\d.eE+-]+)', line)
        if m:
            meta['resolution'] = float(m.group(1))
            continue
        m = re.match(r'origin:\s*\[([^,]+),\s*([^,\]]+)(?:,\s*([^\]]+))?\]', line)
        if m:
            meta['origin'] = [
                float(m.group(1)), float(m.group(2)),
                float(m.group(3)) if m.group(3) else 0.0]
            continue
        m = re.match(r'negate:\s*(\d+)', line)
        if m:
            meta['negate'] = int(m.group(1))
            continue
        m = re.match(r'occupied_thresh:\s*([\d.eE+-]+)', line)
        if m:
            meta['occupied_thresh'] = float(m.group(1))
            continue
        m = re.match(r'free_thresh:\s*([\d.eE+-]+)', line)
        if m:
            meta['free_thresh'] = float(m.group(1))
            continue
    return meta


def occupancy_map_from_disk(yaml_path):
    """Load OccupancyGrid-shaped dict from maps/*.yaml+pgm（与 map_server 规则一致）。

    当 web_ops 的 /map 订阅没接到 TRANSIENT_LOCAL（节点掉发现、换图清缓存、
    Nav2 晚于网页启动等）时，HTTP 仍能把导航底图送给前端。
    """
    yaml_path = os.path.abspath(yaml_path)
    if not os.path.isfile(yaml_path):
        return None
    meta = _parse_map_yaml_meta(yaml_path)
    img_name = meta.get('image') or (os.path.basename(yaml_path)[:-5] + '.pgm')
    if not img_name.endswith('.pgm'):
        img_name += '.pgm'
    pgm_path = os.path.join(os.path.dirname(yaml_path), os.path.basename(img_name))
    if not os.path.isfile(pgm_path):
        return None
    try:
        w, h, pix = _read_p5_pgm(pgm_path)
    except Exception:
        return None
    occ_th = float(meta['occupied_thresh'])
    free_th = float(meta['free_thresh'])
    negate = int(meta['negate'] or 0)
    data = [0] * (w * h)
    for i, v in enumerate(pix):
        # nav2 map_io: negate=0 → occ=(255-color)/255
        color = int(v)
        occ = (color / 255.0) if negate else ((255 - color) / 255.0)
        # PGM row0 = 图像顶 = 高 world-Y；OccupancyGrid row0 = origin = 低 world-Y。
        # map_server 加载时按行翻转，这里必须一致，否则 HTTP 兜底底图上下镜像。
        r, c = divmod(i, w)
        j = (h - 1 - r) * w + c
        if occ > occ_th:
            data[j] = 100
        elif occ < free_th:
            data[j] = 0
        else:
            data[j] = -1
    ox, oy, oz = meta['origin']
    return {
        'info': {
            'width': w,
            'height': h,
            'resolution': float(meta['resolution']),
            'origin': {
                'position': {'x': float(ox), 'y': float(oy), 'z': float(oz)},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            },
        },
        'data': data,
    }


def current_map_json(requested=''):
    """/map 的 HTTP 直取通道，绕开 rosbridge 对该话题的订阅时序问题（见 cb_map 注释）。

    导航在跑时用节点缓存的 /map。导航没开时按网页选中的图读磁盘，
    不再固定回退到 map_live。
    """
    nav_running = proc_alive('component_container_isolated')
    node = _web_ops_node
    msg = node._last_map if node is not None else None
    if nav_running and msg is not None:
        o = msg.info.origin
        return {
            'ok': True,
            'source': 'ros',
            'map': {
                'info': {
                    'width': msg.info.width,
                    'height': msg.info.height,
                    'resolution': msg.info.resolution,
                    'origin': {
                        'position': {'x': o.position.x, 'y': o.position.y, 'z': o.position.z},
                        'orientation': {'x': o.orientation.x, 'y': o.orientation.y,
                                        'z': o.orientation.z, 'w': o.orientation.w},
                    },
                },
                'data': list(msg.data),
            },
            'map_file': os.path.basename(_nav2_current_map) if _nav2_current_map else None,
        }
    yaml_path = _resolve_nav_map(requested) if requested else None
    if not yaml_path and nav_running:
        yaml_path = _nav2_current_map if _nav2_current_map and os.path.isfile(_nav2_current_map) else None
    if not yaml_path and not nav_running:
        yaml_path = _resolve_nav_map(read_nav_pick())
    if not yaml_path:
        yaml_path = _resolve_nav_map('map_live.yaml')
    if not yaml_path:
        yamls = _list_nav_map_yamls()
        yaml_path = os.path.join(MAPS_DIR, yamls[0]) if yamls else None
    disk = occupancy_map_from_disk(yaml_path) if yaml_path else None
    if disk is None:
        return {'ok': False, 'error': '地图还没加载（Nav2/map_server 可能还没起来，磁盘也无可用图）'}
    return {'ok': True, 'source': 'disk', 'map': disk,
            'map_file': os.path.basename(yaml_path) if yaml_path else None}


def _xyyaw_twist(body):
    t = Twist()
    t.linear.x = float(body['x'])
    t.linear.y = float(body['y'])
    t.angular.z = float(body.get('yaw', 0.0))
    return t


def api_init_pose(body):
    """HTTP 设初姿：rosbridge 首次向新话题 publish 常在 DDS 匹配前丢包（实测 rosbridge 重启后
    /web/init_pose 一条都没到），网页改走 HTTP，rosbridge 仅作兜底。"""
    node = _web_ops_node
    if node is None:
        return {'ok': False, 'error': 'web_ops 节点未就绪'}
    try:
        node.cb_init_pose(_xyyaw_twist(body))
    except (KeyError, TypeError, ValueError) as e:
        return {'ok': False, 'error': f'参数错误: {e}'}
    return {'ok': True}


def api_nav_goal(body):
    """HTTP 下发 Nav2 目标；cb_nav_cmd 会等 action server（≤2 s）→ 后台线程，不堵 HTTP。"""
    node = _web_ops_node
    if node is None:
        return {'ok': False, 'error': 'web_ops 节点未就绪'}
    try:
        msg = _xyyaw_twist(body)
    except (KeyError, TypeError, ValueError) as e:
        return {'ok': False, 'error': f'参数错误: {e}'}
    threading.Thread(target=node.cb_nav_cmd, args=(msg,), daemon=True).start()
    return {'ok': True, 'queued': True}


def api_nav_goals(body):
    """单个目标走 NavigateToPose；两个及以上走 NavigateThroughPoses。"""
    node = _web_ops_node
    if node is None:
        return {'ok': False, 'error': 'web_ops 节点未就绪'}
    raw = body.get('poses') if isinstance(body, dict) else None
    if not isinstance(raw, list) or not raw:
        return {'ok': False, 'error': '没有目标点'}
    poses = []
    try:
        for p in raw:
            poses.append((float(p['x']), float(p['y']), float(p.get('yaw', 0.0))))
    except (KeyError, TypeError, ValueError) as e:
        return {'ok': False, 'error': f'参数错误: {e}'}
    if len(poses) == 1:
        msg = _xyyaw_twist({'x': poses[0][0], 'y': poses[0][1], 'yaw': poses[0][2]})
        threading.Thread(target=node.cb_nav_cmd, args=(msg,), daemon=True).start()
        return {'ok': True, 'queued': True, 'count': 1}
    threading.Thread(target=node.send_through_poses, args=(poses,), daemon=True).start()
    return {'ok': True, 'queued': True, 'count': len(poses)}


CRUISE_DIR = '/home/linaro/steel_coin_nav_ws/cruises'


def _cruise_safe(text):
    name = str(text or '').strip().replace('\\', '').replace('/', '')
    name = name.replace('..', '')
    if not name or name in ('.', '..'):
        raise ValueError('名称无效')
    return name[:40]


def _cruise_map_key(map_name):
    base = os.path.basename(str(map_name or '').strip())
    for ext in ('.yaml', '.pgm', '.png'):
        if base.endswith(ext):
            base = base[: -len(ext)]
    return _cruise_safe(base or 'map')


def _cruise_kind(kind):
    if kind not in ('points', 'routes'):
        raise ValueError('类型无效')
    return kind


def _cruise_dir(map_name, kind):
    path = os.path.join(CRUISE_DIR, _cruise_map_key(map_name), _cruise_kind(kind))
    os.makedirs(path, exist_ok=True)
    return path


def _cruise_doc_path(map_name, kind, name):
    return os.path.join(_cruise_dir(map_name, kind), _cruise_safe(name) + '.json')


def cruise_list(map_name):
    out = {'ok': True, 'map': _cruise_map_key(map_name), 'points': [], 'routes': []}
    for kind, key in (('points', 'points'), ('routes', 'routes')):
        folder = os.path.join(CRUISE_DIR, out['map'], kind)
        if not os.path.isdir(folder):
            continue
        for fn in sorted(os.listdir(folder)):
            if not fn.endswith('.json'):
                continue
            try:
                with open(os.path.join(folder, fn), 'r', encoding='utf-8') as f:
                    doc = json.load(f)
            except Exception:
                continue
            if isinstance(doc, dict):
                out[key].append(doc)
    return out


def cruise_save(body):
    if not isinstance(body, dict):
        return {'ok': False, 'error': '参数错误'}
    try:
        kind = _cruise_kind('points' if body.get('kind') == 'points' else 'routes' if body.get('kind') == 'routes' else '')
        name = _cruise_safe(body.get('name'))
        map_name = body.get('map') or ''
        _cruise_map_key(map_name)
    except ValueError as e:
        return {'ok': False, 'error': str(e)}
    if kind == 'points':
        raw = body.get('poses') if isinstance(body.get('poses'), list) else []
        poses = []
        try:
            for p in raw:
                poses.append({
                    'x': round(float(p['x']), 3),
                    'y': round(float(p['y']), 3),
                    'yaw': round(float(p.get('yaw', 0.0)), 4),
                })
        except (KeyError, TypeError, ValueError):
            return {'ok': False, 'error': '巡航点格式不对'}
        if not poses:
            return {'ok': False, 'error': '至少要有一个巡航点'}
        doc = {'name': name, 'map': _cruise_map_key(map_name), 'kind': 'points', 'poses': poses}
    else:
        raw = body.get('vertices') if isinstance(body.get('vertices'), list) else []
        vertices = []
        try:
            for p in raw:
                vertices.append({'x': round(float(p['x']), 3), 'y': round(float(p['y']), 3)})
        except (KeyError, TypeError, ValueError):
            return {'ok': False, 'error': '路线顶点格式不对'}
        if len(vertices) < 2:
            return {'ok': False, 'error': '路线至少两个点'}
        doc = {'name': name, 'map': _cruise_map_key(map_name), 'kind': 'routes', 'vertices': vertices}
    path = _cruise_doc_path(map_name, kind, name)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(doc, f, ensure_ascii=False, indent=2)
    return {'ok': True, 'name': name, 'kind': kind}


def cruise_delete(body):
    if not isinstance(body, dict):
        return {'ok': False, 'error': '参数错误'}
    try:
        path = _cruise_doc_path(body.get('map') or '', _cruise_kind(body.get('kind')), body.get('name'))
    except ValueError as e:
        return {'ok': False, 'error': str(e)}
    if os.path.isfile(path):
        os.remove(path)
    return {'ok': True}


def _sample_polyline(vertices, step=0.2):
    """折线按步长加密，朝向沿切线。返回 [(x, y, yaw), ...]。"""
    pts = []
    for p in vertices:
        pts.append((float(p['x']), float(p['y'])))
    if len(pts) < 2:
        return []
    out = []
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        seg = math.hypot(x1 - x0, y1 - y0)
        yaw = math.atan2(y1 - y0, x1 - x0)
        n = max(1, int(math.ceil(seg / step)))
        for k in range(n):
            t = k / float(n)
            out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, yaw))
    x, y = pts[-1]
    out.append((x, y, out[-1][2]))
    return out


def api_cruise_start_route(body):
    node = _web_ops_node
    if node is None:
        return {'ok': False, 'error': 'web_ops 节点未就绪'}
    raw = body.get('vertices') if isinstance(body, dict) else None
    if not isinstance(raw, list) or len(raw) < 2:
        return {'ok': False, 'error': '路线至少两个点'}
    try:
        vertices = [{'x': float(p['x']), 'y': float(p['y'])} for p in raw]
    except (KeyError, TypeError, ValueError) as e:
        return {'ok': False, 'error': '参数错误: %s' % e}
    threading.Thread(target=node.start_route_cruise, args=(vertices,), daemon=True).start()
    return {'ok': True, 'queued': True, 'count': len(vertices)}


def accept_reloc_pose():
    """用户承认当前位姿 / 手动设姿完成 → 放行导航（Go2W：初姿由人给）。"""
    if _web_ops_node is not None:
        _web_ops_node._loc_user_accepted = True
        _web_ops_node._loc_aligned = True
        _web_ops_node._loc_hit = max(_web_ops_node._loc_hit, 0.45)
    return {'ok': True, 'error': None}


def call_relocalize(timeout=35.0):
    """已移除 2D 盲搜。"""
    return {
        'ok': False, 'removed': True,
        'error': '已改为 Go2W 初姿流程：请点「设初始位姿」在地图上拖出朝向',
    }


def global_relocalize(timeout=20.0):
    """已移除 3D-BBS 全局搜。"""
    return {
        'ok': False, 'removed': True,
        'error': '3D-BBS 全局重定位已移除；请用地图「设初始位姿」',
    }


def set_reloc_mode(body):
    """已移除 auto_relocalize 看门狗模式。"""
    return {
        'ok': True, 'mode': 'manual', 'removed': True,
        'note': '请用地图「设初始位姿」；不再支持自动盲搜模式',
    }


def _nav_yaml_is_occupancy(path):
    """True only for Nav2 occupancy maps (.pgm sibling + image/resolution).

    maps/ 里还有 bbs_dog.yaml 等 3D-BBS 配置（无 .pgm、无 image:），绝不能当导航图。
    """
    if not path or not os.path.isfile(path):
        return False
    try:
        text = open(path, 'r', encoding='utf-8', errors='ignore').read(4000)
    except Exception:
        return False
    if 'target_clouds' in text or 'lidar_topic_name' in text:
        return False
    if not re.search(r'(?m)^image\s*:', text):
        return False
    if not re.search(r'(?m)^resolution\s*:', text):
        return False
    stem = os.path.basename(path)
    if stem.endswith('.yaml'):
        stem = stem[:-5]
    return os.path.isfile(os.path.join(os.path.dirname(path), stem + '.pgm'))


def _resolve_nav_map(name):
    """Resolve UI/API map name → absolute nav yaml path, or None if not a real library map."""
    m = os.path.basename(str(name or '').strip())
    if not m:
        return None
    if m.endswith('.pgm'):
        m = m[:-4] + '.yaml'
    elif not m.endswith('.yaml'):
        m += '.yaml'
    path = os.path.join(MAPS_DIR, m)
    if _nav_yaml_is_occupancy(path):
        return path
    return None


def _list_nav_map_yamls():
    if not os.path.isdir(MAPS_DIR):
        return []
    out = []
    for f in sorted(os.listdir(MAPS_DIR)):
        if not f.endswith('.yaml') or f.startswith('_'):
            continue
        if _nav_yaml_is_occupancy(os.path.join(MAPS_DIR, f)):
            out.append(f)
    return out


def reload_nav_map(body):
    """换图：校验后后台重启 Nav2，HTTP 立刻返回，避免网页一直等像死机。"""
    global _nav2_current_map, _reload_job
    requested = body.get('map') or body.get('name') or ''
    map_path = _resolve_nav_map(requested)
    if not map_path:
        return {
            'ok': False,
            'error': '地图不可用或不在地图里：「%s」（需要同名 .pgm + 导航 yaml）'
                     % (os.path.basename(str(requested)) or '?'),
        }
    m = os.path.basename(map_path)
    map_pgm = m[:-5] + '.pgm' if m.endswith('.yaml') else m
    if not proc_alive('component_container_isolated'):
        _nav2_current_map = None
        return {
            'ok': True, 'map': m, 'map_pgm': map_pgm, 'deferred': True,
            'hint': 'Nav2 未运行，已记下「%s」，请点启动导航' % map_pgm,
        }
    if _reload_job.get('busy') or _nav2_starting:
        return {'ok': False, 'error': '正在切换/启动导航，请稍候再试'}

    def _work():
        global _nav2_current_map, _reload_job
        try:
            if _web_ops_node is not None:
                _web_ops_node._last_map = None
            stop_nav2_procs()
            _nav2_current_map = None
            time.sleep(1.0)
            r = start_nav2({'map': m})
            if r.get('ok'):
                _reload_job.update({
                    'busy': False, 'ok': True, 'error': None,
                    'map': m, 'map_pgm': map_pgm,
                    'hint': r.get('hint') or ('已切换到「%s」' % map_pgm),
                    'ts': time.time(),
                })
            else:
                _reload_job.update({
                    'busy': False, 'ok': False,
                    'error': r.get('error') or '换图失败',
                    'map': m, 'map_pgm': map_pgm, 'hint': None, 'ts': time.time(),
                })
        except Exception as e:
            _reload_job.update({
                'busy': False, 'ok': False, 'error': str(e),
                'map': m, 'map_pgm': map_pgm, 'hint': None, 'ts': time.time(),
            })

    _reload_job = {
        'busy': True, 'ok': None, 'error': None,
        'map': m, 'map_pgm': map_pgm,
        'hint': '正在切换到「%s」…' % map_pgm, 'ts': time.time(),
    }
    threading.Thread(target=_work, daemon=True).start()
    return {
        'ok': True, 'accepted': True, 'busy': True,
        'map': m, 'map_pgm': map_pgm,
        'hint': '正在切换到「%s」，约十几秒…' % map_pgm,
    }

def stop_nav2_api():
    global _nav2_current_map
    stop_nav2_procs()
    _nav2_current_map = None
    return {'ok': True}


# ---- 导航速度（设置页可调；在线改 RPP + velocity_smoother，不重启 Nav2）----
NAV_SPEED_FILE = '/home/linaro/steel_coin_nav_ws/config/nav_speed.json'
# brave（勇敢模式）：RPP 碰撞预判 0.1 s（只在机身轮廓压到障碍格时停）；关 = 0.6 s（提前 ~10 cm 停）。
# 2026-10-02 用户要求默认开启、自担风险（人旁 0.8 m 缝也要沿全局路线过去）。
NAV_SPEED_DEFAULT = {'linear': 0.22, 'angular': 0.45, 'brave': True}
# 上限 0.45：狗头贴墙 0.5 m 内雷达有盲区（FAST-LIO blind），再快刹车余量不够
NAV_SPEED_RANGE = {'linear': (0.10, 0.45), 'angular': (0.25, 0.80)}
_nav_speed_state = {'applied': None, 'applying': False, 'error': '', 'ts': 0.0}
_nav_speed_lock = threading.Lock()


def _clamp_speed(sp):
    out = {}
    for k, (lo, hi) in NAV_SPEED_RANGE.items():
        try:
            v = float(sp.get(k, NAV_SPEED_DEFAULT[k]))
        except (TypeError, ValueError, AttributeError):
            v = NAV_SPEED_DEFAULT[k]
        out[k] = round(min(hi, max(lo, v)), 3)
    try:
        out['brave'] = bool(sp.get('brave', NAV_SPEED_DEFAULT['brave']))
    except AttributeError:
        out['brave'] = NAV_SPEED_DEFAULT['brave']
    return out


def load_nav_speed():
    try:
        with open(NAV_SPEED_FILE, encoding='utf-8') as f:
            return _clamp_speed(json.load(f))
    except Exception:
        return dict(NAV_SPEED_DEFAULT)


def _nav_speed_yaml(sp):
    v, w = sp['linear'], sp['angular']
    # 前视距离随速度变长，否则快了会蛇行。不能短于 0.45：狗转向响应慢，0.30 时进门 90° 弯
    # 转晚了冲过头、贴到门框卡死（2026-10-02 实测）；也别像旧公式 2×v 那样 0.32 就给到 0.64。
    look = round(min(0.9, max(0.45, v * 1.4)), 2)
    ctrl = '\n'.join([
        '/controller_server:',
        '  ros__parameters:',
        '    FollowPath:',
        '      max_lookahead_dist: %.2f' % max(0.7, look + 0.1),
        '      lookahead_dist: %.2f' % look,
        '      desired_linear_vel: %.3f' % v,
        '      rotate_to_heading_angular_vel: %.3f' % w,
        '      max_allowed_time_to_collision_up_to_carrot: %.1f' % (0.1 if sp.get('brave', True) else 0.6),
        ''])
    smooth = '\n'.join([
        '/velocity_smoother:',
        '  ros__parameters:',
        '    max_velocity: [%.3f, 0.14, %.3f]' % (v + 0.02, w),
        '    min_velocity: [0.0, -0.14, -%.3f]' % w,
        ''])
    return ctrl, smooth


def _apply_nav_speed(sp):
    """ros2 param load 两次（controller_server / velocity_smoother）。成功返回 ''，否则错误文本。"""
    ctrl, smooth = _nav_speed_yaml(sp)
    errs = []
    for node, text in (('/controller_server', ctrl), ('/velocity_smoother', smooth)):
        path = '/tmp/nav_speed_%s.yaml' % node.strip('/')
        with open(path, 'w', encoding='utf-8') as f:
            f.write(text)
        try:
            r = subprocess.run(
                ['bash', '-c', 'source /opt/ros/humble/setup.bash >/dev/null 2>&1; '
                 'timeout 20 ros2 param load --no-daemon %s %s' % (node, path)],
                capture_output=True, text=True, timeout=30)
            out = (r.stdout or '') + (r.stderr or '')
            if r.returncode != 0 or 'Failed' in out or 'successful' not in out:
                errs.append('%s: %s' % (node, out.strip()[-200:] or 'rc=%d' % r.returncode))
        except Exception as e:
            errs.append('%s: %s' % (node, e))
    return '; '.join(errs)


def _apply_nav_speed_bg(sp, wait_ready=0.0):
    with _nav_speed_lock:
        if _nav_speed_state['applying']:
            return
        _nav_speed_state['applying'] = True
    try:
        deadline = time.time() + wait_ready
        while True:
            err = _apply_nav_speed(sp)
            if not err or time.time() >= deadline:
                break
            time.sleep(5.0)
        _nav_speed_state.update(applied=None if err else dict(sp), error=err, ts=time.time())
        if err:
            _log_warn('nav speed apply failed: %s' % err)
    finally:
        _nav_speed_state['applying'] = False


def api_nav_speed_get():
    return {'ok': True, 'speed': load_nav_speed(), 'default': NAV_SPEED_DEFAULT,
            'range': NAV_SPEED_RANGE, 'applied': _nav_speed_state['applied'],
            'applying': _nav_speed_state['applying'], 'error': _nav_speed_state['error'],
            'nav_running': proc_alive('component_container_isolated')}


def api_nav_speed_set(body):
    sp = _clamp_speed(body or {})
    try:
        with open(NAV_SPEED_FILE, 'w', encoding='utf-8') as f:
            json.dump(sp, f)
    except Exception as e:
        return {'ok': False, 'error': '保存失败: %s' % e}
    if not proc_alive('component_container_isolated'):
        return {'ok': True, 'speed': sp, 'applied_now': False, 'hint': '已保存，下次启动导航时生效'}
    threading.Thread(target=_apply_nav_speed_bg, args=(sp,), daemon=True).start()
    return {'ok': True, 'speed': sp, 'applied_now': True, 'hint': '已保存，正在应用（约 5 秒）'}


def start_nav2(body):
    global _nav2_starting
    with _nav2_lock:
        if _nav2_starting:
            return {'ok': False, 'error': '导航正在启动中，请稍候'}
        _nav2_starting = True
    try:
        res = _start_nav2_impl(body)
        if isinstance(res, dict) and res.get('ok'):
            sp = load_nav_speed()
            if sp != NAV_SPEED_DEFAULT:
                # Nav2 刚起来时 controller_server 可能还没 active：最多重试 120 s
                threading.Thread(target=_apply_nav_speed_bg, args=(sp, 120.0), daemon=True).start()
        return res
    finally:
        _nav2_starting = False


def _start_nav2_impl(body):
    requested = body.get('map', '') or ''
    map_path = _resolve_nav_map(requested) if requested else None
    if requested and not map_path:
        return {
            'ok': False,
            'error': '缺少地图：「%s」（库中无同名 .pgm + yaml，请先建图保存或重选）'
                     % os.path.basename(str(requested)),
        }
    if not map_path:
        # 未指定时才回退；绝不能把 bbs_dog.yaml 等非占用地图当默认
        for cand in ('map_live.yaml', 'map.yaml'):
            map_path = _resolve_nav_map(cand)
            if map_path:
                break
        if not map_path:
            yamls = _list_nav_map_yamls()
            if not yamls:
                return {'ok': False, 'error': '缺少导航地图：还没有地图，请先建图并保存'}
            map_path = os.path.join(MAPS_DIR, yamls[0])
    m = os.path.basename(map_path)
    if not os.path.isfile(map_path):
        return {'ok': False, 'error': '缺少地图：%s' % m}

    # 幂等: 已经用同一张图跑着就别整套拆了重来 —— 全局重定位是"开盲盒"，
    # 反复点「启动导航」会拿一次刚配准好的定位去赌一次新的盲搜，可能越点越偏。
    global _nav2_current_map
    if (proc_alive('component_container_isolated')
            and _nav2_current_map == map_path):
        ensure_scan_node()
        stop_legacy_reloc_procs()
        return {
            'ok': True, 'map': m,
            'map_pgm': (m[:-5] + '.pgm') if m.endswith('.yaml') else m,
            'already_running': True,
            'hint': '导航已经在跑（同一张图）；定位不对请「设初始位姿」',
            'auto_reloc': False,
        }
    # 建图/导航二选一：会话已关但仍有 mapping launch 残留 → 自动清掉再开导航
    if mapping_stack_busy():
        if _mapping_session:
            return {
                'ok': False,
                'error': '正在建图中，请先「停止建图」后再启动导航',
            }
        stop_mapping_launch_orphans()
        if mapping_stack_busy():
            return {
                'ok': False,
                'error': '建图进程未能退出，请再点一次「停止建图」或刷新后重试',
            }
    _nav2_current_map = map_path

    # 新启导航：清掉上次的「定位已确认」，等自检重定位结果
    if _web_ops_node is not None:
        _web_ops_node._loc_user_accepted = False
        _web_ops_node._loc_aligned = False
        _web_ops_node._loc_hit = 0.0

    # Singleton Nav2 — previous double-launch left two navigate_to_pose servers
    stop_nav2_procs()

    loc = ensure_localization_stack()
    if not loc.get('go2w') and not _go2w_odom_alive():
        return {
            'ok': False,
            'error': 'Go2W 定位链未能启动（需要 FAST-LIO + odom_filter → /odom_nav）',
            'loc': loc,
        }
    if not loc.get('scan') and not _scan_proc_alive():
        ensure_scan_node()
    if not _scan_proc_alive():
        return {
            'ok': False,
            'error': 'stage2 /scan 未起来（钢镚默认 Go2W；请看 /tmp/scan_node.log）',
            'loc': loc,
        }

    scan_ok = True
    params_file, rtk_fuse = _rtk_nav2_params(map_path)
    ensure_rtk_fusion(map_path, rtk_fuse)
    marker = '==== %s map=%s params=%s rtk=%s ====\n' % (
        time.strftime('%H:%M:%S'), m, params_file, 'on' if rtk_fuse else 'off')
    try:
        with open('/tmp/nav2.log', 'a') as f:
            f.write('\n' + marker)
        log_mark_pos = os.path.getsize('/tmp/nav2.log')
    except Exception:
        log_mark_pos = 0
    cmd_str = (
        'ros2 launch nav2_bringup bringup_launch.py '
        f'params_file:={params_file} '
        f'map:={map_path} use_sim_time:=False autostart:=True'
    )
    run_bg(cmd_str, '/tmp/nav2.log')
    # 等 container 真正起来；参数 YAML 坏时 bringup 秒崩，绝不能对前端谎称 ok
    # 只看本次 marker 之后的日志，避免旧 TypeError 误判
    def _nav2_log_since_mark():
        try:
            with open('/tmp/nav2.log', 'rb') as f:
                if log_mark_pos:
                    f.seek(max(0, log_mark_pos - 64))
                return f.read().decode('utf-8', 'ignore')
        except Exception:
            return ''

    up = False
    bad = ''
    for _ in range(36):
        if proc_alive('component_container_isolated'):
            up = True
            break
        tail = _nav2_log_since_mark()
        if 'Caught exception in launch' in tail or 'TypeError' in tail or 'NoneType' in tail:
            bad = tail[-500:]
            break
        time.sleep(0.5)
    if not up:
        stop_nav2_procs()
        hint = 'Nav2 bringup 启动失败（见 /tmp/nav2.log）'
        if 'NoneType' in bad:
            hint = 'Nav2 参数 YAML 非法（常见：某节点 ros__parameters 为空/只有注释）'
        elif bad.strip():
            hint = 'Nav2 launch 异常：' + bad.strip().splitlines()[-1][:160]
        return {'ok': False, 'error': hint, 'loc': loc, 'scan': scan_ok}

    stop_legacy_reloc_procs()
    # 后台催活：若 planner 仍在等 map→odom，AMCL set_initial_pose 会解；
    # 再兜底把 inactive 的 bt/velocity_smoother 拉起来。
    threading.Thread(target=_heal_nav2_lifecycle, args=(18.0,), daemon=True).start()
    return {
        'ok': True, 'map': m, 'map_pgm': (m[:-5] + '.pgm') if m.endswith('.yaml') else m,
        'scan': scan_ok, 'loc': loc,
        'auto_relocalize': False,
        'auto_reloc': False,
        'go2w': True,
        'hint': (
            '已启动（Go2W + RTK）：固定解会拉全局位姿，失锁回到 AMCL。'
            '室内或尚未固定时仍请设初始位姿。'
            if rtk_fuse else
            '已启动（Go2W）：请点「设初始位姿」在地图上拖出朝向，再确认后下目标'
        ),
        'rtk_fusion': rtk_fuse,
    }


def _heal_nav2_lifecycle(timeout_sec=18.0):
    """Activate Nav2 lifecycle nodes left inactive after a stuck planner activate.

    Uses `ros2 lifecycle` subprocesses (not in-process rclpy) so we never
    clash with web_ops_node's own rclpy context.
    """
    names = [
        'controller_server', 'smoother_server', 'planner_server',
        'bt_navigator', 'behavior_server', 'waypoint_follower', 'velocity_smoother',
    ]
    deadline = time.time() + float(timeout_sec)
    while time.time() < deadline and not proc_alive('component_container_isolated'):
        time.sleep(0.4)
    if not proc_alive('component_container_isolated'):
        return
    time.sleep(3.0)
    env = dict(os.environ)
    env['ROS_HOME'] = env.get('ROS_HOME', '/tmp/ros_home')
    prefix = (
        'source /opt/ros/humble/setup.bash && '
        'source /home/linaro/steel_coin_nav_ws/install/setup.bash && '
    )
    while time.time() < deadline:
        pending = False
        for name in names:
            try:
                st = subprocess.run(
                    ['bash', '-lc', prefix + f'ros2 lifecycle get --no-daemon /{name}'],
                    capture_output=True, text=True, timeout=4, env=env)
                out = ((st.stdout or '') + (st.stderr or '')).strip().lower()
            except Exception:
                pending = True
                continue
            # NOTE: "inactive" contains substring "active" — check inactive first.
            if 'unconfigured' in out:
                pending = True
                try:
                    subprocess.run(
                        ['bash', '-lc', prefix + f'ros2 lifecycle set --no-daemon /{name} configure'],
                        capture_output=True, text=True, timeout=6, env=env)
                    subprocess.run(
                        ['bash', '-lc', prefix + f'ros2 lifecycle set --no-daemon /{name} activate'],
                        capture_output=True, text=True, timeout=8, env=env)
                except Exception:
                    pass
            elif 'inactive' in out:
                pending = True
                try:
                    subprocess.run(
                        ['bash', '-lc', prefix + f'ros2 lifecycle set --no-daemon /{name} activate'],
                        capture_output=True, text=True, timeout=8, env=env)
                except Exception:
                    pass
            elif 'active' not in out:
                pending = True
        if not pending:
            break
        time.sleep(1.2)


def restart_rosbridge():
    for pid in proc_pids('rosbridge_websocket'):
        try:
            _kill(pid, signal.SIGKILL)
        except Exception:
            pass
    for pid in proc_pids('rosbridge_websocket_launch'):
        try:
            _kill(pid, signal.SIGKILL)
        except Exception:
            pass
    time.sleep(1.5)
    # 手机端对压缩支持差：必须 false，否则连上又断、页面一直「连接中」
    run_bg(
        'ros2 launch rosbridge_server rosbridge_websocket_launch.xml use_compression:=false',
        '/tmp/rosbridge.log')
    return {'ok': True}


def publish_cmd_vel(body):
    """网页摇杆主通路：HTTP → web_ops → /cmd_vel。

    带单调 seq：高延迟 WiFi 下晚到的旧非零包不得覆盖更新的零速，
    否则会出现「松手还走」。零速指令始终接受。
    无 seq 的旧前端：零速后 1.5s 内拒绝其非零，避免未刷新的标签页抢控制。
    """
    global _cmd_vel_seq, _cmd_vel_stop_mono
    n = _web_ops_node
    if n is None or getattr(n, 'pub_cmd', None) is None:
        return {'ok': False, 'error': 'web_ops not ready'}
    try:
        vx = float(body.get('vx', 0.0) or 0.0)
        vy = float(body.get('vy', 0.0) or 0.0)
        wz = float(body.get('wz', 0.0) or 0.0)
        try:
            seq = int(body.get('seq', 0) or 0)
        except (TypeError, ValueError):
            seq = 0
        is_stop = abs(vx) < 1e-9 and abs(vy) < 1e-9 and abs(wz) < 1e-9
        now = time.monotonic()
        with _cmd_vel_lock:
            if not is_stop:
                if seq > 0:
                    if seq < _cmd_vel_seq:
                        return {'ok': True, 'ignored': 'stale'}
                    _cmd_vel_seq = seq
                else:
                    # 旧前端无 seq：若刚下过零速，丢掉以免「松手还走」
                    if (now - _cmd_vel_stop_mono) < 1.5:
                        return {'ok': True, 'ignored': 'post_stop'}
                    if _cmd_vel_seq > 0:
                        return {'ok': True, 'ignored': 'need_seq'}
            else:
                if seq >= _cmd_vel_seq:
                    _cmd_vel_seq = seq
                _cmd_vel_stop_mono = now
            msg = Twist()
            msg.linear.x = vx
            msg.linear.y = vy
            msg.angular.z = wz
            n.pub_cmd.publish(msg)
            if is_stop:
                for _ in range(2):
                    n.pub_cmd.publish(msg)
        return {'ok': True, 'seq': _cmd_vel_seq, 'stop': is_stop}
    except Exception as e:
        return {'ok': False, 'error': str(e)}


def _genisom_bridge_pids():
    out = []
    for pid in proc_pids('genisom_bridge'):
        try:
            args = subprocess.run(
                ['ps', '-p', str(pid), '-o', 'args='],
                capture_output=True, text=True, timeout=2).stdout or ''
        except Exception:
            args = ''
        if 'web_ops' in args or 'web_ui' in args:
            continue
        if 'genisom_bridge' in args:
            out.append(pid)
    return out


def _stop_genisom_bridge():
    kill_pattern('ros2 launch genisom_bridge')
    for pid in _genisom_bridge_pids():
        try:
            _kill(pid, signal.SIGTERM)
        except Exception:
            pass
    time.sleep(0.6)
    for pid in _genisom_bridge_pids():
        try:
            _kill(pid, signal.SIGKILL)
        except Exception:
            pass
    time.sleep(0.3)
    return not bool(_genisom_bridge_pids())


def ensure_dog_single_mc_ctrl(restart=False):
    """狗端多个 mc_ctrl 会让 SDK 握手超时。尽量 SSH 清理多余实例。

    restart=True：强制杀掉并重拉 start_motion_control（主控重启后僵死运控常见）。
    """
    script = '/home/linaro/steel_coin_nav_ws/scripts/ensure_dog_mc_ctrl.sh'
    if not os.path.isfile(script):
        return {'ok': False, 'error': 'ensure_dog_mc_ctrl.sh missing'}
    try:
        cmd = ['bash', script]
        if restart:
            cmd.append('--restart')
        r = subprocess.run(
            cmd, capture_output=True, text=True, timeout=45 if restart else 25)
        out = ((r.stdout or '') + (r.stderr or ''))[-1500:]
        return {'ok': r.returncode == 0, 'output': out, 'rc': r.returncode,
                'restart': bool(restart)}
    except Exception as e:
        return {'ok': False, 'error': str(e)}


_DOG_SSH = ['sshpass', '-p', 'firefly', 'ssh', '-o', 'StrictHostKeyChecking=no',
            '-o', 'ConnectTimeout=4', 'firefly@192.168.168.168']
# bridge 必须等狗端 mc_ctrl 跑满这么久再连：运控没起完就握 SDK，会把它连「僵死」
# （进程在、但再也不接受 SDK 连接，只能重启运控）。2026-10-02 换电池（主控与狗同时开机）实测。
MC_CTRL_READY_SEC = 40.0


def _dog_mc_ctrl_info():
    """狗端 mc_ctrl 的 (pid, 已运行秒数)，取最老的一个；SSH 不通 / 没有运控 → None。"""
    try:
        r = subprocess.run(_DOG_SSH + ["ps -eo pid,etimes,args | grep '[m]c_ctrl r'"],
                           capture_output=True, text=True, timeout=10)
        rows = []
        for line in (r.stdout or '').splitlines():
            parts = line.split(None, 2)
            if len(parts) >= 2 and parts[0].isdigit() and parts[1].isdigit():
                rows.append((int(parts[0]), int(parts[1])))
        return max(rows, key=lambda x: x[1]) if rows else None
    except Exception:
        return None


def _wait_dog_mc_ctrl_ready(timeout=90.0):
    """等到狗端 mc_ctrl 已运行 ≥ MC_CTRL_READY_SEC。SSH 不通就不等（不能把 bridge 卡死在这）。"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        info = _dog_mc_ctrl_info()
        if info is None:
            return False
        if info[1] >= MC_CTRL_READY_SEC:
            return True
        time.sleep(min(5.0, MC_CTRL_READY_SEC - info[1] + 1.0))
    return False


def set_ctrl_mode(mode, force_restart=False, restart_mc_ctrl=False):
    """APP = 释放 SDK 给遥控器；SDK = 拉起/重启 genisom_bridge。

    force_restart=True：先杀再启（heal 路径必须用这个；旧逻辑在 bridge
    进程还在时直接 return，日志写「restarting」实际什么都没做）。
    restart_mc_ctrl=True：顺带强制重启狗端 mc_ctrl（断连自愈升级路径）。
    """
    mode = (mode or '').strip().upper()
    if mode not in ('APP', 'SDK'):
        return {'ok': False, 'error': 'mode must be APP or SDK'}

    if mode == 'APP':
        stopped = _stop_genisom_bridge()
        return {'ok': stopped, 'ctrl': 'APP' if stopped else 'SDK',
                'msg': '已切换到 APP（遥控器可用）' if stopped else '未能停止 SDK bridge'}

    # SDK 模式
    if force_restart:
        _stop_genisom_bridge()
    if force_restart or not _genisom_bridge_pids():
        # 先尽量保证狗端运控正常，再握 SDK
        try:
            ensure_dog_single_mc_ctrl(restart=bool(restart_mc_ctrl))
        except Exception:
            pass
        # 运控刚（重）启时等它跑满 40 s 再拉 bridge；平时它早已跑了很久，这里立即返回
        _wait_dog_mc_ctrl_ready()
        run_bg('ros2 launch genisom_bridge bridge.launch.py', '/tmp/bridge_run.log')
        time.sleep(1.2)
    alive = bool(_genisom_bridge_pids())
    return {'ok': alive, 'ctrl': 'SDK' if alive else 'APP',
            'msg': ('已重启 SDK' if force_restart else '已切换到 SDK') if alive
            else '启动 genisom_bridge 失败'}

def tail_log(file, n=50):
    safe = os.path.basename(file)
    p = '/tmp/' + safe
    if not os.path.isfile(p):
        p = '/home/linaro/steel_coin_nav_ws/' + safe
    if not os.path.isfile(p):
        return 'log not found: ' + safe
    try:
        r = subprocess.run(['tail', '-n', str(n), p], capture_output=True, text=True, timeout=5)
        return r.stdout[-8000:]
    except Exception as e:
        return 'err: ' + str(e)


def service_status():
    return {
        'livox': proc_alive('livox_ros_driver2'),
        'bridge': proc_alive('genisom_bridge'),
        # UI「LIO」芯片：导航默认看 FAST-LIO；建图时 super_lio 也算亮
        'fastlio': (
            proc_alive('fastlio_mapping') or proc_alive('super_lio_node')
            or proc_alive('relocation_node')
        ),
        'lio_tf': (
            proc_alive('fastlio_nav2_odom_filter') or proc_alive('lio_tf_bridge')
        ),
        'map_building': proc_alive('map_building_node'),
        'scan': _scan_proc_alive(),
        'nav2': proc_alive('component_container_isolated'),
        'auto_relocalize': False,  # 已移除 2D 盲搜
        'rosbridge': proc_alive('rosbridge_websocket'),
        'go2w': _go2w_odom_alive(),
        'rtk': proc_alive('oem700_driver'),
        # 本进程自己就是 web_ops；proc_alive 会跳过 me，这里直接 True
        'webops': True,
    }


def battery_json():
    """电量兜底：后端缓存的 /battery_state，走 HTTP 给前端（不依赖 rosbridge 订阅）。"""
    n = _web_ops_node
    b = getattr(n, '_battery', None) if n is not None else None
    age = None
    if n is not None:
        ts = getattr(n, '_battery_ts', 0.0) or 0.0
        if ts:
            age = round(time.monotonic() - ts, 1)
    if b is None:
        return {'ok': False, 'percentage': None, 'age_sec': age, 'sdk': False}
    try:
        return {
            'ok': True,
            'percentage': float(b.percentage),
            'voltage': float(b.voltage),
            'age_sec': age,
            'sdk': age is not None and age < 5.0,
        }
    except Exception:
        return {'ok': False, 'percentage': None, 'age_sec': age, 'sdk': False}


_sdk_heal_ts = 0.0
_sdk_heal_count = 0
_sdk_heal_window_start = 0.0


_SELF_HEAL_STAMP = '/tmp/web_ops_selfheal_ts'


def _bridge_reads_dog(timeout=6.0):
    # --no-daemon：主控重启后 ros2 daemon 常是坏的（!rclpy.ok()），走 daemon 的 echo 永远读不到 →
    # 一直误判「bridge 读不到狗」。所有 ros2 CLI 调用都加了 --no-daemon（2026-10-02）
    """新进程里 echo 一次 /robot_ctrl_mode：有数据 = bridge 正在读狗（狗 SDK 是通的）。"""
    try:
        r = subprocess.run(
            ['bash', '-c',
             'source /opt/ros/humble/setup.bash >/dev/null 2>&1; '
             'timeout %d ros2 topic echo --once --no-daemon /robot_ctrl_mode std_msgs/msg/Int32' % int(timeout)],
            capture_output=True, text=True, timeout=timeout + 4)
        return 'data:' in (r.stdout or '')
    except Exception:
        return False


def _self_heal_deaf(node, age):
    """狗 SDK 通、但本进程收不到 bridge 话题：重启 rosbridge，再退出本进程让看门狗重拉。
    不碰 bridge / 狗端 mc_ctrl（狗不会趴下）。10 分钟内最多一次（跨重启用文件记时间）。"""
    try:
        last = float(open(_SELF_HEAL_STAMP).read().strip())
    except Exception:
        last = 0.0
    if time.time() - last < 600.0:
        return
    try:
        with open(_SELF_HEAL_STAMP, 'w') as f:
            f.write('%.0f' % time.time())
    except Exception:
        pass
    try:
        node.get_logger().warn(
            'SDK/battery stale (%.1fs) but bridge IS reading dog — web_ops/rosbridge deaf: '
            'restart rosbridge + self-restart web_ops (bridge / mc_ctrl untouched)' % age)
    except Exception:
        pass
    try:
        restart_rosbridge()
    except Exception:
        pass
    # 给 HTTP 响应/日志一点时间，再整进程退出；dog-web-console 看门狗会重新拉起
    threading.Timer(2.0, lambda: os._exit(0)).start()


_BOOT_HEAL_STAMP = '/tmp/web_ops_boot_heal_mc_pid'
_boot_probe_ts = 0.0


def _boot_fast_heal(n, age):
    """开机 / 换电池快速通道：狗端运控刚开机（运行 60 s ~ 15 min，此时狗必然趴着）
    且 bridge 读不到狗 → 直接重启运控（等它跑满 40 s）再拉 bridge。
    每个运控进程只救一次（记 pid，跨 web_ops 重启），救不回来就交给常规流程 / 人工。
    超过 15 min 的运控不走这里：狗可能正被遥控器/APP 控着站立，不能让它突然趴下。"""
    global _boot_probe_ts
    if time.monotonic() - _boot_probe_ts < 15.0:   # 每秒都会调到，SSH 探测限频
        return False
    _boot_probe_ts = time.monotonic()
    info = _dog_mc_ctrl_info()
    if not info:
        return False
    pid, up_s = info
    if not (MC_CTRL_READY_SEC + 20.0 <= up_s <= 900.0):
        return False
    try:
        done = int(open(_BOOT_HEAL_STAMP).read().strip())
    except Exception:
        done = -1
    if pid == done:
        return False
    if _bridge_reads_dog():
        return False
    try:
        n.get_logger().warn(
            'SDK down %.0fs right after dog boot (mc_ctrl pid %d up %ds): restart dog mc_ctrl, '
            'wait %.0fs, restart bridge (dog is lying down after boot)' % (age, pid, up_s, MC_CTRL_READY_SEC))
    except Exception:
        pass
    try:
        set_ctrl_mode('SDK', force_restart=True, restart_mc_ctrl=True)
    except Exception:
        pass
    new = _dog_mc_ctrl_info()
    try:
        with open(_BOOT_HEAL_STAMP, 'w') as f:
            f.write(str(new[0] if new else pid))
    except Exception:
        pass
    return True


def heal_sdk_bridge():
    """狗 SDK 掉线自愈：无新鲜 /battery_state → 重启 bridge；顽固则重启狗端 mc_ctrl。

    三层：
      1) genisom_bridge 内部每 ~3s reconnect（运控正常时够用）
      2) web_ops：电量陈旧 → force_restart bridge + 去重 mc_ctrl
      3) 升级：同一小时内第 2/3 次自愈 → ensure_dog_mc_ctrl --restart
         （主控重启后单个僵死 mc_ctrl 仅去重救不回来）

    限频：启动宽限 40s；电量陈旧 ≥90s 才动手；间隔 ≥180s；1 小时最多 3 次。
    """
    global _sdk_heal_ts, _sdk_heal_count, _sdk_heal_window_start
    n = _web_ops_node
    if n is None:
        return
    # APP 模式故意无 bridge
    if not proc_alive('genisom_bridge'):
        return
    ts = getattr(n, '_battery_ts', 0.0) or 0.0
    up = time.monotonic() - getattr(n, '_boot_mono', time.monotonic())
    if up < 40.0:
        return
    age = (time.monotonic() - ts) if ts else up
    # 开机/换电池：狗刚开机时直接走快速通道（不用等常规 90 s + 180 s 间隔 + 第 2 次才重启运控）
    if age >= 45.0 and (time.monotonic() - _sdk_heal_ts) >= 60.0 and _boot_fast_heal(n, age):
        _sdk_heal_ts = time.monotonic()
        return
    if age < 90.0:
        return
    now = time.monotonic()
    if (now - _sdk_heal_window_start) > 3600.0:
        _sdk_heal_window_start = now
        _sdk_heal_count = 0
    if _sdk_heal_count >= 3:
        return
    if (now - _sdk_heal_ts) < 180.0:
        return
    # 先确认 bridge 是否真的读不到狗（用新进程探测：新进程 DDS 发现正常）。
    # 2026-10-01：bridge 一直在读狗（/odom_dog 20Hz），只是本进程收不到 /battery_state，
    # 旧逻辑照样重启 bridge + 狗端 mc_ctrl → 狗反复趴下。读得到就只救本进程 + rosbridge。
    if _bridge_reads_dog():
        _sdk_heal_ts = now
        _self_heal_deaf(n, age)
        return
    _sdk_heal_ts = now
    _sdk_heal_count += 1
    # 第 1 次：bridge + 去重；第 2/3 次：强制重启狗端运控
    restart_mc = _sdk_heal_count >= 2
    try:
        n.get_logger().warn(
            'SDK/battery stale (%.1fs) — heal bridge%s (%d/3 in 1h)'
            % (age, ' + restart dog mc_ctrl' if restart_mc else ' + dedupe mc_ctrl',
               _sdk_heal_count))
    except Exception:
        pass
    try:
        set_ctrl_mode('SDK', force_restart=True, restart_mc_ctrl=restart_mc)
    except Exception:
        try:
            ensure_dog_single_mc_ctrl(restart=restart_mc)
        except Exception:
            pass
        _stop_genisom_bridge()
        time.sleep(1.0)
        _wait_dog_mc_ctrl_ready()
        run_bg('ros2 launch genisom_bridge bridge.launch.py', '/tmp/bridge_run.log')


def reload_job_status():
    return dict(_reload_job)


def _cpu_pct():
    """Sample /proc/stat twice for approximate CPU usage percent."""
    def read():
        with open('/proc/stat') as f:
            parts = f.readline().split()
        vals = [int(x) for x in parts[1:]]
        idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
        return idle, sum(vals)
    try:
        i1, t1 = read()
        time.sleep(0.12)
        i2, t2 = read()
        dt, di = t2 - t1, i2 - i1
        if dt <= 0:
            return 0.0
        return round(max(0.0, min(100.0, (1.0 - di / dt) * 100.0)), 1)
    except Exception:
        return None


def _fmt_uptime(sec):
    sec = int(sec)
    d, sec = divmod(sec, 86400)
    h, sec = divmod(sec, 3600)
    m, s = divmod(sec, 60)
    if d > 0:
        return f'{d}天 {h:02d}:{m:02d}:{s:02d}'
    return f'{h:02d}:{m:02d}:{s:02d}'


_gnss_lock = threading.Lock()
_gnss_state = {
    'ok': False, 'fix': '无数据', 'sats': None, 'signal': '—', 'source': '', 'ts': 0.0,
}


def gnss_snapshot():
    with _gnss_lock:
        d = dict(_gnss_state)
    ts = d.pop('ts', 0.0) or 0.0
    if ts:
        d['age_sec'] = round(time.time() - ts, 1)
        d['ok'] = d['age_sec'] < 3.0 and d.get('fix') not in (None, '', '无数据')
    else:
        d['age_sec'] = None
        d['ok'] = False
    return d


def _set_gnss(fix, sats, signal):
    with _gnss_lock:
        _gnss_state['fix'] = fix
        _gnss_state['sats'] = sats
        _gnss_state['signal'] = signal
        _gnss_state['ts'] = time.time()


def _set_pose_source(source):
    with _gnss_lock:
        _gnss_state['source'] = source or ''


def _wlan_radio():
    """wlan0 的 IP 以外：信号 dBm、链路质量、SSID。读不到就留空。"""
    dbm = quality = ssid = None
    try:
        lines = open('/proc/net/wireless', encoding='utf-8', errors='ignore').read().splitlines()
        for line in lines[2:]:
            if 'wlan' not in line:
                continue
            parts = line.replace('.', ' ').split()
            if len(parts) >= 4:
                quality = int(float(parts[2]))
                dbm = int(float(parts[3]))
            break
    except Exception:
        pass
    try:
        r = subprocess.run(
            ['iw', 'dev', 'wlan0', 'link'], capture_output=True, text=True, timeout=1.5)
        for line in (r.stdout or '').splitlines():
            s = line.strip()
            if s.startswith('SSID:'):
                ssid = s.split(':', 1)[1].strip() or None
            elif s.lower().startswith('signal:'):
                for tok in s.replace('dBm', ' ').split():
                    if tok.lstrip('-').isdigit():
                        dbm = int(tok)
                        break
    except Exception:
        pass
    return dbm, quality, ssid


_slow_cache = {}


def _cached(key, ttl, fn):
    """fn() 结果缓存 ttl 秒（异常不缓存）。sysinfo 每个浏览器 4 s 轮询一次，df/hostname/ip/iw
    原来每次都 fork 5 个子进程；这些值变化很慢。"""
    now = time.monotonic()
    hit = _slow_cache.get(key)
    if hit is not None and now - hit[0] < ttl:
        return hit[1]
    val = fn()
    _slow_cache[key] = (now, val)
    return val


def sysinfo():
    info = {}
    try:
        with open('/proc/loadavg') as f:
            info['load'] = f.read().strip()
        with open('/proc/uptime') as f:
            up_sec = float(f.read().split()[0])
            info['uptime_sec'] = int(up_sec)
            info['uptime'] = _fmt_uptime(up_sec)
        info['cpu_pct'] = _cpu_pct()
        with open('/proc/meminfo') as f:
            total = avail = 0
            for line in f:
                if line.startswith('MemTotal'):
                    total = int(line.split()[1])
                elif line.startswith('MemAvailable'):
                    avail = int(line.split()[1])
            used = max(0, total - avail)
            info['mem'] = f'{avail // 1024}MB / {total // 1024}MB 可用'
            info['mem_pct'] = round(used * 100.0 / total, 1) if total else None
            info['mem_used_mb'] = used // 1024
            info['mem_total_mb'] = total // 1024
        # Filesystem 1B-blocks Used Available Use% Mounted
        parts = _cached('df', 30.0, lambda: subprocess.run(
            ['df', '-B1', '/'], capture_output=True, text=True, timeout=5
        ).stdout.splitlines()[1].split())
        disk_total, disk_used = int(parts[1]), int(parts[2])
        info['disk'] = f'{int(parts[3]) // (1024 ** 3)}GB 可用'
        info['disk_pct'] = round(disk_used * 100.0 / disk_total, 1) if disk_total else None
        info['services'] = service_status()
        # SDK bridge alive => SDK 通道；否则视为遥控器/APP
        info['ctrl'] = 'SDK' if info['services'].get('bridge') else 'APP'
        info['net'] = _cached('hostname_I', 10.0, lambda: subprocess.run(
            ['hostname', '-I'], capture_output=True, text=True, timeout=5).stdout.strip())
        # Prefer wlan0 for UI hint
        try:
            info['wlan_ip'] = _cached('wlan_ip', 10.0, lambda: (subprocess.run(
                ['bash', '-c', "ip -4 -o addr show dev wlan0 | awk '{print $4}' | cut -d/ -f1"],
                capture_output=True, text=True, timeout=3).stdout or '').strip())
        except Exception:
            info['wlan_ip'] = ''
        dbm, quality, ssid = _cached('wlan_radio', 3.0, _wlan_radio)
        info['wlan_dbm'] = dbm
        info['wlan_quality'] = quality
        info['wlan_ssid'] = ssid or ''
        info['gnss'] = gnss_snapshot()
        info['video_mjpeg'] = '/api/video/mjpeg'
        info['video_rtsp'] = DOG_RTSP
        info['video_width'] = int((_video_cfg or {}).get('width', VIDEO_WIDTH))
        info['video_fps'] = int((_video_cfg or {}).get('fps', VIDEO_FPS))
        info['video_q'] = int((_video_cfg or {}).get('q', VIDEO_Q))
        info['video_profile'] = _video_profile
        info['video_profile_label'] = VIDEO_PROFILES.get(_video_profile, {}).get('label', '')
        info['video_fps_live'] = round(mjpeg_fps(), 1)
    except Exception as e:
        info['error'] = str(e)
    return info


# ---------------- ROS 节点 ----------------
class WebOpsNode(Node):
    def __init__(self):
        super().__init__('web_ops_node')
        self.sub_nav_cmd = self.create_subscription(Twist, '/web/nav_cmd', self.cb_nav_cmd, 10)
        self.sub_nav_cancel = self.create_subscription(Empty, '/web/nav_cancel', self.cb_nav_cancel, 10)
        self.sub_init_pose = self.create_subscription(Twist, '/web/init_pose', self.cb_init_pose, 10)
        self.sub_start_nav2 = self.create_subscription(Empty, '/web/start_nav2', self.cb_start_nav2, 10)
        self.sub_stop_nav2 = self.create_subscription(Empty, '/web/stop_nav2', self.cb_stop_nav2, 10)
        self.sub_mapping = self.create_subscription(Empty, '/web/mapping', self.cb_mapping, 10)

        # 电量：后端缓存一份，走 /api/battery 让前端 HTTP 拉，不依赖 rosbridge 订阅
        self._battery = None
        self._battery_ts = 0.0
        self._boot_mono = time.monotonic()
        # 高频订阅/定时器挂到独立小节点 web_ops_viz + 专用 executor 线程（见下方 _viz_node）

        # TRANSIENT_LOCAL：重定位节点晚订阅也能拿到当前导航状态，避免导航中误触发重搜
        _nav_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.pub_nav_status = self.create_publisher(String, '/web/nav_status', _nav_qos)
        # NavigateToPose 反馈（剩余距离/ETA/恢复次数）给网页 RViz 风格 Nav2 面板
        self.pub_nav_feedback = self.create_publisher(String, '/web/nav_feedback', 10)
        self.pub_sys_status = self.create_publisher(String, '/web/sys_status', 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        # 路网巡航时把整条折线发到 /plan，网页绿线跟画出的路线一致
        self.pub_plan = self.create_publisher(Path, '/plan', 10)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # 高频可视化链路（20 Hz 位姿定时器、10 Hz /scan、20 Hz /battery_state、局部路径/代价地图）
        # 放在独立小节点 + 专用单线程 executor 上。rclpy executor 每次唤醒都要把节点上所有实体
        # （每个 pub/sub 还带 QoS 事件 waitable）重建一遍 wait set，主节点有 ~30 个实体，
        # 这些高频回调全挂主节点时光 executor 开销就占 web_ops ~20% CPU（2026-10-02 py-spy）。
        # 话题名不变，网页无感；主节点只剩 1 Hz 状态 + 偶发的网页指令/导航 action。
        self._viz_node = rclpy.create_node('web_ops_viz')
        vn = self._viz_node
        # 网页 TFClient 常拿不到 map→base_link；这里查 TF 后转发给前端画激光
        self.pub_robot_pose = vn.create_publisher(PoseStamped, '/web/robot_pose', 10)
        # 把 /scan 按「激光时间戳」转到 map，前端直接画 map 系红点，避免箭头位姿与激光不同步导致一走就漂
        self.pub_scan_map = vn.create_publisher(Float32MultiArray, '/web/scan_map', 10)
        # 局部规划/代价地图在 odom 系；转到 map 后给网页画（类似 RViz 图层）
        self.pub_local_plan = vn.create_publisher(Path, '/web/local_plan', 10)
        self.pub_local_cost = vn.create_publisher(OccupancyGrid, '/web/local_costmap', 5)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        # TF 用独立小节点 + 专用线程接收：本节点是单线程 executor，20Hz 位姿定时器 / 激光回调里
        # 带 timeout 的 lookup 会阻塞 executor；若 TF 订阅也挂在本节点上，查不到时就收不到新 TF，
        # 越查不到越阻塞（实测导航启动后 /web/scan_map、/web/robot_pose 一直 0Hz）。
        # 注意不能直接 spin_thread=True 挂本节点：会把同一节点加进第二个 executor，回调执行两次。
        self._tf_node = rclpy.create_node('web_ops_tf_listener')
        self._tf_listener = TransformListener(self._tf_buffer, self._tf_node, spin_thread=True)
        # FAST-LIO 卡死看门狗的快速路径（见 _fastlio_output_alive）；挂在 TF 小节点的线程上，
        # raw=True 只记时间不反序列化，不占主 executor
        self._tf_node.create_subscription(
            Odometry, '/Odometry', _on_odom_raw,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=1),
            raw=True)
        vn.create_subscription(BatteryState, '/battery_state', self.cb_battery, 10)
        vn.create_subscription(
            LaserScan, '/scan', self.cb_scan_to_map, qos_profile_sensor_data)
        vn.create_subscription(Path, '/local_plan', self.cb_local_plan, 10)
        vn.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.cb_local_costmap,
            qos_profile_sensor_data)
        # /map 是 map_server 一次性 TRANSIENT_LOCAL 发布的；rosbridge 对任意话题固定
        # 用 BEST_EFFORT/VOLATILE 订阅（这里这版 roslib.min.js 也不支持传 QoS 覆盖），
        # 只要浏览器这边的 rosbridge 订阅是在那一次性发布*之后*才建立的——WiFi 掉线
        # 重连、或者页面比 Nav2 启动得晚——就永远收不到，网页上"导航加载不出地图"。
        # 用正确匹配的 QoS 在后端订阅一次缓存下来，配 /api/nav2/map 走 HTTP 一次性
        # 拉取，不依赖 rosbridge 的订阅时序。
        self._last_map = None
        self.create_subscription(
            OccupancyGrid, '/map', self.cb_map,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1))

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.through_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.follow_client = ActionClient(self, FollowPath, 'follow_path')
        self._ctrl_param_cli = self.create_client(SetParameters, '/controller_server/set_parameters')
        self._route_token = 0
        self._route_active = False
        self._route_handle = None
        self._route_pts = []
        self._route_verts = []
        self._route_goal_i = 0
        self._route_i = 0
        self._route_search_tight = False
        self._route_total = 0.0
        self._route_t0 = 0.0
        self._map_xy = None
        self.nav_goal_handle = None
        self._nav_goal_token = 0  # 递增；旧 goal 的 cancel/result 回调不得覆盖新状态
        self.nav_state = 'idle'
        self._gnss_fix_topic = ''
        self._gnss_nmea_topic = ''
        self._gnss_gga_ts = 0.0
        self.create_subscription(String, '/gnss/pose_source', self._on_pose_source, 10)
        self.create_timer(5.0, self._discover_gnss)
        # 未对齐禁止下发 Nav2 目标（AMCL set_initial_pose=(0,0) 只为起 lifecycle，不是真定位）
        self._loc_aligned = False
        # cb_status_timer 用：heal_nav_deps() 挪到后台线程跑，这把锁保证同一时刻只有一条在跑
        self._heal_lock = threading.Lock()
        self._loc_user_accepted = False  # 用户点过「确认定位」/手动设姿
        self._loc_hit = 0.0

        self.create_timer(1.0, self.cb_status_timer)
        vn.create_timer(0.05, self.cb_robot_pose_timer)
        self._set_nav_status('idle', '')
        # 所有属性就绪后再启动 viz executor 线程
        self._viz_exec = SingleThreadedExecutor()
        self._viz_exec.add_node(vn)
        threading.Thread(target=self._spin_viz, name='web_ops_viz', daemon=True).start()

    def _spin_viz(self):
        try:
            self._viz_exec.spin()
        except Exception as e:
            # 进程收到 SIGTERM 时 context 被关 → ExternalShutdownException，属正常退出
            if type(e).__name__ != 'ExternalShutdownException':
                self.get_logger().error('web_ops_viz spin stopped: %s' % e)

    def cb_map(self, msg: OccupancyGrid):
        self._last_map = msg

    @staticmethod
    def _yaw_of(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _lookup_xyyaw(self, target, source, stamp=None):
        """Return (tx,ty,tyaw) of transform target←source, or None."""
        try:
            if stamp is not None:
                tf = self._tf_buffer.lookup_transform(
                    target, source, stamp, timeout=Duration(seconds=0.05))
            else:
                tf = self._tf_buffer.lookup_transform(
                    target, source, Time(), timeout=Duration(seconds=0.05))
        except Exception:
            try:
                tf = self._tf_buffer.lookup_transform(
                    target, source, Time(), timeout=Duration(seconds=0.02))
            except Exception:
                return None
        t = tf.transform.translation
        return (t.x, t.y, self._yaw_of(tf.transform.rotation))

    @staticmethod
    def _xform_xy(tx, ty, tyaw, x, y):
        c, s = math.cos(tyaw), math.sin(tyaw)
        return tx + x * c - y * s, ty + x * s + y * c

    def cb_local_plan(self, msg: Path):
        if not msg.poses:
            return
        frame = (msg.header.frame_id or 'odom').lstrip('/')
        if frame == 'map':
            out = msg
        else:
            tf = self._lookup_xyyaw('map', frame)
            if tf is None:
                return
            tx, ty, tyaw = tf
            out = Path()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = 'map'
            for p in msg.poses:
                ps = PoseStamped()
                ps.header.frame_id = 'map'
                ps.header.stamp = p.header.stamp
                x, y = self._xform_xy(tx, ty, tyaw, p.pose.position.x, p.pose.position.y)
                ps.pose.position.x = x
                ps.pose.position.y = y
                # 航向叠加（局部路径箭头不太关键，仍近似）
                pyaw = self._yaw_of(p.pose.orientation) + tyaw
                ps.pose.orientation.z = math.sin(pyaw * 0.5)
                ps.pose.orientation.w = math.cos(pyaw * 0.5)
                out.poses.append(ps)
        self.pub_local_plan.publish(out)

    def cb_local_costmap(self, msg: OccupancyGrid):
        frame = (msg.header.frame_id or 'odom').lstrip('/')
        out = OccupancyGrid()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'map'
        out.info.resolution = msg.info.resolution
        out.info.width = msg.info.width
        out.info.height = msg.info.height
        out.data = msg.data
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        oyaw = self._yaw_of(msg.info.origin.orientation)
        if frame != 'map':
            tf = self._lookup_xyyaw('map', frame)
            if tf is None:
                return
            tx, ty, tyaw = tf
            ox, oy = self._xform_xy(tx, ty, tyaw, ox, oy)
            oyaw = oyaw + tyaw
        out.info.origin = Pose()
        out.info.origin.position.x = ox
        out.info.origin.position.y = oy
        out.info.origin.orientation.z = math.sin(oyaw * 0.5)
        out.info.origin.orientation.w = math.cos(oyaw * 0.5)
        self.pub_local_cost.publish(out)

    def cb_scan_to_map(self, scan: LaserScan):
        """Project LaserScan into map frame at the scan stamp (not latest pose)."""
        if not proc_alive('component_container_isolated'):
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', scan.header.frame_id,
                Time.from_msg(scan.header.stamp),
                timeout=Duration(seconds=0.05))
        except Exception:
            try:
                tf = self._tf_buffer.lookup_transform(
                    'map', scan.header.frame_id, Time(),
                    timeout=Duration(seconds=0.02))
            except Exception:
                return
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        bx = tf.transform.translation.x
        by = tf.transform.translation.y
        rmin = max(float(scan.range_min or 0.05), 0.40)
        rmax = scan.range_max or 30.0
        # numpy 向量化（原逐束 Python 循环 + list 赋值，10 Hz 下单这一项就占数个百分点 CPU）；
        # 规则不变：先收集有效束，再丢掉孤立角向杂点（前端红点更干净）
        try:
            rr = np.frombuffer(scan.ranges, dtype=np.float32).astype(np.float64)
        except (TypeError, ValueError):
            rr = np.asarray(scan.ranges, dtype=np.float64)
        aa = scan.angle_min + np.arange(rr.size, dtype=np.float64) * scan.angle_increment
        with np.errstate(invalid='ignore'):
            valid = np.isfinite(rr) & (rr > rmin) & (rr < rmax)
        br = rr[valid]
        ba = aa[valid]
        n = br.size
        if n < 4:
            return
        # 有效束序列里前后各 2 个邻居：|Δr| ≤ max(0.35, 0.25 r_i) 且 |Δ角| < 0.05 → 至少 1 个才保留
        tol = np.maximum(0.35, br * 0.25)
        keep = np.zeros(n, dtype=bool)
        for k in (1, 2):
            dr = np.abs(br[k:] - br[:-k])
            close = np.abs(ba[k:] - ba[:-k]) < 0.05
            keep[:-k] |= close & (dr <= tol[:-k])
            keep[k:] |= close & (dr <= tol[k:])
        if np.count_nonzero(keep) >= 4:
            br = br[keep]
            ba = ba[keep]  # 过稀时回退用全部有效束，避免整帧空白
        lx = br * np.cos(ba)
        ly = br * np.sin(ba)
        xy = np.empty(br.size * 2, dtype=np.float32)
        xy[0::2] = bx + lx * c - ly * s
        xy[1::2] = by + lx * s + ly * c
        msg = Float32MultiArray()
        msg.data = array.array('f', xy.tobytes())
        self.pub_scan_map.publish(msg)

    def cb_robot_pose_timer(self):
        """Republish map→base_footprint for the web console (scan overlay + dog arrow).

        /scan is published in base_footprint (nav_scan_node); fall back to base_link.
        """
        if not proc_alive('component_container_isolated'):
            return
        t = None
        for frame in ('base_footprint', 'base_link'):
            try:
                t = self._tf_buffer.lookup_transform(
                    'map', frame, Time(),
                    timeout=Duration(seconds=0.05))
                break
            except Exception:
                continue
        if t is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = t.transform.translation.x
        msg.pose.position.y = t.transform.translation.y
        msg.pose.position.z = 0.0
        # 只用平面航向，避免 roll/pitch 渗进网页 yaw（箭头左右偏）
        q = t.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # 与 lio_tf_bridge 一致：用机体系 +X 投影，俯仰时更稳
        # R*[1,0,0]
        fx = 1 - 2 * (q.y * q.y + q.z * q.z)
        fy = 2 * (q.x * q.y + q.z * q.w)
        if fx * fx + fy * fy > 1e-8:
            yaw = math.atan2(fy, fx)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        self._map_xy = (float(msg.pose.position.x), float(msg.pose.position.y))
        self.pub_robot_pose.publish(msg)

    def cb_nav_cmd(self, msg):
        x, y, yaw = msg.linear.x, msg.linear.y, msg.angular.z
        self._route_stop()
        self.get_logger().info(f'nav_goal ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f}deg)')
        if abs(x) < 0.02 and abs(y) < 0.02:
            return
        # 网页/rosbridge 偶发连发同一目标 2～3 次 → cancel/preempt 风暴，加重假到达
        now = time.time()
        last = getattr(self, '_last_nav_cmd', None)
        if last and (now - last[0]) < 0.6 and abs(last[1] - x) < 0.05 and abs(last[2] - y) < 0.05:
            self.get_logger().info('nav_goal debounced (duplicate)')
            return
        self._last_nav_cmd = (now, x, y, yaw)
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self._set_nav_status('aborted', 'nav2_server_unavailable')
            return
        # 先取消旧目标；用 token 忽略旧 result，避免把新导航刷成 canceled
        self._nav_goal_token += 1
        send_token = self._nav_goal_token
        old = self.nav_goal_handle
        self.nav_goal_handle = None
        if old is not None:
            try:
                old.cancel_goal_async()
            except Exception:
                pass
            time.sleep(0.25)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
        self._nav_goal_xyyaw = (float(x), float(y), float(yaw))
        self._nav_fb_ts = 0.0
        f = self.nav_client.send_goal_async(
            goal, feedback_callback=lambda fb, tok=send_token: self._goal_feedback_cb(fb, tok))
        f.add_done_callback(lambda fut, tok=send_token: self._goal_sent_cb(fut, tok))

    def _goal_feedback_cb(self, fb_msg, token):
        """NavigateToPose 反馈 → /web/nav_feedback（JSON，≤2Hz），网页 Nav2 面板显示剩余距离/ETA。"""
        if token != self._nav_goal_token:
            return
        now = time.time()
        if now - getattr(self, '_nav_fb_ts', 0.0) < 0.5:
            return
        self._nav_fb_ts = now
        fb = fb_msg.feedback

        def _sec(d):
            return float(d.sec) + float(d.nanosec) * 1e-9
        gx, gy, gyaw = getattr(self, '_nav_goal_xyyaw', (None, None, None))
        self.pub_nav_feedback.publish(String(data=json.dumps({
            'distance_remaining': round(float(fb.distance_remaining), 3),
            'eta_sec': round(_sec(fb.estimated_time_remaining), 1),
            'nav_time_sec': round(_sec(fb.navigation_time), 1),
            'recoveries': int(fb.number_of_recoveries),
            'goal': [gx, gy, gyaw],
        })))

    def _goal_sent_cb(self, future, token):
        if token != self._nav_goal_token:
            return
        try:
            h = future.result()
        except Exception as e:
            self._set_nav_status('aborted', f'goal_send_{e}')
            return
        if h is None or not h.accepted:
            # 多半是 Nav2 半启动（bt_navigator inactive）：后台补激活，提示重新下发
            def _fix():
                fixed = ensure_nav_lifecycle_active()
                self._set_nav_status(
                    'aborted',
                    '导航服务未就绪，已自动修复，请重新下发目标' if fixed
                    else '目标被 Nav2 拒绝（检查目标点是否在地图可通行区域）')
            threading.Thread(target=_fix, daemon=True).start()
            self._set_nav_status('aborted', '目标被拒绝，正在检查 Nav2 状态…')
            return
        self.nav_goal_handle = h
        self._set_nav_status('navigating', '')
        h.get_result_async().add_done_callback(
            lambda fut, tok=token, handle=h: self._goal_result_cb(fut, tok, handle))

    def _goal_result_cb(self, future, token, handle):
        # 已被更新的目标 / 替换发送：丢弃过期回调
        if token != self._nav_goal_token or self.nav_goal_handle is not handle:
            return
        try:
            st = future.result().status
        except Exception as e:
            self._set_nav_status('aborted', f'result_{e}')
            self.nav_goal_handle = None
            return
        # GoalStatus: SUCCEEDED=4, CANCELED=5, ABORTED=6 (action_msgs)
        if st == 4:
            self._set_nav_status('reached', '')
        elif st == 5:
            self._set_nav_status('canceled', '')
        else:
            self._set_nav_status('aborted', f'status_{st}')
        self.nav_goal_handle = None

    def send_through_poses(self, poses):
        """按顺序走多个位姿（NavigateThroughPoses）。"""
        self._route_stop()
        self.get_logger().info('nav_through %d poses' % len(poses))
        if not self.through_client.wait_for_server(timeout_sec=2.0):
            self._set_nav_status('aborted', 'through_poses_unavailable')
            return
        self._nav_goal_token += 1
        send_token = self._nav_goal_token
        old = self.nav_goal_handle
        self.nav_goal_handle = None
        if old is not None:
            try:
                old.cancel_goal_async()
            except Exception:
                pass
            time.sleep(0.25)
        goal = NavigateThroughPoses.Goal()
        # stamp=0：规划器按位姿时间戳查 map←base_footprint。
        # 写成 now 会钉在下发那一刻；导航刚重启或过几秒重规划时，这个时间已经不在 TF 缓存里，
        # 查找一直失败，/plan 没有路径，狗也不会动。
        for x, y, yaw in poses:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp.sec = 0
            ps.header.stamp.nanosec = 0
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation = Quaternion(
                x=0.0, y=0.0, z=math.sin(float(yaw) / 2.0), w=math.cos(float(yaw) / 2.0))
            goal.poses.append(ps)
        self._nav_goal_xyyaw = (float(poses[0][0]), float(poses[0][1]), float(poses[0][2]))
        self._nav_fb_ts = 0.0
        fut = self.through_client.send_goal_async(
            goal, feedback_callback=lambda fb, tok=send_token: self._goal_feedback_cb(fb, tok))
        fut.add_done_callback(lambda done, tok=send_token: self._goal_sent_cb(done, tok))

    def _on_pose_source(self, msg):
        _set_pose_source((getattr(msg, 'data', '') or '').strip())

    def _discover_gnss(self):
        if self._gnss_fix_topic and self._gnss_nmea_topic:
            return
        try:
            topics = self.get_topic_names_and_types()
        except Exception:
            return
        for name, types in topics:
            if (not self._gnss_fix_topic) and ('sensor_msgs/msg/NavSatFix' in types):
                self.create_subscription(
                    NavSatFix, name, self._on_navsat, qos_profile_sensor_data)
                self._gnss_fix_topic = name
                self.get_logger().info('GNSS NavSatFix: %s' % name)
            if (not self._gnss_nmea_topic) and ('nmea_msgs/msg/Sentence' in types):
                if self._subscribe_nmea(name):
                    self._gnss_nmea_topic = name

    def _subscribe_nmea(self, topic):
        try:
            from nmea_msgs.msg import Sentence
        except Exception:
            return False
        self.create_subscription(Sentence, topic, self._on_nmea, 10)
        self.get_logger().info('GNSS NMEA: %s' % topic)
        return True

    def _on_navsat(self, msg):
        if time.time() - self._gnss_gga_ts < 2.0:
            return
        st = int(msg.status.status)
        fix = {-1: '无解', 0: '单点', 1: '差分', 2: 'RTK'}.get(st, '无解')
        signal = '无'
        if st >= 0:
            try:
                cov = float(msg.position_covariance[0])
            except Exception:
                cov = -1.0
            if cov < 0:
                signal = '有'
            elif cov < 0.05:
                signal = '强'
            elif cov < 1.0:
                signal = '中'
            else:
                signal = '弱'
        _set_gnss(fix, _gnss_state.get('sats'), signal)

    def _on_nmea(self, msg):
        sentence = getattr(msg, 'sentence', '') or ''
        if 'GGA' not in sentence:
            return
        parts = sentence.strip().split(',')
        if len(parts) < 8:
            return
        fix = {'0': '无解', '1': '单点', '2': '差分', '4': 'RTK固定', '5': 'RTK浮点', '6': '惯导'}.get(
            parts[6].strip(), '无解')
        try:
            sats = int(float(parts[7]))
        except Exception:
            sats = None
        hdop = None
        if len(parts) > 8 and parts[8]:
            try:
                hdop = float(parts[8])
            except Exception:
                hdop = None
        if fix == '无解':
            signal = '无'
        elif hdop is None:
            signal = '有'
        elif hdop <= 1.0:
            signal = '强'
        elif hdop <= 2.0:
            signal = '中'
        else:
            signal = '弱'
        self._gnss_gga_ts = time.time()
        _set_gnss(fix, sats, signal)

    def start_route_cruise(self, vertices):
        """按画出的折线逐段走，不改控制器参数。遇障则停在当前这段上等待，不改线。"""
        clean = []
        for p in vertices:
            xy = (float(p['x']), float(p['y']))
            if not clean or math.hypot(xy[0] - clean[-1][0], xy[1] - clean[-1][1]) >= 0.05:
                clean.append(xy)
        if len(clean) < 2:
            self._set_nav_status('aborted', 'route_too_short')
            return
        self._route_verts = clean
        self._route_i = 0
        self._route_pts = _sample_polyline(
            [{'x': x, 'y': y} for x, y in clean], 0.2)
        self._route_total = 0.0
        for i in range(1, len(clean)):
            self._route_total += math.hypot(
                clean[i][0] - clean[i - 1][0], clean[i][1] - clean[i - 1][1])
        self._route_t0 = time.time()
        self._route_active = True
        self._route_token += 1
        token = self._route_token
        self._nav_goal_token += 1
        old = self.nav_goal_handle
        self.nav_goal_handle = None
        if old is not None:
            try:
                old.cancel_goal_async()
            except Exception:
                pass
        self._route_goal_i = 0
        self.get_logger().info(
            'cruise_route %d vertices, one segment at a time, length %.2f m' % (
                len(clean), self._route_total))
        self._publish_route_plan()
        self._send_route_follow(token)

    def _route_stop(self):
        """停下路网跟随。不能去改控制器参数，普通导航和路网共用同一个 FollowPath。"""
        self._route_active = False
        self._route_token += 1
        handle = getattr(self, '_route_handle', None)
        self._route_handle = None
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass

    def _route_release_search(self):
        return

    def _robot_xy(self):
        if self._map_xy:
            return self._map_xy
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_footprint', Time(), timeout=Duration(seconds=0.05))
            return (float(t.transform.translation.x), float(t.transform.translation.y))
        except Exception:
            return None

    def _publish_route_plan(self):
        pts = self._route_pts or []
        if len(pts) < 2:
            return
        try:
            self.pub_plan.publish(self._route_path_msg(pts))
        except Exception:
            pass

    def _route_advance(self):
        """只沿路线往前记进度。几何上更近的后面端点不算已经走过。"""
        pts = self._route_pts or []
        xy = self._robot_xy()
        if not xy or len(pts) < 2:
            return
        i = self._route_i
        while i < len(pts) - 1 and math.hypot(pts[i][0] - xy[0], pts[i][1] - xy[1]) <= 0.45:
            i += 1
        self._route_i = i

    def _route_follow_pts(self):
        """只跟当前这一段，目标是下一个端点，不是整条线的终点。"""
        verts = self._route_verts or []
        gi = self._route_goal_i
        if gi >= len(verts):
            return []
        end = verts[gi]
        if gi == 0:
            xy = self._robot_xy()
            if xy is None:
                return []
            if math.hypot(xy[0] - end[0], xy[1] - end[1]) <= 0.30:
                self._route_goal_i = 1
                return self._route_follow_pts()
            start = xy
        else:
            start = verts[gi - 1]
        yaw = math.atan2(end[1] - start[1], end[0] - start[0])
        seg = math.hypot(end[0] - start[0], end[1] - start[1])
        if seg < 0.05:
            return [(end[0], end[1], yaw), (end[0], end[1], yaw)]
        n = max(1, int(math.ceil(seg / 0.2)))
        out = []
        for k in range(n):
            t = k / float(n)
            out.append((
                start[0] + (end[0] - start[0]) * t,
                start[1] + (end[1] - start[1]) * t,
                yaw))
        out.append((end[0], end[1], yaw))
        return out

    def _route_path_msg(self, pts):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp.sec = 0
        path.header.stamp.nanosec = 0
        for x, y, yaw in pts:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp.sec = 0
            ps.header.stamp.nanosec = 0
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation = Quaternion(
                x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
            path.poses.append(ps)
        return path

    def _send_route_follow(self, token):
        if token != self._route_token or not self._route_active:
            return
        if not self.follow_client.wait_for_server(timeout_sec=2.0):
            self._route_active = False
            self._route_release_search()
            self._set_nav_status('aborted', 'follow_path_unavailable')
            return
        pts = self._route_follow_pts()
        if len(pts) < 2:
            self._route_active = False
            self._route_release_search()
            self._set_nav_status('reached', '')
            return
        end = pts[-1]
        self.get_logger().info(
            'cruise_route follow %d poses, from idx %d -> (%.2f, %.2f)' % (
                len(pts), self._route_i, end[0], end[1]))
        self._publish_route_plan()
        goal = FollowPath.Goal()
        goal.path = self._route_path_msg(pts)
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'goal_checker'
        fut = self.follow_client.send_goal_async(
            goal, feedback_callback=lambda fb, tok=token: self._route_feedback(fb, tok))
        fut.add_done_callback(lambda done, tok=token: self._route_sent(done, tok))

    def _route_feedback(self, fb_msg, token):
        if token != self._route_token or not self._route_active:
            return
        now = time.time()
        if now - getattr(self, '_route_fb_ts', 0.0) < 0.5:
            return
        self._route_fb_ts = now
        dist = float(fb_msg.feedback.distance_to_goal)
        elapsed = max(0.0, now - self._route_t0)
        speed = 0.22
        self.pub_nav_feedback.publish(String(data=json.dumps({
            'distance_remaining': round(dist, 3),
            'eta_sec': round(dist / speed, 1),
            'nav_time_sec': round(elapsed, 1),
            'traveled': round(max(0.0, self._route_total - dist), 3),
            'length': round(self._route_total, 3),
            'goal': None,
        })))

    def _route_sent(self, future, token):
        if token != self._route_token or not self._route_active:
            return
        try:
            handle = future.result()
        except Exception as e:
            self._route_active = False
            self._route_release_search()
            self._set_nav_status('aborted', 'route_send_%s' % e)
            return
        if handle is None or not handle.accepted:
            self._route_active = False
            self._route_release_search()
            self._set_nav_status('aborted', '路线被控制器拒绝')
            return
        self._route_handle = handle
        self._set_nav_status('navigating', '')
        handle.get_result_async().add_done_callback(
            lambda fut, tok=token, h=handle: self._route_result(fut, tok, h))

    def _route_result(self, future, token, handle):
        if token != self._route_token or self._route_handle is not handle:
            return
        self._route_handle = None
        try:
            status = int(future.result().status)
        except Exception as e:
            self._route_active = False
            self._route_release_search()
            self._set_nav_status('aborted', 'route_result_%s' % e)
            return
        if not self._route_active or token != self._route_token:
            return
        if status == 4:
            self._route_goal_i += 1
            if self._route_goal_i >= len(self._route_verts or []):
                self._route_active = False
                self._set_nav_status('reached', '')
                return
            self._send_route_follow(token)
            return
        if status == 5:
            self._route_active = False
            self._route_release_search()
            self._set_nav_status('canceled', '')
            return
        # 碰撞或其他中止：停在线上等一秒，从当前位置接着原路线走，不改线
        def _retry():
            self._publish_zero()
            time.sleep(0.8)
            if token == self._route_token and self._route_active:
                self._send_route_follow(token)
        threading.Thread(target=_retry, daemon=True).start()

    def cb_nav_cancel(self, msg):
        self._route_active = False
        self._route_token += 1
        self._route_release_search()
        route_handle = getattr(self, '_route_handle', None)
        self._route_handle = None
        if route_handle is not None:
            try:
                route_handle.cancel_goal_async()
            except Exception:
                pass
        self._nav_goal_token += 1
        if self.nav_goal_handle is not None:
            try:
                self.nav_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.nav_goal_handle = None
            self._set_nav_status('canceled', '')
        else:
            self._set_nav_status('canceled', 'no_active_goal')
        self._publish_zero()

    def _publish_zero(self):
        try:
            for _ in range(5):
                self.pub_cmd.publish(Twist())
                time.sleep(0.05)
        except Exception:
            pass

    def cb_battery(self, msg):
        self._battery = msg
        self._battery_ts = time.monotonic()

    def _odom_tf_stamp(self):
        try:
            t = self._tf_buffer.lookup_transform(
                'odom', 'base_footprint', Time(), timeout=Duration(seconds=0.15))
            return t.header.stamp
        except Exception:
            return (self.get_clock().now() - Duration(seconds=0.4)).to_msg()

    def cb_init_pose(self, msg):
        x, y, yaw = msg.linear.x, msg.linear.y, msg.angular.z
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp.sec = 0
        pose.header.stamp.nanosec = 0
        self.get_logger().info(
            'init_pose (%.2f, %.2f, yaw %.0fdeg)' % (x, y, math.degrees(yaw)))
        pose.pose.pose.position.x = float(x)
        pose.pose.pose.position.y = float(y)
        pose.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))
        pose.pose.covariance[0] = 0.25
        pose.pose.covariance[7] = 0.25
        pose.pose.covariance[35] = 0.25
        self.pub_init.publish(pose)
        # 用户拖了初始位姿 = 认可当前对齐，允许下目标
        self._loc_user_accepted = True
        self._loc_aligned = True
        self._loc_hit = max(self._loc_hit, 0.45)

    def cb_mapping(self, msg):
        set_mapping_session(True)
        if not proc_alive('super_lio_node'):
            run_bg(f'bash {START_ALL} mapping', '/tmp/mapping.log')
            self.get_logger().info('mapping chain restarted via start_all.sh mapping')
        else:
            # Ensure map_building preview is up
            if not proc_alive('map_building_node'):
                run_bg(
                    'ros2 launch nav2_tools map_building.launch.py',
                    '/tmp/map_building.log')

    def cb_start_nav2(self, msg):
        self.get_logger().info('start nav2 (ROS trigger)')
        # Prefer live map; start_nav2 resolves missing names itself
        start_nav2({'map': 'map_live.yaml'})
        # 启导航时清掉上次卡死的 navigating（无 goal handle）
        self.nav_goal_handle = None
        self._nav_goal_token += 1
        self._set_nav_status('idle', 'nav2_started')

    def cb_stop_nav2(self, msg):
        stop_nav2_api()
        self._publish_zero()
        self.nav_goal_handle = None
        self._nav_goal_token += 1
        self._set_nav_status('idle', 'nav2_stopped')

    def cb_status_timer(self):
        # heal_nav_deps() 内部（ensure_localization_stack/ensure_scan_node/
        # ensure_auto_relocalize）本来就是设计成阻塞轮询、最长能堵 10+ 秒——
        # 这些函数也被 /api/relocalize、/api/nav2/start 等 HTTP 接口直接同步
        # 调用，那些地方就是要等到准确结果，不能改成非阻塞。
        # 但这里是 1Hz ROS 定时器回调，rclpy 默认单线程 executor：直接同步调用
        # heal_nav_deps() 会把这条回调，连带 /web/sys_status 的周期发布和
        # cb_nav_cmd（导航目标）等其它订阅回调，一起卡住最长 10+ 秒。
        # 2026-09-28 实测：super_lio_node 内存涨到 4GB+ 被 OOM 杀掉后，
        # heal_nav_deps 反复触发这段阻塞重启轮询，其间 /web/nav_cmd 收不到——
        # 这就是「导航页设了目标机器人不走」的根因之一。
        # 挪到后台线程跑，_heal_lock 保证同一时刻只有一条在跑，不重叠。
        if self._heal_lock.acquire(blocking=False):
            def _heal():
                try:
                    heal_nav_deps()
                except Exception:
                    pass
                try:
                    heal_sdk_bridge()
                except Exception:
                    pass
                try:
                    busy_prof = video_busy_profile()
                    if busy_prof and _video_profile != busy_prof:
                        set_video_profile(busy_prof, start_feeder=False)
                except Exception:
                    pass
                finally:
                    self._heal_lock.release()
            threading.Thread(target=_heal, daemon=True).start()
        # Nav2 挂了 / 无活动 goal 时不要一直刷 navigating，否则网页以为还在导
        if self.nav_state == 'navigating' and self.nav_goal_handle is None:
            if not self.nav_client.server_is_ready():
                self._set_nav_status('idle', 'nav2_gone')
        # 到达/取消/失败是瞬时事件：过几秒清回 idle，避免 1Hz 重发 + TRANSIENT_LOCAL
        # 让网页反复弹「已到达目标」
        if self.nav_state in ('reached', 'aborted', 'canceled'):
            age = time.time() - getattr(self, '_nav_state_ts', 0)
            if age > 2.5:
                self._set_nav_status('idle', '')
        status = service_status()
        status['nav'] = self.nav_state
        self.pub_sys_status.publish(String(data=json.dumps(status)))
        # 周期性重发导航状态（配合 TRANSIENT_LOCAL），防止 reloc 漏订阅
        self._set_nav_status(self.nav_state, getattr(self, '_nav_detail', '') or '')

    def _set_nav_status(self, state, detail):
        prev = getattr(self, 'nav_state', None)
        self.nav_state = state
        self._nav_detail = detail or ''
        if prev != state:
            self._nav_state_ts = time.time()
        # 同状态同 detail 的周期重发：只刷新 latch，内容不变
        self.pub_nav_status.publish(
            String(data=state if not detail else f'{state}:{detail}'))


def main(args=None):
    global _web_ops_node
    rclpy.init(args=args)
    node = WebOpsNode()
    _web_ops_node = node
    # 启动时先清掉叠出来的重负载孤儿
    try:
        stop_video_feeder()
        collapse_map_building(keep_one=True)
    except Exception:
        pass
    server = ThreadingHTTPServer(('0.0.0.0', 8090), ApiHandler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    threading.Thread(target=video_idle_reaper, daemon=True).start()
    node.get_logger().info('HTTP API listening on 8090')
    try:
        ensure_oem700_driver()
    except Exception as exc:
        node.get_logger().warning('OEM700 driver start failed: %s' % exc)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # ExternalShutdownException when parent kills the process — not a real fault
        if type(e).__name__ != 'ExternalShutdownException':
            node.get_logger().error('spin stopped: %s' % e)
    finally:
        try:
            stop_video_feeder()
        except Exception:
            pass
        try:
            server.shutdown()
        except Exception:
            pass
        try:
            node._viz_exec.shutdown(timeout_sec=1.0)
            node._viz_node.destroy_node()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
