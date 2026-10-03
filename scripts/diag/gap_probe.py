# 只读：狗停住（cmd_vel≈0 且导航中）时，从局部代价图量「狗正前方横截面上的可通行宽度」。
# 每次停住最多记一次快照（间隔 ≥3 s），写 /tmp/gapprobe.log，运行 15 分钟自动退出。
import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, PolygonStamped

LOG = open('/tmp/gapprobe.log', 'a', buffering=1)


def log(s):
    LOG.write(time.strftime('%H:%M:%S ') + s + '\n')


class Probe(Node):
    def __init__(self):
        super().__init__('gap_probe')
        q = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.RELIABLE)
        self.cm = None
        self.fp = None
        self.cmd = (0.0, 0.0)
        self.cmd_t = 0.0
        self.last_snap = 0.0
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: setattr(self, 'cm', m), q)
        self.create_subscription(PolygonStamped, '/local_costmap/published_footprint',
                                 lambda m: setattr(self, 'fp', m), 5)
        self.create_subscription(Twist, '/cmd_vel_nav', self.on_cmd, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_timer(0.5, self.tick)
        self.moving_seen = 0.0
        log('gap probe started')

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)
        self.cmd_t = time.time()
        if abs(m.linear.x) > 0.03:
            self.moving_seen = time.time()

    def tick(self):
        now = time.time()
        # 「卡住」：最近 20 s 内动过（在导航），但最近 1.5 s 前进速度≈0
        stuck = (now - self.moving_seen < 20.0) and (now - self.moving_seen > 1.5)
        if not stuck or now - self.last_snap < 3.0 or self.cm is None or self.fp is None:
            return
        self.last_snap = now
        self.snapshot()

    def snapshot(self):
        m = self.cm
        r = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        a = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        pts = np.array([(p.x, p.y) for p in self.fp.polygon.points])
        c = pts.mean(axis=0)
        # 车头方向：footprint 最长边方向，取前半（polygon 第 0、1 点是 +x 两角？用 0→3 边中点）
        front = (pts[0] + pts[3]) / 2.0 if len(pts) == 4 else pts[0]
        h = math.atan2(front[1] - c[1], front[0] - c[0])
        fx, fy = math.cos(h), math.sin(h)
        lx, ly = -fy, fx
        lines = []
        for ahead in (0.35, 0.45, 0.55, 0.70, 0.85):
            free = []
            for s in np.arange(-1.2, 1.2001, 0.025):
                x = c[0] + fx * ahead + lx * s
                y = c[1] + fy * ahead + ly * s
                i, j = int((y - oy) / r), int((x - ox) / r)
                if 0 <= i < a.shape[0] and 0 <= j < a.shape[1]:
                    free.append((s, a[i, j] < 100))   # 100 = lethal（OccupancyGrid 里 254→100）
                else:
                    free.append((s, False))
            # 以车中线为中心找包含 s=0 附近的最长连续可通行段
            best, cur = (0, 0, 0), None
            for s, f in free:
                if f and cur is None:
                    cur = s
                if (not f) and cur is not None:
                    w = s - cur
                    if w > best[0]:
                        best = (w, cur, s)
                    cur = None
            if cur is not None:
                w = free[-1][0] - cur
                if w > best[0]:
                    best = (w, cur, free[-1][0])
            lines.append('前 %.2f m: 最宽通道 %.2f m（横向 %+.2f~%+.2f）' % (ahead, best[0], best[1], best[2]))
        w = np.linalg.norm(pts[1] - pts[0]), np.linalg.norm(pts[2] - pts[1])
        log('STUCK snapshot  footprint %.2fx%.2f  cmd v=%.2f w=%.2f\n    ' % (max(w), min(w), self.cmd[0], self.cmd[1])
            + '\n    '.join(lines))


rclpy.init()
n = Probe()
t0 = time.time()
while rclpy.ok() and time.time() - t0 < 900:
    rclpy.spin_once(n, timeout_sec=0.2)
log('gap probe exit')
