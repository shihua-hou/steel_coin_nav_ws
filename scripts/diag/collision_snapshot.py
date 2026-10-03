# 只读：盯 /tmp/nav2.log，每出现一次 "detected collision ahead" 就对局部代价图拍快照：
# 离狗轮廓 0.30 m 内的致命格（cost 254 → OccupancyGrid 100）在机体系的位置（+x 前，+y 左）。
# 写 /tmp/collprobe.log，20 分钟后自动退出。
import math, os, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PolygonStamped, Twist

LOG = open('/tmp/collprobe.log', 'a', buffering=1)
HX, HY = 0.335, 0.20   # 发布轮廓半长/半宽（0.65×0.38 + padding 0.01）


def log(s):
    LOG.write(time.strftime('%H:%M:%S ') + s + '\n')


class P(Node):
    def __init__(self):
        super().__init__('coll_probe')
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.cm = self.fp = None
        self.cmd = (0.0, 0.0)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: setattr(self, 'cm', m), q)
        self.create_subscription(PolygonStamped, '/local_costmap/published_footprint',
                                 lambda m: setattr(self, 'fp', m), 5)
        self.create_subscription(Twist, '/cmd_vel_nav', lambda m: setattr(self, 'cmd', (m.linear.x, m.angular.z)), 10)
        self.f = open('/tmp/nav2.log', 'rb')
        self.f.seek(0, os.SEEK_END)
        self.create_timer(0.1, self.poll)
        self.n = 0
        log('coll probe started')

    def poll(self):
        data = self.f.read()
        if not data:
            return
        for line in data.replace(b'\x00', b'').decode('utf-8', 'ignore').splitlines():
            if 'detected collision ahead' in line:
                self.snap()
            elif 'Goal failed' in line or 'Goal succeeded' in line or 'Begin navigating' in line:
                log('--- ' + line.split(']: ')[-1].strip())

    def snap(self):
        if self.cm is None or self.fp is None:
            log('collision! (no costmap yet)')
            return
        self.n += 1
        m = self.cm
        r = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        a = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        pts = np.array([(p.x, p.y) for p in self.fp.polygon.points])
        c = pts.mean(axis=0)
        front = (pts[0] + pts[3]) / 2.0
        h = math.atan2(front[1] - c[1], front[0] - c[0])
        ii, jj = np.nonzero(a == 100)
        if not len(ii):
            log('collision #%d: no lethal cells in local costmap at all  cmd=%.2f,%.2f' % (self.n, *self.cmd))
            return
        dx = ox + (jj + 0.5) * r - c[0]
        dy = oy + (ii + 0.5) * r - c[1]
        bx = math.cos(h) * dx + math.sin(h) * dy
        by = -math.sin(h) * dx + math.cos(h) * dy
        ex, ey = np.abs(bx) - HX, np.abs(by) - HY
        d = np.hypot(np.maximum(ex, 0), np.maximum(ey, 0)) + np.minimum(np.maximum(ex, ey), 0)
        near = np.argsort(d)[:8]
        sides = []
        for k in near:
            if d[k] > 0.30:
                break
            where = ('前' if bx[k] > HX else '后' if bx[k] < -HX else '') + ('左' if by[k] > HY else '右' if by[k] < -HY else '')
            sides.append('(%+.2f,%+.2f) %s %.3fm' % (bx[k], by[k], where or '轮廓内', d[k]))
        log('collision #%d  cmd v=%.2f w=%.2f  nearest lethal %.3f m  inside=%d\n    %s' % (
            self.n, self.cmd[0], self.cmd[1], d.min(), int((d < 0).sum()),
            '\n    '.join(sides) if sides else '(0.30 m 内没有致命格)'))


rclpy.init()
n = P()
t0 = time.time()
while rclpy.ok() and time.time() - t0 < 1200:
    rclpy.spin_once(n, timeout_sec=0.1)
log('coll probe exit')
