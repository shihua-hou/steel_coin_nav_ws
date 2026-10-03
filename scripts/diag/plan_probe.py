# 只读：记录导航过程中「全局路线换了多少次、每次横向偏多少」和「狗停下/原地转的时段」。
# 写 /tmp/planprobe.log，20 分钟后自动退出。
import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist

LOG = open('/tmp/planprobe.log', 'a', buffering=1)


def log(s):
    LOG.write(time.strftime('%H:%M:%S ') + s + '\n')


def resample(p, step=0.05):
    if len(p) < 2:
        return p
    d = np.r_[0, np.cumsum(np.hypot(*np.diff(p, axis=0).T))]
    s = np.arange(0, d[-1], step)
    return np.c_[np.interp(s, d, p[:, 0]), np.interp(s, d, p[:, 1])]


class P(Node):
    def __init__(self):
        super().__init__('plan_probe')
        self.prev = None
        self.n = 0
        self.cmd_state = None
        self.state_since = time.time()
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.on_cmd, 10)
        log('plan probe started')

    def on_plan(self, m):
        if not m.poses:
            return
        p = np.array([(q.pose.position.x, q.pose.position.y) for q in m.poses])
        L = float(np.sum(np.hypot(*np.diff(p, axis=0).T))) if len(p) > 1 else 0.0
        self.n += 1
        if self.prev is None or np.hypot(*(self.prev[-1] - p[-1])) > 0.3:
            log('plan #%d NEW GOAL  len %.2f m' % (self.n, L))
            self.prev = p
            return
        # 新路线前 3 m 每个点到旧路线的最近距离，取最大值 = 横向变化
        a = resample(p)
        a = a[np.r_[0, np.cumsum(np.hypot(*np.diff(a, axis=0).T))] < 3.0] if len(a) > 1 else a
        b = resample(self.prev)
        if len(a) and len(b):
            dmin = np.min(np.hypot(a[:, None, 0] - b[None, :, 0], a[:, None, 1] - b[None, :, 1]), axis=1)
            dev = float(dmin.max())
        else:
            dev = 0.0
        Lp = float(np.sum(np.hypot(*np.diff(self.prev, axis=0).T)))
        if dev > 0.08 or abs(L - Lp) > 0.3:
            log('plan #%d CHANGED  len %.2f -> %.2f m  max lateral shift (first 3 m) %.2f m' % (self.n, Lp, L, dev))
        self.prev = p

    def on_cmd(self, m):
        v, w = m.linear.x, m.angular.z
        st = 'stop' if abs(v) < 0.02 and abs(w) < 0.05 else ('rotate' if abs(v) < 0.02 else 'drive')
        now = time.time()
        if st != self.cmd_state:
            if self.cmd_state in ('stop', 'rotate') and now - self.state_since > 0.4:
                log('    dog %s for %.1f s' % (self.cmd_state, now - self.state_since))
            self.cmd_state = st
            self.state_since = now


rclpy.init()
n = P()
t0 = time.time()
while rclpy.ok() and time.time() - t0 < 1200:
    rclpy.spin_once(n, timeout_sec=0.2)
log('plan probe exit')
