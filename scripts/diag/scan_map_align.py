# 只读：用当前 /scan + /amcl_pose，在静态地图上做小范围搜索，看激光和地图墙线最贴合时
# 需要把定位挪多少（dx, dy, dθ）。挪动量≈0 → 定位准；≥0.1 m → 定位偏。
import math, re, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.ndimage import distance_transform_edt

MAP = '/home/linaro/steel_coin_nav_ws/maps/map_live003'
y = open(MAP + '.yaml').read()
res = float(re.search(r'resolution:\s*([\d.]+)', y).group(1))
org = [float(v) for v in re.search(r'origin:\s*\[([^\]]+)\]', y).group(1).split(',')[:2]]
f = open(MAP + '.pgm', 'rb').read()
hdr = re.match(rb'P5\s+(?:#.*\s+)*(\d+)\s+(\d+)\s+(\d+)\s', f)
W, H, _ = map(int, hdr.groups())
img = np.frombuffer(f[hdr.end():], dtype=np.uint8)[:W * H].reshape(H, W)
occ = img < 50
# PGM 第 0 行是地图最上方（y 最大）
dt = distance_transform_edt(~occ) * res


def dist_at(x, y):
    j = ((x - org[0]) / res).astype(int)
    i = (H - 1 - ((y - org[1]) / res)).astype(int)
    ok = (i >= 0) & (i < H) & (j >= 0) & (j < W)
    d = np.full(x.shape, 1.0)
    d[ok] = dt[i[ok], j[ok]]
    return np.minimum(d, 0.5)


rclpy.init()
n = Node('align_probe')
st = {}
n.create_subscription(LaserScan, '/scan', lambda m: st.setdefault('scans', []).append(m), qos_profile_sensor_data)
n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: st.__setitem__('pose', m),
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
t = time.time()
while (len(st.get('scans', [])) < 5 or 'pose' not in st) and time.time() - t < 15:
    rclpy.spin_once(n, timeout_sec=0.2)
p = st['pose'].pose.pose
q = p.orientation
px, py = p.position.x, p.position.y
pth = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
xs, ys = [], []
for s in st['scans'][-5:]:
    r = np.array(s.ranges)
    a = s.angle_min + np.arange(len(r)) * s.angle_increment
    ok = np.isfinite(r) & (r < 6.0)
    xs.append(r[ok] * np.cos(a[ok])); ys.append(r[ok] * np.sin(a[ok]))
lx, ly = np.concatenate(xs), np.concatenate(ys)
print('amcl pose (%.3f, %.3f, %.1f deg), scan pts %d' % (px, py, math.degrees(pth), len(lx)))


def score(dx, dy, dth):
    th = pth + dth
    wx = px + dx + math.cos(th) * lx - math.sin(th) * ly
    wy = py + dy + math.sin(th) * lx + math.cos(th) * ly
    return dist_at(wx, wy).mean()


base = score(0, 0, 0)
best = (base, 0, 0, 0)
for dth in np.radians(np.arange(-6, 6.01, 1.0)):
    for dx in np.arange(-0.25, 0.2501, 0.025):
        for dy in np.arange(-0.25, 0.2501, 0.025):
            sc = score(dx, dy, dth)
            if sc < best[0]:
                best = (sc, dx, dy, dth)
sc, dx, dy, dth = best
print('mean scan-to-wall distance: now %.3f m  ->  best %.3f m' % (base, sc))
print('best correction: dx=%+.3f dy=%+.3f (|%.3f| m)  dth=%+.1f deg' % (dx, dy, math.hypot(dx, dy), math.degrees(dth)))
# 机体系表达（前/左）
fx = math.cos(pth) * dx + math.sin(pth) * dy
fy = -math.sin(pth) * dx + math.cos(pth) * dy
print('in dog frame: forward %+.3f m, left %+.3f m' % (fx, fy))
