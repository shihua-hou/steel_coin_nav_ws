# -*- coding: utf-8 -*-
"""ground_check.py v2 - 只对最低处点云(真实地面)做平面拟合
用法: python3 ground_check.py
判定: TILT_FROM_VERTICAL < 8°  = 地图水平合格;  8~15° = 轻微(初始化扰动);  >15° = 异常
"""
import rclpy, struct, math
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class GroundCheck(Node):
    def __init__(self):
        super().__init__('ground_check')
        self.sub = self.create_subscription(PointCloud2, '/lio/cloud_world', self.cb, 5)
        self.done = False

    def cb(self, msg):
        offs = {f.name: f.offset for f in msg.fields}
        step = msg.point_step
        n = len(msg.data) // step
        if n < 100:
            return
        xs, ys, zs = [], [], []
        stride = max(1, n // 60000)
        for i in range(0, n, stride):
            b = i * step
            try:
                xs.append(struct.unpack_from('<f', msg.data, b + offs['x'])[0])
                ys.append(struct.unpack_from('<f', msg.data, b + offs['y'])[0])
                zs.append(struct.unpack_from('<f', msg.data, b + offs['z'])[0])
            except Exception:
                pass
        if len(xs) < 500:
            return

        # --- 方式1: 全点拟合(旧) ---
        a1, b1, c1, tilt1 = self._fit(xs, ys, zs)

        # --- 方式2: 只取最低 15% 的点(地面) ---
        zs_sorted = sorted(zs)
        thr = zs_sorted[max(1, len(zs_sorted) * 15 // 100)]
        gx, gy, gz = [], [], []
        for x, y, z in zip(xs, ys, zs):
            if z <= thr:
                gx.append(x); gy.append(y); gz.append(z)
        if len(gx) >= 200:
            a2, b2, c2, tilt2 = self._fit(gx, gy, gz)
        else:
            a2 = b2 = c2 = float('nan'); tilt2 = float('nan')

        print(f'POINTS_ALL={len(xs)}  LOW_15PCT={len(gx)} (z_thr={thr:.2f})')
        print(f'[ALL]   PLANE z={a1:.4f}x+{b1:.4f}y+{c1:.4f}  TILT={tilt1:.2f} deg')
        print(f'[GROUND] PLANE z={a2:.4f}x+{b2:.4f}y+{c2:.4f}  TILT={tilt2:.2f} deg  <-- 以这个为准')
        print(f'Z_RANGE=[{min(zs):.2f},{max(zs):.2f}] MEAN={sum(zs)/len(zs):.2f}')
        self.done = True

    @staticmethod
    def _fit(xs, ys, zs):
        n = len(xs)
        sx = sum(xs) / n; sy = sum(ys) / n; sz = sum(zs) / n
        sxx = sum((x - sx) ** 2 for x in xs)
        syy = sum((y - sy) ** 2 for y in ys)
        sxy = sum((x - sx) * (y - sy) for x, y in zip(xs, ys))
        sxz = sum((x - sx) * (z - sz) for x, z in zip(xs, zs))
        syz = sum((y - sy) * (z - sz) for y, z in zip(ys, zs))
        det = sxx * syy - sxy * sxy
        if abs(det) < 1e-12:
            return 0, 0, sz, 0.0
        a = (syy * sxz - sxy * syz) / det
        b = (sxx * syz - sxy * sxz) / det
        c = sz - a * sx - b * sy
        norm = math.sqrt(a * a + b * b + 1.0)
        nz = 1.0 / norm
        tilt = math.degrees(math.acos(abs(nz)))
        return a, b, c, tilt


def main():
    rclpy.init()
    node = GroundCheck()
    import time as _t
    t0 = _t.time()
    while not node.done and _t.time() - t0 < 8.0:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


main()
