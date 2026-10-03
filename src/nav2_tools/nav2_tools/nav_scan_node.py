#!/usr/bin/env python3
"""nav_scan_node — /lio/cloud_world → /scan，算法与建图页 3D 点云一致。

建图页 (web_ui/index.html rebuildMapPoints) 已验证正确：
  1. /lio/cloud_world（world）
  2. levelMat = rot_align(nominalUp(pitch_down, roll), +Z)  # 俯仰校平
  3. hh = qz + lidar_z                                     # 离地高度
  4. 高度带过滤后画点

本节点做同样的事，只是把当前帧变到 imu（雷达/IMU 机系）后再校平，
取高度带投影成 LaserScan，并平移到 base_footprint（雷达在基座前方 lidar_x）。

禁止再做 y=-y / 外参方向翻转等「对冲」——左右前后以建图页为准。
"""
from __future__ import annotations

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, LaserScan
from tf2_ros import Buffer, TransformListener
import sensor_msgs_py.point_cloud2 as pc2


EXTRINSIC_YAML = '/home/linaro/steel_coin_nav_ws/config/lidar_extrinsic.yaml'
NOMINAL_PITCH_DOWN = 0.342586  # ~19.6 deg
NOMINAL_ROLL = 0.0


def load_extrinsic(path=EXTRINSIC_YAML):
    pitch, roll, lidar_z = NOMINAL_PITCH_DOWN, NOMINAL_ROLL, 0.45
    try:
        with open(path, 'r') as f:
            for line in f:
                line = line.split('#', 1)[0].strip()
                if not line or ':' not in line:
                    continue
                k, v = line.split(':', 1)
                k, v = k.strip(), v.strip()
                if k == 'pitch_down':
                    pitch = float(v)
                elif k == 'roll':
                    roll = float(v)
                elif k == 'lidar_z':
                    lidar_z = float(v)
    except Exception:
        pass
    return pitch, roll, lidar_z


def nominal_up(pitch_down, roll):
    """与 web_ui nominalUp() / calib_lidar_imu_gravity.py 相同。"""
    sp, cp = math.sin(pitch_down), math.cos(pitch_down)
    sr, cr = math.sin(roll), math.cos(roll)
    return (-sp, sr * cp, cr * cp)


def rot_align(a, b):
    """3x3 行主序，把单位向量 a 旋到 b。等同 web levelMat() / pcd2pgm level_mat_from_normal。"""
    an = math.sqrt(sum(c * c for c in a)) or 1.0
    a = tuple(c / an for c in a)
    bn = math.sqrt(sum(c * c for c in b)) or 1.0
    b = tuple(c / bn for c in b)
    vx, vy, vz = (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
    c = a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
    s2 = vx*vx + vy*vy + vz*vz
    if s2 < 1e-12:
        if c > 0:
            return ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))
        return ((-1., 0., 0.), (0., -1., 0.), (0., 0., -1.))
    k = (1.0 - c) / s2
    return (
        (1 + (-vz*vz - vy*vy) * k, -vz + vx*vy*k, vy + vx*vz*k),
        (vz + vx*vy*k, 1 + (-vz*vz - vx*vx) * k, -vx + vy*vz*k),
        (-vy + vx*vz*k, vx + vy*vz*k, 1 + (-vy*vy - vx*vx) * k),
    )


def transform_to_mat(tf):
    q = tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )
    t = tf.transform.translation
    return R, (t.x, t.y, t.z)


def apply_R(R, px, py, pz):
    return (
        R[0][0] * px + R[0][1] * py + R[0][2] * pz,
        R[1][0] * px + R[1][1] * py + R[1][2] * pz,
        R[2][0] * px + R[2][1] * py + R[2][2] * pz,
    )


def apply_Rt(R, t, px, py, pz):
    x, y, z = apply_R(R, px, py, pz)
    return (x + t[0], y + t[1], z + t[2])


def rpy_to_quat(r, p, y):
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def quat_rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def leveled_lidar_xy(pitch_down, roll, mount_x=0.25, mount_y=0.0, mount_z=0.45):
    """校平后 IMU 原点相对 base_footprint 的平面偏移。

    俯仰会把 mount_z 耦合进水平距离：继续用裸 lidar_x=0.25 会系统性偏 ~23cm，
    激光贴不上墙、AMCL 也被带歪。与 lio_tf_bridge / static imu→base_link 同外参。
    """
    # bridge: lidar_pitch = -pitch_down
    q_bl_lf = rpy_to_quat(roll, -pitch_down, 0.0)
    t_lf_bl = quat_rotate(quat_conj(q_bl_lf), (-mount_x, -mount_y, -mount_z))
    Ra = rot_align(nominal_up(pitch_down, roll), (0.0, 0.0, 1.0))
    bl = apply_R(Ra, *t_lf_bl)
    return (-bl[0], -bl[1])


class NavScanNode(Node):
    def __init__(self):
        super().__init__('nav_scan_node')
        # 高度带：离地高度 hh（与建图页 MapGnd.minH/maxH 同语义，导航障碍带更窄）
        # 与 config/obstacle_height_band.yaml / stage2 / pcd2pgm / Go2W 一致
        self.z_min = float(self.declare_parameter('z_min', 0.05).value)
        self.z_max = float(self.declare_parameter('z_max', 0.85).value)
        self.range_min = float(self.declare_parameter('range_min', 0.40).value)
        self.range_max = float(self.declare_parameter('range_max', 8.0).value)
        self.angle_increment = float(self.declare_parameter('angle_increment', 0.01).value)

        pitch_down, roll, yaml_z = load_extrinsic()
        self.lidar_z = float(self.declare_parameter('lidar_z', yaml_z).value)
        # 安装平移（base_link→雷达），与 bridge.launch 一致；可被参数覆盖
        mount_x = float(self.declare_parameter('mount_x', 0.25).value)
        mount_y = float(self.declare_parameter('mount_y', 0.0).value)
        def_lx, def_ly = leveled_lidar_xy(
            pitch_down, roll, mount_x, mount_y, self.lidar_z)
        self.lidar_x = float(self.declare_parameter('lidar_x', def_lx).value)
        self.lidar_y = float(self.declare_parameter('lidar_y', def_ly).value)
        self._nbin = max(1, int(round((2.0 * math.pi) / self.angle_increment)))
        self._miss_tf = 0
        self._ok = 0

        # 与建图页 levelMat() / lidar_extrinsic 一致：~19.6° 下仰校平（非 Go2 平装）
        up = nominal_up(pitch_down, roll)
        self.R_align = rot_align(up, (0.0, 0.0, 1.0))

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/lio/cloud_world', self.cb_cloud, qos_profile_sensor_data)

        self.get_logger().info(
            'nav_scan(~19.6°校平): pitch_down=%.1f° roll=%.1f° lidar_z=%.2f '
            'band=[%.2f,%.2f] lidar_xy=(%.3f,%.3f) mount=(%.2f,%.2f) → /scan(base_footprint)'
            % (math.degrees(pitch_down), math.degrees(roll), self.lidar_z,
               self.z_min, self.z_max, self.lidar_x, self.lidar_y, mount_x, mount_y))

    def cb_cloud(self, msg: PointCloud2):
        # 点云在 world。流程与建图页一致：
        #   imu←world → R_align(~19.6°校平) → 高度带 → +lidar_xy → /scan(base_footprint)
        # 必须用点云时刻的 world→imu，不能用狗 odom 时刻去套 LIO 点云（会漂）。
        now = self.get_clock().now()
        try:
            tf_fp = self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except Exception:
            self._miss_tf += 1
            if self._miss_tf % 30 == 1:
                self.get_logger().warn('waiting TF odom→base_footprint')
            return
        tf_odom_t = rclpy.time.Time.from_msg(tf_fp.header.stamp)
        # 狗里程计偶发卡顿：0.4s 太紧会导致导航中大量丢 /scan，AMCL 只能跟纯里程计漂
        if (now - tf_odom_t).nanoseconds > int(0.80 * 1e9):
            self._miss_tf += 1
            if self._miss_tf % 20 == 1:
                self.get_logger().warn(
                    'odom→base_footprint TF stale (%.0fms), skip scan'
                    % ((now - tf_odom_t).nanoseconds / 1e6))
            return

        cloud_t = rclpy.time.Time.from_msg(msg.header.stamp)
        tf = None
        try:
            tf = self.tf_buffer.lookup_transform(
                'imu', msg.header.frame_id, cloud_t,
                timeout=Duration(seconds=0.05))
        except Exception:
            try:
                tf = self.tf_buffer.lookup_transform(
                    'imu', msg.header.frame_id, rclpy.time.Time(),
                    timeout=Duration(seconds=0.05))
            except Exception:
                self._miss_tf += 1
                if self._miss_tf % 30 == 1:
                    self.get_logger().warn(
                        'waiting TF imu←%s (miss=%d)'
                        % (msg.header.frame_id, self._miss_tf))
                return

        R, tvec = transform_to_mat(tf)
        Ra = self.R_align
        zoff = self.lidar_z
        lx, ly = self.lidar_x, self.lidar_y
        bins = [self.range_max + 1.0] * self._nbin
        n_keep = 0

        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            # 1) 世界点 → IMU 机系 imu
            bx, by, bz = apply_Rt(R, tvec, float(p[0]), float(p[1]), float(p[2]))
            # 2) 建图页同款 ~19.6° 校平
            qx, qy, qz = apply_R(Ra, bx, by, bz)
            # 3) 离地高度
            hh = qz + zoff
            if hh < self.z_min or hh > self.z_max:
                continue
            # 4) 平移到 base_footprint（雷达在前方；已含俯仰耦合）
            x = qx + lx
            y = qy + ly
            rng = math.hypot(x, y)
            if rng < self.range_min or rng > self.range_max:
                continue
            ang = math.atan2(y, x)
            idx = int((ang + math.pi) / self.angle_increment) % self._nbin
            if rng < bins[idx]:
                bins[idx] = rng
            n_keep += 1

        scan = LaserScan()
        # /scan 在 base_footprint：用当前 odom→footprint 时间戳，避免 min(imu,odom) 偏旧
        # 导致 AMCL/代价地图 TF 外推失败
        scan.header.stamp = tf_fp.header.stamp
        scan.header.frame_id = 'base_footprint'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = [
            float(r) if r <= self.range_max else float('inf') for r in bins]
        self.pub.publish(scan)
        self._ok += 1
        if self._ok % 50 == 1:
            valid = sum(1 for r in bins if r <= self.range_max)
            self.get_logger().info(
                'scan ok (body+19.6°align): band_pts~%d beams=%d/%d'
                % (n_keep, valid, self._nbin))


def main():
    rclpy.init()
    node = NavScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if 'ExternalShutdown' not in type(e).__name__:
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
