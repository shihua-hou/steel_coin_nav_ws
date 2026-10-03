#!/usr/bin/env python3
"""lio_tf_bridge: Nav2 odom + Super-LIO 里程计桥接。

默认 odom_source:=dog：
  - /odom_dog → /odom_nav + TF odom→base_footprint
  - 站立：零速航向锁（含离锁迟滞）+ 陀螺零偏估计
  - 行走：直接跟 SDK 平面位姿（不再做重互补滤波，避免和 AMCL 打架）
  - Super-LIO 自行广播 TF world→imu

odom_source:=lio：旧行为，平面里程计也来自 Super-LIO（易漂）。
"""
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, TwistWithCovariance
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def quat_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )


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


def yaw_from_quat(q):
    fx, fy, _ = quat_rotate(q, (1.0, 0.0, 0.0))
    return math.atan2(fy, fx)


def _stamp_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class LioTfBridge(Node):
    def __init__(self):
        super().__init__('lio_tf_bridge')
        self.declare_parameter('lidar_x', 0.0)
        self.declare_parameter('lidar_y', 0.0)
        self.declare_parameter('lidar_z', 0.0)
        self.declare_parameter('lidar_roll', 0.0)
        self.declare_parameter('lidar_pitch', 0.0)
        self.declare_parameter('lidar_yaw', 0.0)
        self.declare_parameter('odom_source', 'dog')
        self.odom_source = str(self.get_parameter('odom_source').value).strip().lower()

        self.q_bl_lf = rpy_to_quat(
            self.get_parameter('lidar_roll').value,
            self.get_parameter('lidar_pitch').value,
            self.get_parameter('lidar_yaw').value)
        self.t_bl_lf = (
            self.get_parameter('lidar_x').value,
            self.get_parameter('lidar_y').value,
            self.get_parameter('lidar_z').value)
        self.q_lf_bl = quat_conj(self.q_bl_lf)
        self.t_lf_bl = quat_rotate(self.q_lf_bl,
                                   (-self.t_bl_lf[0], -self.t_bl_lf[1], -self.t_bl_lf[2]))

        self.tf_bc = TransformBroadcaster(self)
        self.tf_static_bc = StaticTransformBroadcaster(self)

        if self.odom_source == 'lio':
            self._publish_odom_world()
            self.create_timer(2.0, self._publish_odom_world)

        self.pub_odom = self.create_publisher(Odometry, '/odom_nav', 10)
        self.pub_odom2 = self.create_publisher(Odometry, '/odom', 10)

        # 零速航向锁：站立防陀螺零偏拧航向；离锁要持续运动，防尖峰解开锁
        self._yaw_corr = 0.0
        self._held_x = 0.0
        self._held_y = 0.0
        self._held_yaw = 0.0
        self._hold_since = None
        self._hold_active = False
        self._hold_lin = 0.04
        self._hold_ang = 0.028
        self._hold_need = 0.30
        self._hold_exit_need = 0.45
        self._hold_exit_since = None
        self._gyro_bias = 0.0
        self._bias_tau = 1.5
        self._last_t = None
        self._have_hold_pose = False

        self.sub_lio = self.create_subscription(
            Odometry, '/lio/odom', self.cb_lio, 10)
        if self.odom_source == 'dog':
            self.sub_dog = self.create_subscription(
                Odometry, '/odom_dog', self.cb_dog, 10)

        self.get_logger().info(
            f'lio_tf_bridge ready: odom_source={self.odom_source} '
            f'extrinsic T=({self.t_bl_lf[0]:.3f},{self.t_bl_lf[1]:.3f},{self.t_bl_lf[2]:.3f}) '
            f'pitch={self.get_parameter("lidar_pitch").value:.3f} '
            f'zero_yaw_hold=on imu_comp=off')

    def _publish_odom_world(self):
        t = TransformStamped()
        t.header.stamp.sec = 0
        t.header.stamp.nanosec = 0
        t.header.frame_id = 'odom'
        t.child_frame_id = 'world'
        t.transform.rotation.w = 1.0
        self.tf_static_bc.sendTransform(t)

    def _publish_nav_odom(self, x, y, yaw, twist_msg, stamp):
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_footprint'
        out.pose.pose.position.x = float(x)
        out.pose.pose.position.y = float(y)
        out.pose.pose.position.z = 0.0
        out.pose.pose.orientation.z = math.sin(yaw * 0.5)
        out.pose.pose.orientation.w = math.cos(yaw * 0.5)
        out.twist = twist_msg
        self.pub_odom.publish(out)
        self.pub_odom2.publish(out)

        fp = TransformStamped()
        fp.header.stamp = stamp
        fp.header.frame_id = 'odom'
        fp.child_frame_id = 'base_footprint'
        fp.transform.translation.x = float(x)
        fp.transform.translation.y = float(y)
        fp.transform.translation.z = 0.0
        fp.transform.rotation.z = math.sin(yaw * 0.5)
        fp.transform.rotation.w = math.cos(yaw * 0.5)
        self.tf_bc.sendTransform(fp)

    def _make_twist(self, vx, vy, wz):
        tw = TwistWithCovariance()
        tw.twist.linear.x = float(vx)
        tw.twist.linear.y = float(vy)
        tw.twist.angular.z = float(wz)
        return tw

    def _blend(self, prev, meas, dt, tau):
        if tau <= 1e-6 or dt <= 0.0:
            return meas
        a = 1.0 - math.exp(-dt / tau)
        return prev + (meas - prev) * max(0.0, min(1.0, a))

    def _freeze_pub(self, sdk_yaw, stamp):
        self._yaw_corr = self._held_yaw - sdk_yaw
        self._publish_nav_odom(
            self._held_x, self._held_y, self._held_yaw,
            self._make_twist(0.0, 0.0, 0.0), stamp)

    def cb_dog(self, msg: Odometry):
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        t = _stamp_sec(stamp)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        sdk_yaw = yaw_from_quat((q.x, q.y, q.z, q.w))
        if abs(p.x) > 500.0 or abs(p.y) > 500.0:
            return

        try:
            vx = float(msg.twist.twist.linear.x)
            vy = float(msg.twist.twist.linear.y)
            wz = float(msg.twist.twist.angular.z)
        except Exception:
            vx = vy = wz = 0.0

        dt = 0.0
        if self._last_t is not None:
            dt = t - self._last_t
            if dt < 0.0 or dt > 0.5:
                dt = 0.0
        self._last_t = t

        still = (abs(vx) < self._hold_lin and abs(vy) < self._hold_lin
                 and abs(wz) < self._hold_ang)
        now_wall = self.get_clock().now().nanoseconds * 1e-9

        if still:
            self._hold_exit_since = None
            if self._hold_since is None:
                self._hold_since = now_wall
            if dt > 0.0:
                self._gyro_bias = self._blend(self._gyro_bias, wz, dt, self._bias_tau)
            if (not self._hold_active) and (now_wall - self._hold_since) >= self._hold_need:
                self._hold_active = True
                self._held_x = float(p.x)
                self._held_y = float(p.y)
                self._held_yaw = sdk_yaw + self._yaw_corr
                self._have_hold_pose = True
            if self._hold_active and self._have_hold_pose:
                self._freeze_pub(sdk_yaw, stamp)
                return
        else:
            self._hold_since = None
            if self._hold_active and self._have_hold_pose:
                if self._hold_exit_since is None:
                    self._hold_exit_since = now_wall
                if (now_wall - self._hold_exit_since) < self._hold_exit_need:
                    self._freeze_pub(sdk_yaw, stamp)
                    return
                self._hold_active = False
                self._hold_exit_since = None
            else:
                self._hold_exit_since = None

        # 行走：直通 SDK（保留站立积累的 yaw_corr，避免离锁跳变）
        pub_yaw = sdk_yaw + self._yaw_corr
        self._held_x, self._held_y, self._held_yaw = float(p.x), float(p.y), pub_yaw
        self._have_hold_pose = True
        wz_c = wz - self._gyro_bias
        self._publish_nav_odom(
            p.x, p.y, pub_yaw, self._make_twist(vx, vy, wz_c), stamp)

    def cb_lio(self, msg: Odometry):
        if self.odom_source != 'lio':
            return
        p = msg.pose.pose
        q_cb = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        t_cb = (p.position.x, p.position.y, p.position.z)
        stamp = self.get_clock().now().to_msg()
        if abs(t_cb[0]) > 500.0 or abs(t_cb[1]) > 500.0:
            return
        q_ob = quat_mult(q_cb, self.q_lf_bl)
        t_ob = quat_rotate(q_cb, self.t_lf_bl)
        t_ob = (t_cb[0] + t_ob[0], t_cb[1] + t_ob[1], t_cb[2] + t_ob[2])
        yaw = yaw_from_quat(q_ob)
        self._publish_nav_odom(t_ob[0], t_ob[1], yaw, msg.twist, stamp)


def main():
    rclpy.init()
    node = LioTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
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
