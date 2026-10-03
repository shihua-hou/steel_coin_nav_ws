#!/usr/bin/env python3
"""Sole map→odom publisher while RTK fusion is on.

RTK fixed (fresh /rtk/map_pose) supplies x/y, and yaw when the message says
the dual-antenna heading is valid. Otherwise AMCL's /amcl_pose is used.
odom→base_footprint stays with the FAST-LIO odom filter.
"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, TransformStamped
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from oem700_rtk.nmea_codec import yaw_wrap


def _yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _quat(yaw):
    quat = Quaternion()
    quat.z = math.sin(yaw * 0.5)
    quat.w = math.cos(yaw * 0.5)
    return quat


def _clamp(value, low, high):
    return max(low, min(high, value))


def _lerp_angle(current, target, alpha):
    return yaw_wrap(current + alpha * yaw_wrap(target - current))


class GlobalPoseArbiter(Node):
    def __init__(self):
        super().__init__("global_pose_arbiter")
        self.declare_parameter("rtk_timeout", 1.5)
        self.declare_parameter("amcl_timeout", 1.0)
        self.declare_parameter("max_jump_xy", 1.5)
        self.declare_parameter("max_jump_yaw", 0.52)
        self.declare_parameter("correction_alpha", 0.2)
        self.declare_parameter("max_correction_step", 0.5)
        self.declare_parameter("max_yaw_step", 0.6)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        self._rtk = None
        self._rtk_t = None
        self._amcl = None
        self._amcl_t = None
        self._filtered = None
        self._seeded = False
        self._rtk_accepted = False
        self._source = "hold"

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_pub = TransformBroadcaster(self)
        self._src_pub = self.create_publisher(String, "/gnss/pose_source", 10)
        self._init_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/rtk/map_pose", self._on_rtk, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10)
        period = 1.0 / max(float(self.get_parameter("publish_rate").value), 1.0)
        self.create_timer(period, self._tick)

    def _on_rtk(self, msg):
        self._rtk = msg
        self._rtk_t = self._now()

    def _on_amcl(self, msg):
        self._amcl = msg
        self._amcl_t = self._now()

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _fresh(self, stamp, timeout_name):
        if stamp is None:
            return False
        return (self._now() - stamp) <= float(self.get_parameter(timeout_name).value)

    def _pose_of(self, msg):
        p = msg.pose.pose
        yaw_ok = float(msg.pose.covariance[35]) >= 0.0
        return p.position.x, p.position.y, _yaw_of(p.orientation), yaw_ok

    def _global_from_odom(self, ox, oy, oyaw):
        if self._filtered is None:
            return None
        tx, ty, tyaw = self._filtered
        c = math.cos(tyaw)
        s = math.sin(tyaw)
        gx = c * ox - s * oy + tx
        gy = s * ox + c * oy + ty
        return gx, gy, yaw_wrap(oyaw + tyaw)

    def _amcl_near_origin(self):
        if not self._fresh(self._amcl_t, "amcl_timeout") or self._amcl is None:
            return True
        ax, ay, _ayaw, _ok = self._pose_of(self._amcl)
        return math.hypot(ax, ay) < 0.4

    def _within_gate(self, target, ox, oy, oyaw):
        current = self._global_from_odom(ox, oy, oyaw)
        if current is None:
            return True
        dx = target[0] - current[0]
        dy = target[1] - current[1]
        # Yaw is not a reason to drop the fix. Rotate-to-heading turns the dog
        # well past max_jump_yaw; rejecting that sample freezes map→odom and
        # the controller then sees an empty path.
        if math.hypot(dx, dy) > float(self.get_parameter("max_jump_xy").value):
            return False
        return True

    def _choose(self, ox, oy, oyaw):
        held = self._global_from_odom(ox, oy, oyaw)
        amcl_xyyaw = None
        if self._fresh(self._amcl_t, "amcl_timeout") and self._amcl is not None:
            ax, ay, ayaw, _ok = self._pose_of(self._amcl)
            amcl_xyyaw = (ax, ay, ayaw)

        rtk_ok = self._fresh(self._rtk_t, "rtk_timeout") and self._rtk is not None
        if rtk_ok:
            rx, ry, ryaw, yaw_ok = self._pose_of(self._rtk)
            if yaw_ok:
                yaw = ryaw
            elif amcl_xyyaw is not None:
                yaw = amcl_xyyaw[2]
            elif held is not None:
                yaw = held[2]
            else:
                yaw = oyaw
            target = (rx, ry, yaw)
            # The placeholder AMCL pose is (0, 0). The first fixed solution may
            # sit far from that origin; later samples still have to pass the gate.
            if (not self._rtk_accepted and self._amcl_near_origin()) or self._within_gate(
                    target, ox, oy, oyaw):
                return target, "rtk"
            self.get_logger().warning(
                "RTK jump rejected; staying on AMCL",
                throttle_duration_sec=2.0)

        if amcl_xyyaw is not None and self._within_gate(amcl_xyyaw, ox, oy, oyaw):
            return amcl_xyyaw, "amcl"
        if held is not None:
            return held, "hold"
        return (0.0, 0.0, 0.0), "hold"

    def _correction(self, global_pose, ox, oy, oyaw):
        gx, gy, gyaw = global_pose
        tyaw = yaw_wrap(gyaw - oyaw)
        c = math.cos(tyaw)
        s = math.sin(tyaw)
        tx = gx - (c * ox - s * oy)
        ty = gy - (s * ox + c * oy)
        return tx, ty, tyaw

    def _smooth(self, target):
        alpha = _clamp(float(self.get_parameter("correction_alpha").value), 0.0, 1.0)
        if self._filtered is None or alpha >= 1.0:
            blended = target
        else:
            cx, cy, cyaw = self._filtered
            blended = (
                cx + alpha * (target[0] - cx),
                cy + alpha * (target[1] - cy),
                _lerp_angle(cyaw, target[2], alpha),
            )
        if self._filtered is None:
            return blended
        cx, cy, cyaw = self._filtered
        dx = blended[0] - cx
        dy = blended[1] - cy
        dist = math.hypot(dx, dy)
        max_step = float(self.get_parameter("max_correction_step").value)
        if max_step >= 0.0 and dist > max_step and dist > 1e-9:
            scale = max_step / dist
            nx = cx + dx * scale
            ny = cy + dy * scale
        else:
            nx, ny = blended[0], blended[1]
        max_yaw = float(self.get_parameter("max_yaw_step").value)
        dyaw = yaw_wrap(blended[2] - cyaw)
        if max_yaw >= 0.0 and abs(dyaw) > max_yaw:
            dyaw = math.copysign(max_yaw, dyaw)
        return nx, ny, yaw_wrap(cyaw + dyaw)

    def _lookup_odom(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                str(self.get_parameter("odom_frame").value),
                str(self.get_parameter("base_frame").value),
                rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        return t.x, t.y, _yaw_of(tf.transform.rotation)

    def _tick(self):
        odom = self._lookup_odom()
        if odom is None:
            return
        ox, oy, oyaw = odom
        global_pose, source = self._choose(ox, oy, oyaw)
        correction = self._correction(global_pose, ox, oy, oyaw)
        # The first fixed solution can sit tens of metres from the AMCL
        # placeholder. Easing that correction in 0.5 m steps leaves the
        # published pose outside the jump gate, so every later fix is rejected
        # and the dog stays on the placeholder.
        if source == "rtk" and not self._rtk_accepted:
            self._filtered = correction
        else:
            self._filtered = self._smooth(correction)
        self._source = source
        self._publish_tf(self._filtered)
        src = String()
        src.data = source
        self._src_pub.publish(src)
        if source == "rtk":
            self._rtk_accepted = True
        if source == "rtk" and not self._seeded:
            self._seed_initial(global_pose)
            self._seeded = True

    def _publish_tf(self, correction):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = str(self.get_parameter("map_frame").value)
        transform.child_frame_id = str(self.get_parameter("odom_frame").value)
        transform.transform.translation.x = correction[0]
        transform.transform.translation.y = correction[1]
        transform.transform.translation.z = 0.0
        transform.transform.rotation = _quat(correction[2])
        self._tf_pub.sendTransform(transform)

    def _seed_initial(self, global_pose):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(global_pose[0])
        msg.pose.pose.position.y = float(global_pose[1])
        msg.pose.pose.orientation = _quat(float(global_pose[2]))
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        self._init_pub.publish(msg)
        self.get_logger().info(
            "seeded /initialpose from RTK x=%.2f y=%.2f yaw=%.1fdeg" % (
                global_pose[0], global_pose[1], math.degrees(global_pose[2])))


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPoseArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
