#!/usr/bin/env python3
"""Publish a flattened, map-bounded FAST-LIO odometry stream for Nav2."""

import copy
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from isaac_go2_nav2.map_scan_simulator import read_map_yaml, read_pgm


def clamp(value, low, high):
    return max(low, min(high, value))


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def yaw_wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_from_yaw(yaw):
    quat = Quaternion()
    quat.x = 0.0
    quat.y = 0.0
    quat.z = math.sin(yaw * 0.5)
    quat.w = math.cos(yaw * 0.5)
    return quat


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


def qrot(q, v):
    x, y, z, _w = qmul(qmul(q, (v[0], v[1], v[2], 0.0)), qconj(q))
    return (x, y, z)


def rpy_from_quat(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(clamp(sinp, -1.0, 1.0))

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class FastlioNav2OdomFilter(Node):
    def __init__(self):
        super().__init__("fastlio_nav2_odom_filter")

        self.declare_parameter("input_odom_topic", "/Odometry")
        self.declare_parameter("output_odom_topic", "/Odometry_nav2")
        self.declare_parameter("odom_frame", "camera_init")
        self.declare_parameter("nav_base_frame", "body_nav")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("initial_x", -6.0)
        self.declare_parameter("initial_y", -4.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("truth_initial_x", -6.0)
        self.declare_parameter("truth_initial_y", -4.0)
        self.declare_parameter("truth_initial_yaw", 0.0)
        self.declare_parameter("map_margin", 0.15)
        self.declare_parameter("reject_out_of_map", True)
        self.declare_parameter("max_abs_z", 0.8)
        self.declare_parameter("max_abs_roll_pitch", 0.8)
        self.declare_parameter("max_linear_speed", 1.5)
        self.declare_parameter("max_angular_speed", 3.0)
        self.declare_parameter("reject_non_monotonic_timestamps", True)
        self.declare_parameter("max_timestamp_gap", -1.0)
        self.declare_parameter("max_position_jump", 0.75)
        self.declare_parameter("max_yaw_jump", 0.75)
        self.declare_parameter("publish_last_on_reject", True)
        # 连续拒收超过该秒数（FAST-LIO 发散）就停发 odom/TF，让 Nav2 因 TF 超时中止、狗停下；
        # 否则一直重复旧位姿 = 里程计冻结，Nav2 照常开 → 乱走乱撞（2026-10-01 实测）。<=0 不限。
        self.declare_parameter("max_hold_on_reject_sec", 1.0)
        # 单帧跳变（xy/yaw 跳、超速）不再永久冻结：吸收这一帧的跳变（输出保持连续），
        # 以新位置为基准继续。2 s 内吸收超过该次数 = FAST-LIO 在乱跳，停发让 Nav2 停车。
        # 旧逻辑一次跳变后每帧都和冻结旧值比 → 永远拒收（2026-10-01 走廊实测）。
        self.declare_parameter("max_absorb_per_2s", 4)
        self.declare_parameter("stamp_mode", "now")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("derive_twist", True)
        self.declare_parameter("truth_odom_topic", "")
        self.declare_parameter("truth_warn_distance", 0.25)
        self.declare_parameter("truth_warn_yaw", 0.15)
        self.declare_parameter("truth_log_period", 2.0)
        # 钢镚雷达低头 ~19.6°：FAST-LIO 的 camera_init = 启动时的 IMU 系，本身是斜的。
        # 给出 body→base_link 外参后输出 E^-1·P·E（odom = 启动时的水平机体系），
        # 否则直接取斜系的 x/y/yaw，前进距离缩 ~6%、转向后 yaw/位置都带误差。
        # 默认单位阵 = 不调平（Go2W 仿真里雷达平装）。
        for key, default in (("x", 0.0), ("y", 0.0), ("z", 0.0),
                             ("qx", 0.0), ("qy", 0.0), ("qz", 0.0), ("qw", 1.0)):
            self.declare_parameter("base_extrinsic_" + key, default)

        self.map_available = False
        self.map_min_x = 0.0
        self.map_min_y = 0.0
        self.map_max_x = 0.0
        self.map_max_y = 0.0
        self._load_map_bounds()

        self.last_valid_odom = None
        self.last_valid_raw_x = None
        self.last_valid_raw_y = None
        self.last_valid_yaw = None
        self.last_valid_time = None
        self.last_truth_pose = None
        self.last_truth_log_wall_time = 0.0
        self.reject_count = 0
        self.last_reject_log_wall_time = 0.0
        self.reject_since = None
        self.last_diverge_log_wall_time = 0.0
        self.offset = (0.0, 0.0, 0.0)  # 累计吸收的跳变（平面 SE2，作用在调平后的原始位姿上）
        self.absorb_times = []

        g = lambda k: float(self.get_parameter("base_extrinsic_" + k).value)
        q = (g("qx"), g("qy"), g("qz"), g("qw"))
        n = math.sqrt(sum(c * c for c in q)) or 1.0
        self.ext_q = tuple(c / n for c in q)
        self.ext_t = (g("x"), g("y"), g("z"))
        self.level_enabled = any(abs(c) > 1e-9 for c in self.ext_t + self.ext_q[:3])
        if self.level_enabled:
            self.get_logger().info(
                "Leveling FAST-LIO odom with body->base extrinsic t=%s q=%s"
                % (self.ext_t, tuple(round(c, 6) for c in self.ext_q))
            )

        self.pub = self.create_publisher(
            Odometry,
            str(self.get_parameter("output_odom_topic").value),
            20,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            str(self.get_parameter("input_odom_topic").value),
            self.on_odom,
            20,
        )
        truth_odom_topic = str(self.get_parameter("truth_odom_topic").value)
        if truth_odom_topic:
            self.create_subscription(Odometry, truth_odom_topic, self.on_truth_odom, 20)
            self.get_logger().info("Comparing Nav2 odom against simulation truth %s" % truth_odom_topic)
        self.get_logger().info(
            "Publishing Nav2 odom %s -> %s from %s"
            % (
                str(self.get_parameter("odom_frame").value),
                str(self.get_parameter("nav_base_frame").value),
                str(self.get_parameter("input_odom_topic").value),
            )
        )

    def _load_map_bounds(self):
        try:
            map_yaml = str(self.get_parameter("map_yaml").value)
            if not map_yaml:
                self.map_available = False
                self.get_logger().info("No static map bounds configured; FAST-LIO odom will not be map-bounds filtered")
                return
            map_info = read_map_yaml(map_yaml)
            width, height, _max_value, _pixels = read_pgm(map_info["image"])
            resolution = float(map_info["resolution"])
            origin = map_info["origin"]
            self.map_min_x = float(origin[0])
            self.map_min_y = float(origin[1])
            self.map_max_x = self.map_min_x + width * resolution
            self.map_max_y = self.map_min_y + height * resolution
            self.map_available = True
            self.get_logger().info(
                "Loaded map bounds x=[%.2f, %.2f] y=[%.2f, %.2f] from %s"
                % (self.map_min_x, self.map_max_x, self.map_min_y, self.map_max_y, map_yaml)
            )
        except Exception as exc:
            self.map_available = False
            self.get_logger().warning("Could not load map bounds: %s" % exc)

    def _stamp(self, msg):
        stamp_mode = str(self.get_parameter("stamp_mode").value).lower()
        if stamp_mode == "odom":
            return msg.header.stamp
        return self.get_clock().now().to_msg()

    def _pose_in_origin(self, x, y, yaw, x0, y0, yaw0):
        c = math.cos(yaw0)
        s = math.sin(yaw0)
        return x0 + c * x - s * y, y0 + s * x + c * y, yaw_wrap(yaw0 + yaw)

    def _map_pose(self, x, y, yaw):
        return self._pose_in_origin(
            x,
            y,
            yaw,
            float(self.get_parameter("initial_x").value),
            float(self.get_parameter("initial_y").value),
            float(self.get_parameter("initial_yaw").value),
        )

    def _truth_map_pose(self, x, y, yaw):
        return self._pose_in_origin(
            x,
            y,
            yaw,
            float(self.get_parameter("truth_initial_x").value),
            float(self.get_parameter("truth_initial_y").value),
            float(self.get_parameter("truth_initial_yaw").value),
        )

    def on_truth_odom(self, msg):
        _roll, _pitch, yaw = rpy_from_quat(msg.pose.pose.orientation)
        p = msg.pose.pose.position
        self.last_truth_pose = self._truth_map_pose(float(p.x), float(p.y), yaw)

    def _maybe_log_truth_offset(self, odom, yaw):
        if self.last_truth_pose is None:
            return

        p = odom.pose.pose.position
        nav_map_x, nav_map_y, nav_map_yaw = self._map_pose(float(p.x), float(p.y), yaw)
        truth_map_x, truth_map_y, truth_map_yaw = self.last_truth_pose
        distance = math.hypot(nav_map_x - truth_map_x, nav_map_y - truth_map_y)
        yaw_error = abs(yaw_wrap(nav_map_yaw - truth_map_yaw))
        warn_distance = float(self.get_parameter("truth_warn_distance").value)
        warn_yaw = float(self.get_parameter("truth_warn_yaw").value)
        if distance < warn_distance and yaw_error < warn_yaw:
            return

        now = time.monotonic()
        log_period = float(self.get_parameter("truth_log_period").value)
        if now - self.last_truth_log_wall_time < log_period:
            return
        self.last_truth_log_wall_time = now
        self.get_logger().warning(
            "FAST-LIO/Nav2 pose differs from Isaac truth: "
            "dist=%.2fm yaw=%.1fdeg nav=(%.2f, %.2f) truth=(%.2f, %.2f)"
            % (
                distance,
                math.degrees(yaw_error),
                nav_map_x,
                nav_map_y,
                truth_map_x,
                truth_map_y,
            )
        )

    def _validate(self, msg, roll, pitch, yaw):
        p = msg.pose.pose.position
        values = [p.x, p.y, p.z, roll, pitch, yaw]
        if not all(math.isfinite(float(value)) for value in values):
            return False, "non-finite pose"

        max_abs_z = float(self.get_parameter("max_abs_z").value)
        if max_abs_z >= 0.0 and abs(float(p.z)) > max_abs_z:
            return False, "z %.2f exceeds %.2f" % (float(p.z), max_abs_z)

        max_abs_roll_pitch = float(self.get_parameter("max_abs_roll_pitch").value)
        if max_abs_roll_pitch >= 0.0 and max(abs(roll), abs(pitch)) > max_abs_roll_pitch:
            return False, "roll/pitch %.2f %.2f exceed %.2f" % (
                roll,
                pitch,
                max_abs_roll_pitch,
            )

        raw_time = stamp_seconds(msg.header.stamp)
        if not math.isfinite(raw_time):
            return False, "non-finite timestamp"

        if self.last_valid_time is not None:
            dt = raw_time - self.last_valid_time
            if bool(self.get_parameter("reject_non_monotonic_timestamps").value) and dt <= 1.0e-6:
                return False, "non-monotonic timestamp %.6f <= %.6f" % (raw_time, self.last_valid_time)

            max_timestamp_gap = float(self.get_parameter("max_timestamp_gap").value)
            if max_timestamp_gap > 0.0 and dt > max_timestamp_gap:
                return False, "timestamp gap %.2fs exceeds %.2fs" % (dt, max_timestamp_gap)

            distance = math.hypot(float(p.x) - self.last_valid_raw_x, float(p.y) - self.last_valid_raw_y)
            yaw_delta = abs(yaw_wrap(yaw - self.last_valid_yaw))

            max_position_jump = float(self.get_parameter("max_position_jump").value)
            if max_position_jump > 0.0 and distance > max_position_jump:
                return False, "xy jump %.2fm exceeds %.2fm" % (distance, max_position_jump)

            max_yaw_jump = float(self.get_parameter("max_yaw_jump").value)
            if max_yaw_jump > 0.0 and yaw_delta > max_yaw_jump:
                return False, "yaw jump %.2frad exceeds %.2frad" % (yaw_delta, max_yaw_jump)

            if dt > 1.0e-6:
                max_linear_speed = float(self.get_parameter("max_linear_speed").value)
                if max_linear_speed > 0.0:
                    speed = distance / dt
                    if speed > max_linear_speed:
                        return False, "xy speed %.2f exceeds %.2f" % (speed, max_linear_speed)

                max_angular_speed = float(self.get_parameter("max_angular_speed").value)
                if max_angular_speed > 0.0:
                    angular_speed = yaw_delta / dt
                    if angular_speed > max_angular_speed:
                        return False, "yaw speed %.2f exceeds %.2f" % (angular_speed, max_angular_speed)

        if (
            bool(self.get_parameter("reject_out_of_map").value)
            and self.map_available
        ):
            margin = float(self.get_parameter("map_margin").value)
            map_x, map_y, _map_yaw = self._map_pose(float(p.x), float(p.y), yaw)
            if (
                map_x < self.map_min_x - margin
                or map_x > self.map_max_x + margin
                or map_y < self.map_min_y - margin
                or map_y > self.map_max_y + margin
            ):
                return False, "map pose %.2f %.2f out of bounds" % (map_x, map_y)

        return True, ""

    def _apply_derived_twist(self, odom, msg, yaw):
        if not bool(self.get_parameter("derive_twist").value):
            return
        if self.last_valid_time is None:
            return

        raw_time = stamp_seconds(msg.header.stamp)
        dt = raw_time - self.last_valid_time
        if dt <= 1.0e-3:
            return

        p = msg.pose.pose.position
        vx_odom = (float(p.x) - self.last_valid_raw_x) / dt
        vy_odom = (float(p.y) - self.last_valid_raw_y) / dt
        wz = yaw_wrap(yaw - self.last_valid_yaw) / dt

        c = math.cos(yaw)
        s = math.sin(yaw)
        odom.twist.twist.linear.x = c * vx_odom + s * vy_odom
        odom.twist.twist.linear.y = -s * vx_odom + c * vy_odom
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = wz

    def _filtered_odom(self, msg, yaw):
        odom = copy.deepcopy(msg)
        odom.header.stamp = self._stamp(msg)
        odom.header.frame_id = str(self.get_parameter("odom_frame").value) or msg.header.frame_id
        odom.child_frame_id = str(self.get_parameter("nav_base_frame").value)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat_from_yaw(yaw)
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        self._apply_derived_twist(odom, msg, yaw)
        return odom

    def _publish(self, odom):
        self.pub.publish(odom)
        if not bool(self.get_parameter("publish_tf").value):
            return
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def _publish_last(self, msg):
        if self.last_valid_odom is None:
            return
        odom = copy.deepcopy(self.last_valid_odom)
        odom.header.stamp = self._stamp(msg)
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0
        self._publish(odom)

    def _log_reject(self, reason):
        self.reject_count += 1
        now = time.monotonic()
        if now - self.last_reject_log_wall_time < 1.0:
            return
        self.last_reject_log_wall_time = now
        self.get_logger().warning(
            "Rejected FAST-LIO odom for Nav2 (%s); total rejects=%d"
            % (reason, self.reject_count)
        )

    def _level(self, msg):
        """P(camera_init->body) → E^-1·P·E：启动时的水平机体系下的 base 位姿。"""
        o = msg.pose.pose.orientation
        p = msg.pose.pose.position
        qp = (o.x, o.y, o.z, o.w)
        qe, te = self.ext_q, self.ext_t
        qe_inv = qconj(qe)
        q = qmul(qmul(qe_inv, qp), qe)
        rp_te = qrot(qp, te)
        t = qrot(qe_inv, (rp_te[0] + p.x - te[0], rp_te[1] + p.y - te[1], rp_te[2] + p.z - te[2]))
        out = copy.deepcopy(msg)
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = t
        (out.pose.pose.orientation.x, out.pose.pose.orientation.y,
         out.pose.pose.orientation.z, out.pose.pose.orientation.w) = q
        return out

    def _apply_offset(self, x, y, yaw):
        ox, oy, oth = self.offset
        c, s = math.cos(oth), math.sin(oth)
        return ox + c * x - s * y, oy + s * x + c * y, yaw_wrap(yaw + oth)

    def _absorb(self, x, y, yaw):
        """改 offset，让当前原始位姿映射到上一帧输出位姿（吞掉这一帧的跳变）。"""
        last = self.last_valid_odom.pose.pose
        lyaw = 2.0 * math.atan2(last.orientation.z, last.orientation.w)
        oth = yaw_wrap(lyaw - yaw)
        c, s = math.cos(oth), math.sin(oth)
        self.offset = (last.position.x - (c * x - s * y), last.position.y - (s * x + c * y), oth)

    def _remember_raw(self, msg, yaw):
        p = msg.pose.pose.position
        self.last_valid_raw_x = float(p.x)
        self.last_valid_raw_y = float(p.y)
        self.last_valid_yaw = yaw
        self.last_valid_time = stamp_seconds(msg.header.stamp)

    def _halt_or_hold(self, msg, reason):
        now = time.monotonic()
        if self.reject_since is None:
            self.reject_since = now
        hold = float(self.get_parameter("max_hold_on_reject_sec").value)
        if hold > 0.0 and now - self.reject_since > hold:
            if now - self.last_diverge_log_wall_time > 2.0:
                self.last_diverge_log_wall_time = now
                self.get_logger().error(
                    "FAST-LIO odom bad for %.1fs (%s): stop publishing odom/TF so Nav2 halts"
                    % (now - self.reject_since, reason)
                )
            return
        if bool(self.get_parameter("publish_last_on_reject").value):
            self._publish_last(msg)

    def on_odom(self, msg):
        if self.level_enabled:
            msg = self._level(msg)
        roll, pitch, yaw = rpy_from_quat(msg.pose.pose.orientation)
        valid, reason = self._validate(msg, roll, pitch, yaw)
        if not valid:
            self._log_reject(reason)
            absorbable = self.last_valid_odom is not None and reason.startswith(
                ("xy jump", "yaw jump", "xy speed", "yaw speed"))
            if not absorbable:
                self._halt_or_hold(msg, reason)
                return
            # 以当前帧为新基准（之后的帧只和它比），输出保持在上一帧
            p = msg.pose.pose.position
            self._absorb(float(p.x), float(p.y), yaw)
            self._remember_raw(msg, yaw)
            now = time.monotonic()
            self.absorb_times = [t for t in self.absorb_times if now - t < 2.0] + [now]
            if len(self.absorb_times) > int(self.get_parameter("max_absorb_per_2s").value):
                self._halt_or_hold(msg, "%d jumps in 2s, last: %s" % (len(self.absorb_times), reason))
                return
            self.reject_since = None
            self._publish_last(msg)
            return
        self.reject_since = None

        odom = self._filtered_odom(msg, yaw)
        p = msg.pose.pose.position
        x, y, oyaw = self._apply_offset(float(p.x), float(p.y), yaw)
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = quat_from_yaw(oyaw)
        self._publish(odom)
        self._maybe_log_truth_offset(odom, oyaw)

        self.last_valid_odom = copy.deepcopy(odom)
        self._remember_raw(msg, yaw)


def main(args=None):
    rclpy.init(args=args)
    node = FastlioNav2OdomFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
