#!/usr/bin/env python3
"""Project a WGS84 RTK fix into the occupancy-map frame using a per-map datum.

Publishes /rtk/map_pose only while GGA quality is 4 and RMC mode is R
(/gnss/rtk_fixed). Yaw is marked invalid (covariance[35] < 0) when the
dual-antenna heading is missing, so the arbiter keeps AMCL's heading.
"""

import math
import os

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool

from oem700_rtk.fusion_launch import datum_path
from oem700_rtk.geodesy import base_from_antenna, enu_to_map, geodetic_to_enu
from oem700_rtk.nmea_codec import yaw_wrap


def _yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _quat(yaw):
    quat = Quaternion()
    quat.z = math.sin(yaw * 0.5)
    quat.w = math.cos(yaw * 0.5)
    return quat


def load_datum(path):
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    antenna = data.get("antenna_xyz") or [0.0, 0.0, 0.0]
    return {
        "lat0": float(data["lat0"]),
        "lon0": float(data["lon0"]),
        "alt0": float(data.get("alt0", 0.0)),
        "yaw_enu": float(data.get("yaw_enu", 0.0)),
        "ax": float(antenna[0]),
        "ay": float(antenna[1]) if len(antenna) > 1 else 0.0,
        "heading_offset": float(data.get("heading_offset", 0.0)),
    }


class RtkToMap(Node):
    def __init__(self):
        super().__init__("rtk_to_map")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("max_age", 0.5)
        map_yaml = str(self.get_parameter("map_yaml").value)
        path = datum_path(map_yaml) if map_yaml else ""
        self._datum = None
        if path and os.path.isfile(path):
            self._datum = load_datum(path)
            self.get_logger().info("RTK datum %s" % path)
        else:
            self.get_logger().warning(
                "no datum for %s; /rtk/map_pose stays silent (AMCL keeps map→odom)"
                % (map_yaml or "(unset)"))

        self._fixed = False
        self._fixed_stamp = 0.0
        self._fix = None
        self._heading_yaw = None
        self._heading_ok = False
        self._heading_stamp = 0.0
        self._heading_hist = []
        self._pub = self.create_publisher(PoseWithCovarianceStamped, "/rtk/map_pose", 10)
        self.create_subscription(NavSatFix, "/gnss/fix", self._on_fix, qos_profile_sensor_data)
        self.create_subscription(Bool, "/gnss/rtk_fixed", self._on_fixed, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/gnss/heading", self._on_heading, 10)
        self.create_timer(0.1, self._tick)

    def _on_fixed(self, msg):
        self._fixed = bool(msg.data)
        self._fixed_stamp = time_now(self)

    def _on_fix(self, msg):
        if msg.status.status < 0:
            return
        self._fix = msg

    def _on_heading(self, msg):
        now = time_now(self)
        self._heading_stamp = now
        reported = float(msg.pose.covariance[0]) >= 0.0
        yaw = _yaw_of(msg.pose.pose.orientation) if reported else None
        # Heading on this board keeps turning while the dog is standing still,
        # and the steps between messages are small, so a one-step check never
        # trips. Require the last second to stay inside 15 degrees.
        if reported and yaw is not None:
            self._heading_hist.append((now, yaw))
        self._heading_hist = [(t, y) for t, y in self._heading_hist if now - t <= 1.2]
        ok = False
        if reported and yaw is not None and len(self._heading_hist) >= 3:
            span = max(abs(yaw_wrap(y - yaw)) for _t, y in self._heading_hist)
            covered = now - self._heading_hist[0][0]
            # Standing still, this heading was turning ~20 deg/s. A real
            # dual-antenna fix stays inside a few degrees for a full second.
            ok = covered >= 0.8 and span < math.radians(5.0)
        self._heading_ok = ok
        self._heading_yaw = yaw if ok else None

    def _tick(self):
        if self._datum is None or self._fix is None or not self._fixed:
            return
        now = time_now(self)
        if now - self._fixed_stamp > float(self.get_parameter("max_age").value):
            return
        fix_age = now - (self._fix.header.stamp.sec + self._fix.header.stamp.nanosec * 1e-9)
        if fix_age > 1.0:
            return
        datum = self._datum
        east, north, _up = geodetic_to_enu(
            self._fix.latitude, self._fix.longitude, self._fix.altitude,
            datum["lat0"], datum["lon0"], datum["alt0"])
        ant_x, ant_y = enu_to_map(east, north, datum["yaw_enu"])
        yaw_ok = (
            self._heading_ok
            and self._heading_yaw is not None
            and now - self._heading_stamp < 1.0
        )
        if yaw_ok:
            yaw = yaw_wrap(self._heading_yaw - datum["yaw_enu"] + datum["heading_offset"])
            bx, by = base_from_antenna(ant_x, ant_y, yaw, datum["ax"], datum["ay"])
        else:
            # Heading unknown: publish the antenna point. A non-zero lever arm
            # would need a yaw to subtract, and yaw 0 would shift the fix.
            yaw = 0.0
            bx, by = ant_x, ant_y
        out = PoseWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        out.pose.pose.position.x = bx
        out.pose.pose.position.y = by
        out.pose.pose.orientation = _quat(yaw)
        out.pose.covariance[0] = 0.0004
        out.pose.covariance[7] = 0.0004
        # Negative yaw variance tells the arbiter to keep the laser heading.
        out.pose.covariance[35] = 0.01 if yaw_ok else -1.0
        self._pub.publish(out)


def time_now(node):
    return node.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = RtkToMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
