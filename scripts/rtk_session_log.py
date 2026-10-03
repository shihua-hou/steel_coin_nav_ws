#!/usr/bin/env python3
"""Append 1 Hz JSON lines of GNSS and localization for a later RTK check.

Does not record lidar. Safe to leave running while mapping and navigating
from the web UI.
"""

import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Int32, String
from tf2_ros import Buffer, TransformListener

LOG_PATH = os.environ.get(
    "RTK_SESSION_LOG", "/home/linaro/steel_coin_nav_ws/logs/rtk_session.jsonl")


def yaw_of(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pose_xyyaw(msg):
    p = msg.pose.pose
    yaw_ok = float(msg.pose.covariance[35]) >= 0.0
    return {
        "x": round(p.position.x, 3),
        "y": round(p.position.y, 3),
        "yaw_deg": round(math.degrees(yaw_of(p.orientation)), 2),
        "yaw_ok": yaw_ok,
    }


class SessionLog(Node):
    def __init__(self):
        super().__init__("rtk_session_log")
        os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
        self._fp = open(LOG_PATH, "a", encoding="utf-8")
        self._fix = None
        self._fix_t = 0.0
        self._fixed = None
        self._quality = None
        self._heading = None
        self._heading_t = 0.0
        self._source = None
        self._amcl = None
        self._amcl_t = 0.0
        self._odom = None
        self._odom_t = 0.0
        self._rtk_map = None
        self._rtk_map_t = 0.0
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(NavSatFix, "/gnss/fix", self._on_fix, qos_profile_sensor_data)
        self.create_subscription(Bool, "/gnss/rtk_fixed", self._on_fixed, 10)
        self.create_subscription(Int32, "/gnss/quality", self._on_quality, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/gnss/heading", self._on_heading, 10)
        self.create_subscription(String, "/gnss/pose_source", self._on_source, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10)
        self.create_subscription(Odometry, "/odom_nav", self._on_odom, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/rtk/map_pose", self._on_rtk_map, 10)
        self._write({"event": "start", "path": LOG_PATH})
        self.create_timer(1.0, self._tick)

    def _now(self):
        return time.time()

    def _write(self, row):
        row["t"] = round(self._now(), 3)
        self._fp.write(json.dumps(row, ensure_ascii=False) + "\n")
        self._fp.flush()

    def _on_fix(self, msg):
        self._fix = msg
        self._fix_t = self._now()

    def _on_fixed(self, msg):
        value = bool(msg.data)
        if value != self._fixed:
            self._write({"event": "rtk_fixed", "value": value})
        self._fixed = value

    def _on_quality(self, msg):
        self._quality = int(msg.data)

    def _on_heading(self, msg):
        self._heading = pose_xyyaw(msg)
        self._heading_t = self._now()

    def _on_source(self, msg):
        text = msg.data or ""
        if text != self._source:
            self._write({"event": "pose_source", "value": text})
        self._source = text

    def _on_amcl(self, msg):
        self._amcl = pose_xyyaw(msg)
        self._amcl_t = self._now()

    def _on_odom(self, msg):
        p = msg.pose.pose
        self._odom = {
            "x": round(p.position.x, 3),
            "y": round(p.position.y, 3),
            "yaw_deg": round(math.degrees(yaw_of(p.orientation)), 2),
        }
        self._odom_t = self._now()

    def _on_rtk_map(self, msg):
        self._rtk_map = pose_xyyaw(msg)
        self._rtk_map_t = self._now()

    def _age(self, stamp):
        if not stamp:
            return None
        return round(self._now() - stamp, 2)

    def _map_base(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        return {
            "x": round(t.x, 3),
            "y": round(t.y, 3),
            "yaw_deg": round(math.degrees(yaw_of(tf.transform.rotation)), 2),
        }

    def _tick(self):
        fix = None
        if self._fix is not None:
            fix = {
                "lat": round(self._fix.latitude, 8),
                "lon": round(self._fix.longitude, 8),
                "alt": round(self._fix.altitude, 3),
                "status": int(self._fix.status.status),
                "age": self._age(self._fix_t),
            }
        row = {
            "fix": fix,
            "rtk_fixed": self._fixed,
            "quality": self._quality,
            "heading": self._heading if self._age(self._heading_t) is not None and self._age(self._heading_t) < 3 else None,
            "pose_source": self._source,
            "amcl": self._amcl if (self._age(self._amcl_t) or 99) < 3 else None,
            "odom": self._odom if (self._age(self._odom_t) or 99) < 3 else None,
            "map_base": self._map_base(),
            "rtk_map": self._rtk_map if (self._age(self._rtk_map_t) or 99) < 3 else None,
        }
        self._write(row)


def main():
    rclpy.init()
    node = SessionLog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
