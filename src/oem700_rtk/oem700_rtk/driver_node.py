#!/usr/bin/env python3
"""Read OEM700 NMEA and, when asked, write the Sixents NTRIP account onto the board.

Air700E already runs the NTRIP client over its own 4G link. This node only
talks to the USB/6PIN UART (115200). It does not open a second NTRIP session
and it does not inject RTCM.
"""

import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Int32

try:
    from nmea_msgs.msg import Sentence
except ImportError:
    Sentence = None

try:
    import yaml
except ImportError:
    yaml = None

from oem700_rtk.nmea_codec import (
    checksum_ok,
    format_sentence,
    parse_gga,
    parse_heading,
    parse_rmc,
    split_sentence,
)
from oem700_rtk.serial_port import (
    candidate_ports,
    open_serial,
    read_some,
    write_all,
)


def _quat_from_yaw(yaw):
    quat = Quaternion()
    quat.z = math.sin(yaw * 0.5)
    quat.w = math.cos(yaw * 0.5)
    return quat


def _nmea_line_count(blob):
    """How many lines are actual NMEA, not a log line that happens to quote one."""
    text = blob.decode("ascii", "ignore")
    return sum(
        1 for line in text.splitlines()
        if line.startswith("$GN") or line.startswith("$GP") or line.startswith("$GL"))


def load_cors(path):
    if not path or not os.path.isfile(path) or yaml is None:
        return None
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    need = ("host", "port", "mount_point", "username", "password")
    if any(not data.get(key) for key in need):
        return None
    data["port"] = int(data["port"])
    data["upload_gga"] = int(data.get("upload_gga", 1))
    return data


class Oem700Driver(Node):
    def __init__(self):
        super().__init__("oem700_driver")
        self.declare_parameter("cors_file", "/home/linaro/steel_coin_nav_ws/config/oem700_cors.yaml")
        self.declare_parameter("configure", True)
        self.declare_parameter("baud", 115200)

        self._fix_pub = self.create_publisher(NavSatFix, "/gnss/fix", qos_profile_sensor_data)
        self._quality_pub = self.create_publisher(Int32, "/gnss/quality", 10)
        self._fixed_pub = self.create_publisher(Bool, "/gnss/rtk_fixed", 10)
        self._heading_pub = self.create_publisher(PoseWithCovarianceStamped, "/gnss/heading", 10)
        self._nmea_pub = None
        if Sentence is not None:
            self._nmea_pub = self.create_publisher(Sentence, "/gnss/nmea", 10)
        else:
            self.get_logger().warning("nmea_msgs not installed; /gnss/nmea will not be published")

        self._rmc_mode = ""
        self._quality = 0
        self._stop = False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def destroy_node(self):
        self._stop = True
        return super().destroy_node()

    def _loop(self):
        baud = int(self.get_parameter("baud").value)
        while rclpy.ok() and not self._stop:
            fd = self._claim_port(baud)
            if fd is None:
                self.get_logger().warning(
                    "OEM700 NMEA port not found (ttyACM/ttyUSB or /dev/oem700_nmea)",
                    throttle_duration_sec=10.0)
                time.sleep(2.0)
                continue
            try:
                reopen = False
                if bool(self.get_parameter("configure").value):
                    reopen = self._configure(fd)
                if reopen:
                    # saveconfig 会让 USB 串口重新枚举，旧 fd 变成 deleted。
                    self.get_logger().info("reopen serial after saveconfig")
                    time.sleep(2.0)
                    continue
                self._read_forever(fd)
            except Exception as exc:
                self.get_logger().warning("OEM700 serial closed: %s" % exc)
            finally:
                try:
                    os.close(fd)
                except Exception:
                    pass
            time.sleep(1.0)

    def _claim_port(self, baud):
        # 四个口里：一个持续刷 NMEA，一个是 4G 日志（偶尔夹一条 GGA），一个是 RTCM。
        # 按 1 秒内的 NMEA 行数选最多的那个，单条不够。
        best_path, best_score = None, 0
        for path in candidate_ports():
            fd = None
            try:
                fd = open_serial(path, baud)
                write_all(fd, b"\r\n")
                blob = b""
                deadline = time.monotonic() + 1.0
                while time.monotonic() < deadline:
                    blob += read_some(fd, 0.2)
                score = _nmea_line_count(blob)
                # if06 是定位口，有一句 GGA 就用它，避免被日志口的偶发 GGA 抢走。
                if path.endswith("-if06") and score >= 1:
                    best_path, best_score = path, score
                    break
                if score > best_score:
                    best_path, best_score = path, score
            except Exception:
                pass
            finally:
                if fd is not None:
                    try:
                        os.close(fd)
                    except Exception:
                        pass
        if best_path is None or best_score < 1:
            return None
        try:
            fd = open_serial(best_path, baud)
        except Exception:
            return None
        self.get_logger().info("OEM700 NMEA on %s (%d sentences/s)" % (best_path, best_score))
        return fd

    def _collect(self, fd, seconds):
        blob = b""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            blob += read_some(fd, 0.2)
        return blob.decode("ascii", "ignore")

    def _configure(self, fd):
        cors = load_cors(str(self.get_parameter("cors_file").value))
        write_all(fd, format_sentence("gnp,") + "\r\n")
        text = self._collect(fd, 1.5)
        current = self._parse_gnp(text)
        wrote = False
        if cors is None:
            self.get_logger().info("no CORS yaml; skip $snp (board keeps its saved account)")
        elif current and self._cors_matches(current, cors):
            # 账号已在板子上。再发 gpgga/sne 会让 UM982 停几秒 NMEA，首页就闪成无信号。
            self.get_logger().info("OEM700 NTRIP already matches %s:%s/%s" % (
                cors["host"], cors["port"], cors["mount_point"]))
        else:
            body = "snp,%s,%s,%s,%s,%s,%s" % (
                cors["host"], cors["port"], cors["mount_point"],
                cors["username"], cors["password"], cors["upload_gga"])
            write_all(fd, format_sentence(body) + "\r\n")
            reply = self._collect(fd, 1.5)
            if "$snp,ok" in reply.replace(" ", ""):
                self.get_logger().info("OEM700 NTRIP written ($snp,ok)")
                wrote = True
            else:
                self.get_logger().warning("OEM700 $snp got no ok; keep reading NMEA anyway")
            write_all(fd, format_sentence("sne,1,") + "\r\n")
            # 手册里 1 = 1 Hz。只在刚写入账号时配置一次。
            for cmd in ("gpgga 1", "gprmc 1", "gphdt 1"):
                write_all(fd, cmd + "\r\n")
                time.sleep(0.05)
        if wrote:
            write_all(fd, "saveconfig\r\n")
            self.get_logger().info("UM982 saveconfig after NTRIP change")
            return True
        return False

    @staticmethod
    def _parse_gnp(text):
        for raw in text.splitlines():
            body, _cs = split_sentence(raw)
            if not body.startswith("gnp,"):
                continue
            parts = body.split(",")
            # gnp,host,port,mount,user,up_gga
            if len(parts) < 6:
                continue
            try:
                port = int(parts[2])
                up = int(parts[5])
            except ValueError:
                continue
            return {
                "host": parts[1],
                "port": port,
                "mount_point": parts[3],
                "username": parts[4],
                "upload_gga": up,
            }
        return None

    @staticmethod
    def _cors_matches(current, cors):
        return (
            current["host"] == cors["host"]
            and int(current["port"]) == int(cors["port"])
            and current["mount_point"] == cors["mount_point"]
            and current["username"] == cors["username"]
            and int(current["upload_gga"]) == int(cors["upload_gga"])
        )

    def _read_forever(self, fd):
        buf = b""
        silent_since = time.monotonic()
        while rclpy.ok() and not self._stop:
            chunk = read_some(fd, 0.5)
            if not chunk:
                # USB 重枚举后旧 fd 还在，但设备节点已经没了，必须重开。
                if time.monotonic() - silent_since > 3.0:
                    raise RuntimeError("no NMEA for 3s")
                continue
            silent_since = time.monotonic()
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "ignore").strip()
                if line.startswith("$"):
                    self._on_line(line)
            if len(buf) > 8192:
                buf = buf[-1024:]

    def _on_line(self, line):
        if not checksum_ok(line):
            return
        if self._nmea_pub is not None:
            msg = Sentence()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gnss"
            msg.sentence = line
            self._nmea_pub.publish(msg)
        gga = parse_gga(line)
        if gga is not None:
            self._quality = int(gga["quality"])
            self._publish_fix(gga)
            return
        rmc = parse_rmc(line)
        if rmc is not None:
            self._rmc_mode = rmc["mode"] or ""
            self._publish_fixed()
            return
        heading = parse_heading(line)
        if heading is not None:
            self._publish_heading(heading)

    def _fixed_now(self):
        return self._quality == 4 and self._rmc_mode == "R"

    def _publish_fixed(self):
        msg = Bool()
        msg.data = self._fixed_now()
        self._fixed_pub.publish(msg)
        q = Int32()
        q.data = int(self._quality)
        self._quality_pub.publish(q)

    def _publish_fix(self, gga):
        self._publish_fixed()
        if gga["lat"] is None or gga["lon"] is None:
            return
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gnss"
        quality = int(gga["quality"])
        if quality <= 0:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
        elif quality == 1:
            msg.status.status = NavSatStatus.STATUS_FIX
        elif quality == 4:
            msg.status.status = NavSatStatus.STATUS_GBAS_FIX
        else:
            # Float (5) and DGPS (2) are not a fixed RTK solution.
            msg.status.status = NavSatStatus.STATUS_SBAS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = float(gga["lat"])
        msg.longitude = float(gga["lon"])
        msg.altitude = float(gga["alt"] or 0.0)
        hdop = gga["hdop"] if gga["hdop"] is not None else 99.0
        var = max(hdop, 0.01) ** 2
        if quality == 4:
            var = min(var, 0.01)
        msg.position_covariance[0] = var
        msg.position_covariance[4] = var
        msg.position_covariance[8] = var * 4.0
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._fix_pub.publish(msg)

    def _publish_heading(self, heading):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "enu"
        msg.pose.pose.orientation = _quat_from_yaw(float(heading["yaw"]))
        # covariance[0] < 0 means the antenna heading is not usable.
        msg.pose.covariance[0] = 0.02 if heading["valid"] else -1.0
        self._heading_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Oem700Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
