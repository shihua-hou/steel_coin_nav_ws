"""Stage 2 point cloud projection for the FAST_LIO OS0 route."""

import math
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


class Stage2PointCloudToLaserScan(Node):
    def __init__(self):
        super().__init__("stage2_pointcloud_to_laserscan")

        self.declare_parameter("target_frame", "body")
        self.declare_parameter("transform_tolerance", 0.10)
        self.declare_parameter("use_tf", True)
        self.declare_parameter("stamp_mode", "cloud")
        # stamp_mode=tf：戳 = 最新 stamp_tf_frame→output_frame TF 的时间 - stamp_tf_lag。
        # 保证 Nav2(AMCL/costmap) 的 tf2 MessageFilter 收到时 TF 已可插值，不走
        # waitForTransform —— Humble tf2 在该路径与 TF 监听线程互锁（实测 Nav2 容器死锁）。
        self.declare_parameter("stamp_tf_frame", "odom")
        self.declare_parameter("stamp_tf_lag", 0.10)
        self.declare_parameter("min_height", 0.05)
        self.declare_parameter("max_height", 0.85)
        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("angle_increment", math.radians(0.5))
        self.declare_parameter("scan_time", 0.10)
        self.declare_parameter("range_min", 0.30)
        self.declare_parameter("range_max", 12.0)
        self.declare_parameter("use_inf", True)
        self.declare_parameter("inf_epsilon", 1.0)
        self.declare_parameter("self_filter_enabled", True)
        self.declare_parameter("self_filter_min_x", -0.45)
        self.declare_parameter("self_filter_max_x", 0.45)
        self.declare_parameter("self_filter_min_y", -0.32)
        self.declare_parameter("self_filter_max_y", 0.32)
        # 孤立束滤波：±window 内几乎没有相近距离的邻居 → 视为杂点丢掉
        self.declare_parameter("isolate_filter_enabled", True)
        self.declare_parameter("isolate_beam_window", 2)
        self.declare_parameter("isolate_min_neighbors", 1)
        self.declare_parameter("isolate_range_ratio", 0.25)
        self.declare_parameter("isolate_range_abs", 0.35)
        # 钢镚：先 TF 到 body（FAST-LIO 树），再左乘 body→output 外参，
        # 避免与 odom_filter 的 odom→base_footprint 抢父帧导致乱飘。
        self.declare_parameter("output_frame", "")
        self.declare_parameter("body_extrinsic_x", 0.0)
        self.declare_parameter("body_extrinsic_y", 0.0)
        self.declare_parameter("body_extrinsic_z", 0.0)
        self.declare_parameter("body_extrinsic_qx", 0.0)
        self.declare_parameter("body_extrinsic_qy", 0.0)
        self.declare_parameter("body_extrinsic_qz", 0.0)
        self.declare_parameter("body_extrinsic_qw", 1.0)
        # 近场补点：FAST-LIO blind=0.5 会把离雷达 0.5 m 内的点全丢，狗头前左方站人/贴墙都看不见。
        # 不改 FAST-LIO（近点会让 LIO 抖）；C++ 节点 near_field_cloud 从原始 /livox/lidar
        # 取 0.25~0.70 m 的点、已平移到 body 系，发小点云 /near_cloud，这里并入 /scan。
        # （Python 直接订 CustomMsg 反序列化 2 万点/帧会把 stage2 拖满单核，2026-10-01 已弃用）
        self.declare_parameter("near_topic", "/near_cloud")   # "" = 关
        self.declare_parameter("near_max_age", 0.30)
        # 近场点落在狗自身轮廓里（output_frame 下的盒子）一律丢：轮廓内不可能有外部障碍，
        # 只是兜底（雷达装在狗头上方，腿不会进视野；实测自身点只有雷达 0.2 m 内的外壳/头顶，
        # 已被 near_field_cloud min_range 0.25 去掉）。盒子 = 实测 0.635×0.36 m 机身 + 余量（2026-10-02）
        self.declare_parameter("near_self_box", [-0.34, 0.34, -0.20, 0.20])  # xmin,xmax,ymin,ymax
        self._near_xyz = None
        self._near_wall_time = 0.0

        self._warned_frame_mismatch = False
        self._warned_bad_stamp_mode = False
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        # BEST_EFFORT：与 Nav2 costmap / AMCL 的 sensor QoS 匹配（RELIABLE 会对不上，代价图一直无激光）
        self._pub = self.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
        self._sub = self.create_subscription(
            PointCloud2, "cloud_in", self._on_cloud, qos_profile_sensor_data)
        near_topic = str(self.get_parameter("near_topic").value).strip()
        if near_topic:
            self._near_sub = self.create_subscription(
                PointCloud2, near_topic, self._on_near, qos_profile_sensor_data)
            self.get_logger().info("Near-field merge from %s enabled" % near_topic)

    def _on_near(self, msg: PointCloud2) -> None:
        pts = point_cloud2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
        self._near_wall_time = time.monotonic()
        if not len(pts):
            self._near_xyz = None
            return
        xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float32, copy=False)
        output_frame = str(self.get_parameter("output_frame").value).strip()
        target_frame = str(self.get_parameter("target_frame").value)
        if output_frame and output_frame != target_frame:
            bx, by, _bz = self._apply_body_extrinsic(xyz[:, 0], xyz[:, 1], xyz[:, 2])
        else:
            bx, by = xyz[:, 0], xyz[:, 1]
        x0, x1, y0, y1 = [float(v) for v in self.get_parameter("near_self_box").value]
        inside = (bx >= x0) & (bx <= x1) & (by >= y0) & (by <= y1)
        xyz = xyz[~inside]
        self._near_xyz = xyz if xyz.shape[0] else None

    def _scan_stamp(self, msg: PointCloud2):
        stamp_mode = str(self.get_parameter("stamp_mode").value).lower()
        if stamp_mode == "cloud":
            return msg.header.stamp
        if stamp_mode == "now":
            return self.get_clock().now().to_msg()
        if stamp_mode == "tf":
            lag = Duration(seconds=max(float(self.get_parameter("stamp_tf_lag").value), 0.0))
            child = str(self.get_parameter("output_frame").value).strip() or "base_footprint"
            try:
                tf = self._tf_buffer.lookup_transform(
                    str(self.get_parameter("stamp_tf_frame").value), child, Time())
                return (Time.from_msg(tf.header.stamp) - lag).to_msg()
            except TransformException:
                return (self.get_clock().now() - lag).to_msg()
        if not self._warned_bad_stamp_mode:
            self.get_logger().warning(
                "Unknown stamp_mode '%s'; falling back to current ROS time" % stamp_mode
            )
            self._warned_bad_stamp_mode = True
        return self.get_clock().now().to_msg()

    def _transform_xyz(self, points, transform):
        x = points["x"].astype(np.float32, copy=False)
        y = points["y"].astype(np.float32, copy=False)
        z = points["z"].astype(np.float32, copy=False)

        q = transform.transform.rotation
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z

        qx = q.x
        qy = q.y
        qz = q.z
        qw = q.w
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 0.0:
            raise ValueError("transform quaternion has zero norm")
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        out_x = (1.0 - 2.0 * (yy + zz)) * x + 2.0 * (xy - wz) * y + 2.0 * (xz + wy) * z + tx
        out_y = 2.0 * (xy + wz) * x + (1.0 - 2.0 * (xx + zz)) * y + 2.0 * (yz - wx) * z + ty
        out_z = 2.0 * (xz - wy) * x + 2.0 * (yz + wx) * y + (1.0 - 2.0 * (xx + yy)) * z + tz
        return out_x.astype(np.float32), out_y.astype(np.float32), out_z.astype(np.float32)

    def _apply_body_extrinsic(self, x, y, z):
        """Points are in body; (t,q) is pose of output_frame in body (ROS static TF).

        p_body = R * p_out + t  =>  p_out = R^T * (p_body - t)
        """
        tx = float(self.get_parameter("body_extrinsic_x").value)
        ty = float(self.get_parameter("body_extrinsic_y").value)
        tz = float(self.get_parameter("body_extrinsic_z").value)
        qx = float(self.get_parameter("body_extrinsic_qx").value)
        qy = float(self.get_parameter("body_extrinsic_qy").value)
        qz = float(self.get_parameter("body_extrinsic_qz").value)
        qw = float(self.get_parameter("body_extrinsic_qw").value)
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        x = x.astype(np.float32, copy=False)
        y = y.astype(np.float32, copy=False)
        z = z.astype(np.float32, copy=False)
        if norm <= 1e-9:
            return x - np.float32(tx), y - np.float32(ty), z - np.float32(tz)
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
        # R^T via conjugate quaternion (-qx,-qy,-qz,qw)
        qx, qy, qz = -qx, -qy, -qz
        xx, yy, zz = qx * qx, qy * qy, qz * qz
        xy, xz, yz = qx * qy, qx * qz, qy * qz
        wx, wy, wz = qw * qx, qw * qy, qw * qz
        dx = x - np.float32(tx)
        dy = y - np.float32(ty)
        dz = z - np.float32(tz)
        out_x = (1.0 - 2.0 * (yy + zz)) * dx + 2.0 * (xy - wz) * dy + 2.0 * (xz + wy) * dz
        out_y = 2.0 * (xy + wz) * dx + (1.0 - 2.0 * (xx + zz)) * dy + 2.0 * (yz - wx) * dz
        out_z = 2.0 * (xz - wy) * dx + 2.0 * (yz + wx) * dy + (1.0 - 2.0 * (xx + yy)) * dz
        return out_x.astype(np.float32), out_y.astype(np.float32), out_z.astype(np.float32)

    def _xyz_in_target_frame(self, msg: PointCloud2, target_frame: str):
        points = point_cloud2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)
        source_frame = msg.header.frame_id
        if not target_frame or not source_frame or source_frame == target_frame:
            return msg.header, points["x"], points["y"], points["z"]

        if not bool(self.get_parameter("use_tf").value):
            if not self._warned_frame_mismatch:
                self.get_logger().warning(
                    "Publishing scan in target_frame without TF transform; "
                    "use a cloud already expressed in that frame."
                )
                self._warned_frame_mismatch = True
            return msg.header, points["x"], points["y"], points["z"]

        timeout = Duration(seconds=max(float(self.get_parameter("transform_tolerance").value), 0.0))
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=timeout,
            )
        except TransformException as exc:
            self.get_logger().warning(
                "Skipping cloud: no TF %s <- %s: %s"
                % (target_frame, source_frame, exc),
                throttle_duration_sec=1.0,
            )
            return None

        return transform.header, *self._transform_xyz(points, transform)

    def _on_cloud(self, msg: PointCloud2) -> None:
        target_frame = str(self.get_parameter("target_frame").value)
        min_height = float(self.get_parameter("min_height").value)
        max_height = float(self.get_parameter("max_height").value)
        angle_min = float(self.get_parameter("angle_min").value)
        angle_max = float(self.get_parameter("angle_max").value)
        angle_increment = float(self.get_parameter("angle_increment").value)
        scan_time = float(self.get_parameter("scan_time").value)
        range_min = float(self.get_parameter("range_min").value)
        range_max = float(self.get_parameter("range_max").value)
        use_inf = bool(self.get_parameter("use_inf").value)
        inf_epsilon = float(self.get_parameter("inf_epsilon").value)
        self_filter_enabled = bool(self.get_parameter("self_filter_enabled").value)
        self_filter_min_x = float(self.get_parameter("self_filter_min_x").value)
        self_filter_max_x = float(self.get_parameter("self_filter_max_x").value)
        self_filter_min_y = float(self.get_parameter("self_filter_min_y").value)
        self_filter_max_y = float(self.get_parameter("self_filter_max_y").value)

        if angle_increment <= 0.0 or angle_max <= angle_min:
            self.get_logger().error("Invalid scan angular bounds")
            return

        beam_count = int(math.floor((angle_max - angle_min) / angle_increment)) + 1
        if beam_count <= 0:
            return

        transformed = self._xyz_in_target_frame(msg, target_frame)
        if transformed is None:
            return
        scan_header, x, y, z = transformed
        near = self._near_xyz
        if (near is not None and near.shape[0]
                and time.monotonic() - self._near_wall_time
                <= float(self.get_parameter("near_max_age").value)):
            # 近场点已在 body 系（与 target_frame=body 相同）
            x = np.concatenate([np.asarray(x, dtype=np.float32), near[:, 0]])
            y = np.concatenate([np.asarray(y, dtype=np.float32), near[:, 1]])
            z = np.concatenate([np.asarray(z, dtype=np.float32), near[:, 2]])
        output_frame = str(self.get_parameter("output_frame").value).strip()
        if output_frame and output_frame != target_frame:
            x, y, z = self._apply_body_extrinsic(x, y, z)

        fill_value = math.inf if use_inf else range_max + inf_epsilon
        scan_ranges = np.full(beam_count, fill_value, dtype=np.float32)

        if x.size:
            ranges = np.hypot(x, y)
            angles = np.arctan2(y, x)

            valid = (
                (z >= min_height)
                & (z <= max_height)
                & (ranges >= range_min)
                & (ranges <= range_max)
                & (angles >= angle_min)
                & (angles <= angle_max)
            )
            if self_filter_enabled:
                inside_self = (
                    (x >= self_filter_min_x)
                    & (x <= self_filter_max_x)
                    & (y >= self_filter_min_y)
                    & (y <= self_filter_max_y)
                )
                valid = valid & ~inside_self
            if np.any(valid):
                bins = np.floor((angles[valid] - angle_min) / angle_increment).astype(np.int32)
                bins = np.clip(bins, 0, beam_count - 1)
                np.minimum.at(scan_ranges, bins, ranges[valid])

        if bool(self.get_parameter("isolate_filter_enabled").value):
            win = max(1, int(self.get_parameter("isolate_beam_window").value))
            need = max(1, int(self.get_parameter("isolate_min_neighbors").value))
            ratio = float(self.get_parameter("isolate_range_ratio").value)
            abs_tol = float(self.get_parameter("isolate_range_abs").value)
            cleaned = scan_ranges.copy()
            for i in range(beam_count):
                r = float(scan_ranges[i])
                if not math.isfinite(r) or r >= range_max:
                    continue
                tol = max(abs_tol, r * ratio)
                neigh = 0
                for j in range(i - win, i + win + 1):
                    if j == i or j < 0 or j >= beam_count:
                        continue
                    rj = float(scan_ranges[j])
                    if math.isfinite(rj) and abs(rj - r) <= tol:
                        neigh += 1
                if neigh < need:
                    cleaned[i] = fill_value
            scan_ranges = cleaned

        scan = LaserScan()
        scan.header = scan_header
        scan.header.stamp = self._scan_stamp(msg)
        scan.header.frame_id = output_frame or target_frame or scan.header.frame_id
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_increment
        scan.time_increment = 0.0
        scan.scan_time = scan_time
        scan.range_min = range_min
        scan.range_max = range_max
        scan.ranges = scan_ranges.tolist()
        self._pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = Stage2PointCloudToLaserScan()
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
