#!/usr/bin/env python3
"""Accumulate /lio/cloud_world into a live OccupancyGrid on /map_building.

Used by the web UI during Super-LIO mapping so the 2D preview is not stuck
waiting for Nav2's /map (which only exists after a PGM is loaded).
"""
import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from std_msgs.msg import Header


def _cloud_xyz(msg: PointCloud2):
    """Yield (x, y, z) from PointCloud2 (xyz float32)."""
    field_map = {f.name: f for f in msg.fields}
    if not all(k in field_map for k in ('x', 'y', 'z')):
        return
    off_x, off_y, off_z = field_map['x'].offset, field_map['y'].offset, field_map['z'].offset
    step = msg.point_step
    data = bytes(msg.data)
    n = msg.width * msg.height
    for i in range(n):
        base = i * step
        if base + off_z + 4 > len(data):
            break
        x = struct.unpack_from('<f', data, base + off_x)[0]
        y = struct.unpack_from('<f', data, base + off_y)[0]
        z = struct.unpack_from('<f', data, base + off_z)[0]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            yield x, y, z


class MapBuildingNode(Node):
    def __init__(self):
        super().__init__('map_building_node')
        self.resolution = self.declare_parameter('resolution', 0.05).value
        self.z_min = self.declare_parameter('z_min', -0.3).value
        self.z_max = self.declare_parameter('z_max', 1.2).value
        self.occ_min = self.declare_parameter('occ_min', 0.12).value
        self.ground_z = self.declare_parameter('ground_z', 0.05).value
        self.publish_rate = self.declare_parameter('publish_rate', 2.0).value
        self.max_extent = self.declare_parameter('max_extent', 40.0).value
        self.frame_id = self.declare_parameter('frame_id', 'world').value

        self._cells = {}
        self._dirty = False
        self._cloud_frames = 0
        self._point_total = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=2)
        self.sub = self.create_subscription(
            PointCloud2, '/lio/cloud_world', self.cb_cloud, qos)
        self.pub = self.create_publisher(OccupancyGrid, '/map_building', 1)
        self.create_timer(1.0 / max(0.5, self.publish_rate), self.cb_publish)
        self.get_logger().info(
            f'map_building_node ready: res={self.resolution} z=[{self.z_min},{self.z_max}]')

    def cb_cloud(self, msg: PointCloud2):
        res = self.resolution
        added = 0
        # Decimate dense Livox clouds for CPU budget
        for i, (x, y, z) in enumerate(_cloud_xyz(msg)):
            if i % 3 != 0:
                continue
            if z < self.z_min or z > self.z_max:
                continue
            if abs(x) > self.max_extent or abs(y) > self.max_extent:
                continue
            ix = int(math.floor(x / res))
            iy = int(math.floor(y / res))
            key = (ix, iy)
            cell = self._cells.get(key)
            if cell is None:
                self._cells[key] = [z, z, 1]
            else:
                if z < cell[0]:
                    cell[0] = z
                if z > cell[1]:
                    cell[1] = z
                cell[2] += 1
            added += 1
        if added:
            self._cloud_frames += 1
            self._point_total += added
            self._dirty = True
            if msg.header.frame_id:
                self.frame_id = msg.header.frame_id

    def cb_publish(self):
        if not self._cells:
            return
        if not self._dirty and self._cloud_frames > 0:
            # Still republish occasionally so late subscribers see the map
            pass
        self._dirty = False

        xs = [k[0] for k in self._cells]
        ys = [k[1] for k in self._cells]
        ix_min, ix_max = min(xs), max(xs)
        iy_min, iy_max = min(ys), max(ys)
        # pad 2 cells
        ix_min -= 2
        iy_min -= 2
        ix_max += 2
        iy_max += 2
        w = ix_max - ix_min + 1
        h = iy_max - iy_min + 1
        if w * h > 8_000_000:
            self.get_logger().warn(f'grid too large {w}x{h}, skip publish')
            return

        data = [-1] * (w * h)
        for (ix, iy), (zmin, zmax, _cnt) in self._cells.items():
            cx = ix - ix_min
            cy = iy - iy_min
            if cx < 0 or cy < 0 or cx >= w or cy >= h:
                continue
            spread = zmax - zmin
            idx = cy * w + cx
            if spread >= self.occ_min or zmax > self.ground_z + self.occ_min:
                data[idx] = 100
            else:
                data[idx] = 0

        grid = OccupancyGrid()
        grid.header = Header()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.frame_id
        info = MapMetaData()
        info.resolution = float(self.resolution)
        info.width = w
        info.height = h
        info.origin = Pose()
        info.origin.position.x = ix_min * self.resolution
        info.origin.position.y = iy_min * self.resolution
        info.origin.orientation.w = 1.0
        grid.info = info
        grid.data = data
        self.pub.publish(grid)

    def reset(self):
        self._cells.clear()
        self._cloud_frames = 0
        self._point_total = 0
        self._dirty = True


def main(args=None):
    rclpy.init(args=args)
    node = MapBuildingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
