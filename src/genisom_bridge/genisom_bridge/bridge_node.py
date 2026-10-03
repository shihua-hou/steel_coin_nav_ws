"""genisom_bridge ROS2 node: bridges AgiBot ZSL-1 mc_sdk <-> ROS2.

Topics:
  sub  /cmd_vel            geometry_msgs/Twist   -> move(vx, vy, yaw_rate)
  pub  /imu/data_raw       sensor_msgs/Imu
  pub  /odom_dog           nav_msgs/Odometry     (机体里程计，始终发布，不发 TF)
  pub  /odom               nav_msgs/Odometry     (仅当 publish_odom_tf=true)
  pub  /joint_states       sensor_msgs/JointState (12 joints)
  pub  /battery_state      sensor_msgs/BatteryState
  pub  /robot_ctrl_mode    std_msgs/Int32
Services (std_srvs/Trigger):
  /bridge/stand_up  /bridge/lie_down  /bridge/passive
  /bridge/jump      /bridge/front_jump /bridge/backflip /bridge/shake_hand
TF: base_link -> livox_frame (静态外参);
    odom -> base_link 仅当 publish_odom_tf=true（导航默认关，由 lio_tf_bridge 发 footprint）
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, JointState
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from genisom_bridge.sdk_interface import GenisomSDK

CTRL_MODE_TEXT = {
    0: 'lie_down_damping', 1: 'stand', 10: 'lie_down_free',
    18: 'moving', 21: 'action', 51: 'lying_down',
}

# HighLevel 动作类接口（standUp/lieDown/passive/move/jump/frontJump/backflip/
# shakeHand/attitudeControl/twoLegStand）均返回 uint32_t 状态码，0=成功，
# 参见 4.1HighLevel函数介绍.md
SDK_ERR_TEXT = {
    0: 'ok',
    0x3012: 'motor_data_lost',
    0x3010: 'motor_disabled',
    0x3011: 'motor_fault',
    0x3009: 'motor_angle_overrange',
    0x3007: 'state_switch_failed',
    0x3013: 'cmd_velocity_too_large',
}


def rpy_to_quat(r, p, y):
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


class GenisomBridge(Node):
    def __init__(self):
        super().__init__('genisom_bridge')

        self.local_ip = self.declare_parameter('local_ip', '192.168.168.150').value
        self.local_port = self.declare_parameter('local_port', 43988).value
        self.dog_ip = self.declare_parameter('dog_ip', '192.168.168.168').value
        self.state_rate = self.declare_parameter('state_rate', 20.0).value
        self.cmd_min_interval = self.declare_parameter('cmd_min_interval', 0.05).value
        self.lidar_x = self.declare_parameter('lidar_x', 0.0).value
        self.lidar_y = self.declare_parameter('lidar_y', 0.0).value
        self.lidar_z = self.declare_parameter('lidar_z', 0.0).value
        self.lidar_roll = self.declare_parameter('lidar_roll', 0.0).value
        self.lidar_pitch = self.declare_parameter('lidar_pitch', 0.0).value
        self.lidar_yaw = self.declare_parameter('lidar_yaw', 0.0).value
        # Default false: Nav2 footprint TF 由 lio_tf_bridge 根据 /odom_dog 发，避免双 TF
        self.publish_odom_tf = self.declare_parameter('publish_odom_tf', False).value

        self._cmd_lock = threading.Lock()
        self._last_cmd_time = 0.0
        self._last_cmd_wall = 0.0
        self._cmd_nonzero = False
        self.cmd_deadman_sec = float(
            self.declare_parameter('cmd_deadman_sec', 0.55).value)

        self.get_logger().info(
            f'Connecting SDK: local={self.local_ip}:{self.local_port} dog={self.dog_ip}')
        self.sdk = GenisomSDK(self.local_ip, self.local_port, self.dog_ip)
        self._sdk_last_reconnect = 0.0
        self._sdk_warn_ts = 0.0
        # initRobot 后握手常要 0.5~2s；立刻 checkConnect 会假失败，之后又从不重连
        if self.sdk.wait_connected(timeout_sec=2.5, interval=0.25):
            self.get_logger().info('SDK connected')
        else:
            self.get_logger().error(
                'SDK checkConnect failed - will auto-reconnect on state timer')

        # RELIABLE：与 rosbridge / web_ops 默认发布端匹配；BEST_EFFORT 会导致
        # 部分发布端发现不相容，网页摇杆指令到不了狗。
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        self.sub_cmd_vel = self.create_subscription(
            Twist, 'cmd_vel', self.cb_cmd_vel, cmd_qos)

        self.pub_imu = self.create_publisher(Imu, 'imu/data_raw', 20)
        # 机体里程计始终发布，供导航短时运动；不发 TF，避免与 lio_tf_bridge 冲突
        self.pub_odom_dog = self.create_publisher(Odometry, 'odom_dog', 20)
        self.pub_odom = None
        self.tf_bc = None
        if self.publish_odom_tf:
            self.pub_odom = self.create_publisher(Odometry, 'odom', 20)
            self.tf_bc = TransformBroadcaster(self)
        self.pub_joint = self.create_publisher(JointState, 'joint_states', 20)
        self.pub_batt = self.create_publisher(BatteryState, 'battery_state', 10)
        self.pub_mode = self.create_publisher(Int32, 'robot_ctrl_mode', 10)

        self.tf_static_bc = StaticTransformBroadcaster(self)
        self._publish_static_lidar_tf()

        for name, fn in [('stand_up', self.sdk.stand_up), ('lie_down', self.sdk.lie_down),
                         ('passive', self.sdk.passive), ('jump', self.sdk.jump),
                         ('front_jump', self.sdk.front_jump), ('backflip', self.sdk.backflip),
                         ('shake_hand', self.sdk.shake_hand)]:
            self.create_service(Trigger, f'bridge/{name}',
                                self._make_srv_cb(fn))

        self.create_timer(1.0 / max(1.0, float(self.state_rate)), self.cb_state)
        # 遥控死区：超过 deadman 未收到 cmd_vel 则强制零速，防止网页/WiFi 丢松手包后继续走
        self.create_timer(0.05, self.cb_cmd_deadman)
        self.get_logger().info(
            f'genisom_bridge ready (publish_odom_tf={self.publish_odom_tf}, '
            f'/odom_dog always on, cmd_deadman={self.cmd_deadman_sec:.2f}s)')

    def _publish_static_lidar_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'livox_frame'
        t.transform.translation.x = self.lidar_x
        t.transform.translation.y = self.lidar_y
        t.transform.translation.z = self.lidar_z
        q = rpy_to_quat(self.lidar_roll, self.lidar_pitch, self.lidar_yaw)
        t.transform.rotation.x, t.transform.rotation.y = q[1], q[2]
        t.transform.rotation.z, t.transform.rotation.w = q[3], q[0]
        self.tf_static_bc.sendTransform(t)

    def _make_odom_msg(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        p = self.sdk.get_position()
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = p
        rpy = self.sdk.get_rpy()
        q = rpy_to_quat(rpy[0], rpy[1], rpy[2])
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y = q[1], q[2]
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q[3], q[0]
        v = self.sdk.get_world_velocity()
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = v
        g = self.sdk.get_body_gyro()
        msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = g
        return msg, q, p

    def cb_cmd_vel(self, msg: Twist):
        # 零速必须立刻下发：WiFi 抖动时松手归零若被 cmd_min_interval 丢掉，
        # 狗会一直执行上一次非零速度（用户反馈「松手还走」）。
        is_stop = (
            abs(msg.linear.x) < 1e-9
            and abs(msg.linear.y) < 1e-9
            and abs(msg.angular.z) < 1e-9)
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._cmd_lock:
            if (not is_stop) and (now - self._last_cmd_time < self.cmd_min_interval):
                return
            self._last_cmd_time = now
            self._last_cmd_wall = time.monotonic()
            self._cmd_nonzero = not is_stop
        try:
            self.sdk.move(msg.linear.x, msg.linear.y, msg.angular.z)
        except Exception as e:
            self.get_logger().warn(f'cmd_vel -> move failed: {e}')

    def cb_cmd_deadman(self):
        """遥控超时自动停车：前端漏发零速 / 图传占满 WiFi 时的最后一道保险。"""
        if not self._cmd_nonzero:
            return
        if (time.monotonic() - self._last_cmd_wall) < self.cmd_deadman_sec:
            return
        with self._cmd_lock:
            if not self._cmd_nonzero:
                return
            if (time.monotonic() - self._last_cmd_wall) < self.cmd_deadman_sec:
                return
            self._cmd_nonzero = False
            self._last_cmd_time = self.get_clock().now().nanoseconds * 1e-9
        try:
            self.sdk.move(0.0, 0.0, 0.0)
            self.get_logger().warn(
                f'cmd_vel deadman ({self.cmd_deadman_sec:.2f}s) → force stop')
        except Exception as e:
            self.get_logger().warn(f'deadman stop failed: {e}')

    def cb_state(self):
        try:
            if not self.sdk.connected():
                now = time.monotonic()
                # 每 3s 真正重连一次（重建 HighLevel + initRobot）
                if (now - self._sdk_last_reconnect) >= 3.0:
                    self._sdk_last_reconnect = now
                    self.get_logger().warn('SDK disconnected — reconnecting…')
                    if self.sdk.reconnect():
                        self.get_logger().info('SDK reconnected')
                    return
                if (now - self._sdk_warn_ts) >= 5.0:
                    self._sdk_warn_ts = now
                    self.get_logger().warn(
                        'SDK still disconnected (next reconnect soon)')
                return
            self._publish_imu()
            odom_msg, q, p = self._make_odom_msg()
            self.pub_odom_dog.publish(odom_msg)
            if self.publish_odom_tf and self.pub_odom is not None:
                self.pub_odom.publish(odom_msg)
                if self.tf_bc is not None:
                    t = TransformStamped()
                    t.header.stamp = odom_msg.header.stamp
                    t.header.frame_id = 'odom'
                    t.child_frame_id = 'base_link'
                    t.transform.translation.x = p[0]
                    t.transform.translation.y = p[1]
                    t.transform.translation.z = p[2]
                    t.transform.rotation.x, t.transform.rotation.y = q[1], q[2]
                    t.transform.rotation.z, t.transform.rotation.w = q[3], q[0]
                    self.tf_bc.sendTransform(t)
            self._publish_joints()
            self._publish_battery()
            self._publish_mode()
        except Exception as e:
            self.get_logger().warn(f'state publish error: {e}')

    def _publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        q = self.sdk.get_quaternion()
        msg.orientation.x, msg.orientation.y = q[1], q[2]
        msg.orientation.z, msg.orientation.w = q[3], q[0]
        g = self.sdk.get_body_gyro()
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = g
        a = self.sdk.get_body_acc()
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = a
        self.pub_imu.publish(msg)

    def _publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = GenisomSDK.joint_names()
        pos, vel, eff = self.sdk.get_joint_states()
        msg.position = pos
        msg.velocity = vel
        msg.effort = eff
        self.pub_joint.publish(msg)

    def _publish_battery(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        try:
            msg.percentage = float(self.sdk.get_battery()) / 100.0
        except Exception:
            msg.percentage = float('nan')
        self.pub_batt.publish(msg)

    def _publish_mode(self):
        msg = Int32()
        msg.data = int(self.sdk.get_ctrl_mode())
        self.pub_mode.publish(msg)

    def _make_srv_cb(self, fn):
        def cb(req, res):
            try:
                code = int(fn())
                res.success = (code == 0)
                res.message = SDK_ERR_TEXT.get(code, f'error_0x{code:x}')
            except Exception as e:
                res.success = False
                res.message = str(e)
            return res
        return cb


def main(args=None):
    rclpy.init(args=args)
    node = GenisomBridge()
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
