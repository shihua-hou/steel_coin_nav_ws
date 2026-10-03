"""Launch genisom_bridge with optional lidar extrinsic params."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='genisom_bridge',
            executable='genisom_bridge',
            name='genisom_bridge',
            output='screen',
            parameters=[{
                'local_ip': '192.168.168.150',
                'local_port': 43988,
                'dog_ip': '192.168.168.168',
                'state_rate': 20.0,
                # 不发 odom TF；机体里程计走 /odom_dog，由 lio_tf_bridge 转成 Nav2 footprint
                'publish_odom_tf': False,
                'lidar_x': 0.25,
                'lidar_y': 0.0,
                'lidar_z': 0.45,
                'lidar_roll': -0.013647,
                'lidar_pitch': -0.342586,
                'lidar_yaw': 0.0,
            }],
        ),
    ])
