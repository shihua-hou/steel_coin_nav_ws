"""Launch lio_tf_bridge with lidar extrinsic params (must match genisom_bridge).
用法: ros2 launch lio_tf_bridge bridge.launch.py [odom_source:=dog|lio]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom_source = LaunchConfiguration('odom_source')
    return LaunchDescription([
        DeclareLaunchArgument('odom_source', default_value='dog',
                              description='dog=底盘里程计(导航默认,抗漂); lio=Super-LIO(易漂,仅调试/重定位脚本)'),
        Node(
            package='lio_tf_bridge',
            executable='lio_tf_bridge',
            name='lio_tf_bridge',
            output='screen',
            parameters=[{
                'lidar_x': 0.25,
                'lidar_y': 0.0,
                'lidar_z': 0.45,
                'lidar_roll': -0.013647,
                'lidar_pitch': -0.342586,
                'lidar_yaw': 0.0,
                'odom_source': odom_source,
            }],
        ),
        # Static imu -> base_link (inverse of base_link->imu).
        # Super-LIO broadcasts world->imu (imu == IMU frame). lidar_imu extrinsic
        # is identity rotation + ~4cm translation, so imu ≈ lidar frame.
        # genisom_bridge publishes base_link->livox_frame; this closes the tree.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_to_base_link_static',
            arguments=[
                '-0.084303', '0.006341', '-0.507792',
                '0.006523', '-0.170463', '0.000000', '0.985343',
                'imu', 'base_link',
            ],
        ),
    ])
