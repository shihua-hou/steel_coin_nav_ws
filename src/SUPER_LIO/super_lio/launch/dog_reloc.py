"""重定位模式 launch: 用 relocation_node 加载先验 map.pcd 做 NDT+ICP 重定位。
用法: ros2 launch super_lio dog_reloc.py rviz:=false
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('super_lio')
    config_yaml = os.path.join(pkg, 'config', 'dog_reloc.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'relocation.rviz')

    rviz_flag = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Whether to start RVIZ2'),
        Node(
            package='super_lio',
            executable='relocation_node',
            name='relocation_node',
            output='screen',
            parameters=[config_yaml],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='reloc_rviz',
            arguments=['-d', rviz_config, '--ros-args', '--log-level', 'warn'],
            condition=IfCondition(rviz_flag),
        ),
    ])
