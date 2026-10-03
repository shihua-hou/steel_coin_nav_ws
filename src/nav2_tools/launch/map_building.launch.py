from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_tools',
            executable='map_building_node',
            name='map_building_node',
            output='screen',
            parameters=[{
                'resolution': 0.05,
                # 重力系下障碍高度带：抬离地面、砍掉天花，避免墙斜切/地面噪声进占据
                'z_min': 0.12,
                'z_max': 1.05,
                'occ_min': 0.10,
                'ground_z': 0.08,
                'publish_rate': 2.0,
                'frame_id': 'world',
            }],
        ),
    ])
