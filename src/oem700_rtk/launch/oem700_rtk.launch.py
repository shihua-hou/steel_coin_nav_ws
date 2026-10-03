#!/usr/bin/env python3
"""OEM700 driver, optional map projection, optional map→odom arbiter."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    start_arbiter = LaunchConfiguration("start_arbiter")
    map_yaml = LaunchConfiguration("map")
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument(
            "start_arbiter", default_value="false",
            description="Publish map→odom. Only when AMCL tf_broadcast is false."),
        Node(
            package="oem700_rtk",
            executable="oem700_driver",
            name="oem700_driver",
            output="screen",
        ),
        Node(
            package="oem700_rtk",
            executable="rtk_to_map",
            name="rtk_to_map",
            output="screen",
            parameters=[{"map_yaml": map_yaml}],
        ),
        Node(
            package="oem700_rtk",
            executable="global_pose_arbiter",
            name="global_pose_arbiter",
            output="screen",
            condition=IfCondition(PythonExpression(["'", start_arbiter, "' == 'true'"])),
        ),
    ])
