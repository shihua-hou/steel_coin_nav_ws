#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auto_relocalize',
            executable='auto_relocalize',
            name='auto_relocalize',
            output='screen',
            parameters=[{
                # 启动不盲搜；关掉看门狗自动全图搜（低置信度时会乱拉位姿）
                'auto_on_startup': False,
                'watchdog_en': False,
                # 对齐门槛：距离场容差后命中率可比「纯占格」高一截
                'min_black_hit': 0.50,
                # 手动重定位：≥该值才灌 /initialpose（旧纯占格常卡在 26~32%）
                'reloc_trigger_hit': 0.32,
                'black_hit_dist': 0.15,
                # 0=关闭「到点后自动重定位」（原先 0.55 常把正确终点拉到错误峰）
                'post_nav_hit': 0.0,
                'post_nav_settle_sec': 1.5,
                # 静止局部微调默认关：置信度~55% 时会把位姿来回拽，比不修更漂
                'local_refine_en': False,
                'local_refine_period_sec': 2.5,
                'local_xy_radius': 0.30,
                'local_yaw_deg': 12.0,
                'local_improve': 0.05,
                'local_min_hit': 0.65,
                'mid_wait_sec': 15.0,
                'reloc_cooldown_sec': 45.0,
                'accept_score': 0.40,
                'sigma': 0.25,
                'watchdog_sigma': 0.08,
                'coarse_step': 0.20,
                'coarse_yaw_step_deg': 10.0,
                'max_beam_range': 12.0,
                'min_clearance': 0.12,
                'num_threads': 4,
                'watchdog_count': 3,
            }],
        ),
    ])
