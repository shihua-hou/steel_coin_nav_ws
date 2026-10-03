#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""teleop_dog.py - SDK 模式下用键盘控制机器狗建图移动（发布 /cmd_vel）
用法: 在主控 SSH 终端: python3 ~/steel_coin_nav_ws/teleop_dog.py
按键: w=前  s=后  a=左转  d=右转  空格=停  q=退出
速度: 平移 0.12 m/s, 旋转 0.2 rad/s（慢速安全）
"""
import rclpy, sys, select, termios, tty
from rclpy.node import Node
from geometry_msgs.msg import Twist

KEYS = {
    'w': (0.12, 0.0),
    's': (-0.12, 0.0),
    'a': (0.0, 0.20),
    'd': (0.0, -0.20),
}


class Teleop(Node):
    def __init__(self):
        super().__init__('teleop_dog')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.v = (0.0, 0.0)
        self.timer = self.create_timer(0.05, self.tick)

    def tick(self):
        m = Twist()
        m.linear.x = self.v[0]
        m.angular.z = self.v[1]
        self.pub.publish(m)


def main():
    rclpy.init()
    node = Teleop()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setraw(fd)
    print('\n=== teleop_dog 就绪 ===')
    print('w=前  s=后  a=左转  d=右转  空格=停  q=退出')
    print('速度: 0.12 m/s | 持续发布(0.55s 死区保护, 松手不会自动走)')
    print('安全: 前方清空, 有人盯防!\n', flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if ch == 'q':
                    break
                elif ch == ' ':
                    node.v = (0.0, 0.0)
                elif ch in KEYS:
                    node.v = KEYS[ch]
                else:
                    node.v = (0.0, 0.0)
    finally:
        node.v = (0.0, 0.0)
        node.tick()
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()


main()
