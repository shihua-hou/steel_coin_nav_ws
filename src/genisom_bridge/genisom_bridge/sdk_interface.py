"""Wrapper around mc_sdk_zsl_1_py (AgiBot D1 Edu Ultra / ZSL-1 HighLevel API).

Coordinates: forward X, left Y, up Z (matches ROS REP-103).
Speed dead-bands per SDK docs:
  vx: (-3,-0.05] or [0.05,3)   -> values inside (-0.05,0.05) must be 0
  vy: (-1,-0.1] or [0.1,1)     -> inside (-0.1,0.1) must be 0
  yaw: (-3,-0.02] or [0.02,3)  -> inside (-0.02,0.02) must be 0
"""
import os
import platform
import sys
import time

SDK_REL = os.path.abspath(
    '/home/linaro/agibot_D1_Edu-Ultra/lib/zsl-1/' +
    platform.machine().replace('amd64', 'x86_64').replace('arm64', 'aarch64'))
if SDK_REL not in sys.path:
    sys.path.insert(0, SDK_REL)

import mc_sdk_zsl_1_py  # noqa: E402

LEG_NAMES = ['FR', 'FL', 'RR', 'RL']


def _clamp_deadband(value, lo, hi):
    """If |value| < lo -> 0, else clamp to sign-preserved [lo, hi]."""
    if abs(value) < lo:
        return 0.0
    return max(-hi, min(hi, value))


class GenisomSDK:
    def __init__(self, local_ip, local_port, dog_ip):
        self.local_ip = local_ip
        self.local_port = int(local_port)
        self.dog_ip = dog_ip
        self.app = mc_sdk_zsl_1_py.HighLevel()
        self._connect()

    def _connect(self):
        self.app.initRobot(self.local_ip, self.local_port, self.dog_ip)

    def connected(self):
        try:
            return bool(self.app.checkConnect())
        except Exception:
            return False

    def wait_connected(self, timeout_sec=2.0, interval=0.2):
        """initRobot 后握手常需几百毫秒；启动时立刻 checkConnect 会误报断开。"""
        deadline = time.monotonic() + float(timeout_sec)
        while time.monotonic() < deadline:
            if self.connected():
                return True
            time.sleep(interval)
        return self.connected()

    def reconnect(self):
        """断线后重建 HighLevel 并重新 initRobot（仅 checkConnect 不够，旧 socket 会假死）。"""
        try:
            self.app = mc_sdk_zsl_1_py.HighLevel()
            self._connect()
            return self.wait_connected(timeout_sec=2.5, interval=0.25)
        except Exception:
            return False

    # ---------- motion ----------
    def move(self, vx, vy, yaw_rate):
        vx = _clamp_deadband(vx, 0.05, 3.0)
        vy = _clamp_deadband(vy, 0.1, 1.0)
        yaw = _clamp_deadband(yaw_rate, 0.02, 3.0)
        return self.app.move(vx, vy, yaw)

    def stand_up(self):
        return self.app.standUp()

    def lie_down(self):
        return self.app.lieDown()

    def passive(self):
        return self.app.passive()

    def jump(self):
        return self.app.jump()

    def front_jump(self):
        return self.app.frontJump()

    def backflip(self):
        return self.app.backflip()

    def shake_hand(self):
        return self.app.shakeHand()

    # ---------- state ----------
    def get_quaternion(self):
        q = self.app.getQuaternion()
        return [float(q[0]), float(q[1]), float(q[2]), float(q[3])]  # w,x,y,z

    def get_rpy(self):
        r = self.app.getRPY()
        return [float(r[0]), float(r[1]), float(r[2])]

    def get_body_acc(self):
        a = self.app.getBodyAcc()
        return [float(a[0]), float(a[1]), float(a[2])]

    def get_body_gyro(self):
        g = self.app.getBodyGyro()
        return [float(g[0]), float(g[1]), float(g[2])]

    def get_position(self):
        p = self.app.getPosition()
        return [float(p[0]), float(p[1]), float(p[2])]

    def get_world_velocity(self):
        v = self.app.getWorldVelocity()
        return [float(v[0]), float(v[1]), float(v[2])]

    def get_body_velocity(self):
        v = self.app.getBodyVelocity()
        return [float(v[0]), float(v[1]), float(v[2])]

    def get_battery(self):
        return int(self.app.getBatteryPower())

    def get_ctrl_mode(self):
        return int(self.app.getCurrentCtrlmode())

    def get_joint_states(self):
        """Return (pos[12], vel[12], eff[12]) ordered abad,hip,knee x [FR,FL,RR,RL]."""
        pos = (list(self.app.getLegAbadJoint()) + list(self.app.getLegHipJoint())
               + list(self.app.getLegKneeJoint()))
        vel = (list(self.app.getLegAbadJointVel()) + list(self.app.getLegHipJointVel())
               + list(self.app.getLegKneeJointVel()))
        eff = (list(self.app.getLegAbadJointTorque()) + list(self.app.getLegHipJointTorque())
               + list(self.app.getLegKneeJointTorque()))
        return [float(x) for x in pos], [float(x) for x in vel], [float(x) for x in eff]

    @staticmethod
    def joint_names():
        return ([f'{leg}_abad' for leg in LEG_NAMES] +
                [f'{leg}_hip' for leg in LEG_NAMES] +
                [f'{leg}_knee' for leg in LEG_NAMES])
