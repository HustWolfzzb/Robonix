#!/usr/bin/env python3
# -*-coding:utf8-*-

from ctrl_base import CtrlBase
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from piper_msgs.msg import PiperStatusMsg, PosCmd
import time
import numpy as np


class CtrlByROS(Node, CtrlBase):
    def __init__(self, debug_mode: bool = False):
        super().__init__("piper_ctrl_by_ros_node")
        self.debug = debug_mode
        self.enable_pub = self.create_publisher(Bool, "enable_flag", 1)
        self.pos_cmd_pub = self.create_publisher(PosCmd, "pos_cmd", 1)
        self.joint_state_pub = self.create_publisher(JointState, "joint_ctrl_single", 1)
        self.create_subscription(
            PiperStatusMsg, "arm_status", self.piper_status_callback, 1
        )
        self.create_subscription(
            JointState, "joint_states_single", self.joint_states_callback, 1
        )
        self.piper_status = PiperStatusMsg()
        self.joint_states = JointState()

        self.zero_joint_state = JointState()
        self.zero_joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.15, 0.0, 0.0]
        if self.debug:
            self.zero_joint_state.position = [10.0] * self.joint_num
        else:
            self.zero_joint_state.velocity = [100.0] * self.joint_num

        self.enable_pub.publish(Bool(data=True))
        self.reset()

    def __del__(self):
        try:
            self.reset()
        except TimeoutError as e:
            print(f"Error during reset: {e}")
        self.enable_pub.publish(Bool(data=False))

    def reset(self, timeout: int = 5):
        self.joint_state_pub.publish(self.zero_joint_state)
        start = time.time()
        while self.piper_status.motion_status == 0x01:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                raise TimeoutError("Reset operation timed out.")

    def send_a_step(self):
        if getattr(self, "joint_states_cmd", None) is None:
            return
        self.joint_state_pub.publish(self.joint_state_cmd)
        rclpy.spin_once(self, timeout_sec=0.1)
        while self.piper_status.motion_status == 0x01:
            rclpy.spin_once(self, timeout_sec=0.1)

    # def set_joint(self, joint_id2positions: dict[str, float]) -> dict[str, float]:
    #     self.joint_state_cmd = JointState()
    #     self.joint_state_cmd.name = list(joint_id2positions.keys())
    #     self.joint_state_cmd.position = list(joint_id2positions.values())
    #     self.joint_state_cmd.velocity = self.zero_joint_state.velocity
    #     return self.piper_status.out_of_limit

    def get_joint(self) -> np.ndarray:
        if (
            self.joint_states is None
            or len(self.joint_states.position) < self.joint_num
        ):
            return np.zeros(self.joint_num)
        return np.array(self.joint_states.position[: self.joint_num])

    def set_ee_pose(
        self, position: list[float], euler_angles: list[float], timeout: int = 5
    ):
        self.pos_cmd = PosCmd()
        self.pos_cmd.x = position[0]
        self.pos_cmd.y = position[1]
        self.pos_cmd.z = position[2]
        # 将欧拉角从度转换为弧度
        euler_angles = np.deg2rad(euler_angles)
        self.pos_cmd.roll = euler_angles[0]
        self.pos_cmd.pitch = euler_angles[1]
        self.pos_cmd.yaw = euler_angles[2]
        self.pos_cmd_pub.publish(self.pos_cmd)

        start = time.time()
        while self.piper_status.motion_status == 0x01:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                raise TimeoutError("Set end effector pose operation timed out.")

    def piper_status_callback(self, msg):
        self.piper_status = msg

    def joint_states_callback(self, msg):
        self.joint_states = msg
