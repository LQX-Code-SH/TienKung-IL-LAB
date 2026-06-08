#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机器人位置初始化脚本，用于模型推理前将机器人复位到预设位置。

在 lerobot-tienkung 容器内运行：
    source /opt/ros/humble/setup.bash
    python3 /ubt_IL/scripts/deploy/reset.py

前置条件：ROS2 DDS 可达（真机或 Isaac Sim 已启动）。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from bodyctrl_msgs.msg import MotorStatusMsg, MotorStatus, CmdSetMotorPosition, SetMotorPosition

from time import sleep

left_handPos = [-0.152, 0.068, 0.135, -1.155, 0.124, -0.361, -0.006]
right_hanPos = [-0.291, -0.003, -0.136, -1.155, -0.124, -0.361, 0.194]


def c(name, pos):
    return SetMotorPosition(name=name, pos=pos, spd=0.2, cur=5.0)


def cArr(arr, offset):
    return [c(i + offset, arr[i]) for i in range(len(arr))]


def h(data):
    msg = JointState()
    msg.name = [f"{i}" for i in range(1, 7)]
    msg.position = [float(i) for i in data]
    return msg


class MotorResetNode(Node):
    def push(self, type, msg, side="right"):
        self.get_logger().info(f"Publishing to {type} with data: {msg}")
        if type == "arm":
            self.arm_pub.publish(CmdSetMotorPosition(cmds=msg))
        elif type == "head":
            self.head_pub.publish(CmdSetMotorPosition(cmds=msg))
        elif type == "hand":
            if side == "left":
                self.left_hand_pub.publish(msg)
            else:
                self.right_hand_pub.publish(msg)
        else:
            self.get_logger().error(f"Unknown type: {type}")

    def __init__(self):
        super().__init__("motor_reset_node")
        self.arm_pub = self.create_publisher(CmdSetMotorPosition, "/arm/cmd_pos", 10)
        self.head_pub = self.create_publisher(CmdSetMotorPosition, "/head/cmd_pos", 10)
        self.right_hand_pub = self.create_publisher(JointState, "/inspire_hand/ctrl/right_hand", 10)
        self.left_hand_pub = self.create_publisher(JointState, "/inspire_hand/ctrl/left_hand", 10)

        sleep(1)
        self.reset_motors()

    def reset_motors(self):
        self.push("head", [c(1, 0.0), c(2, 0.35), c(3, 0.0)])
        hand_open = h([1, 1, 1, 1, 1, 0])
        self.push("hand", hand_open, side="right")
        self.push("hand", hand_open, side="left")
        self.push("arm", [c(22, -1.5)])
        sleep(3)
        self.push("arm", [c(24, -1.5)])
        sleep(2)
        self.push("arm", cArr(right_hanPos, 21))
        self.get_logger().info("Reset commands published to /arm/cmd_pos, /head/cmd_pos, and both hands")


def main():
    rclpy.init()
    node = MotorResetNode()
    sleep(1)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
