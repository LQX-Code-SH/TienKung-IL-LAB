#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机器人位置初始化脚本，用于模型推理前将机器人复位到预设位置。

在 lerobot-tienkung 容器内运行：
    source /opt/ros/humble/setup.bash
    python3 /ubt_IL/scripts/deploy/reset.py

前置条件：ROS2 DDS 可达（真机或 Isaac Sim 已启动）。

配置文件（可选）：
    默认读取 /tmp/tienkung_bridge_config.json（由 TienKungRobot._start_bridge() 写出）。
    无此文件时使用内置默认值。
"""

import argparse
import json
import os
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from bodyctrl_msgs.msg import MotorStatusMsg, MotorStatus, CmdSetMotorPosition, SetMotorPosition

from time import sleep

# 配置文件路径（由 TienKungRobot._start_bridge() 写出）
DEFAULT_CONFIG_FILE = "/tmp/tienkung_bridge_config.json"

# 内置默认值（与 tienkung 插件 constants.py 保持同步）
_DEFAULTS = {
    "left_arm_motor_ids": [11, 12, 13, 14, 15, 16, 17],
    "right_arm_motor_ids": [21, 22, 23, 24, 25, 26, 27],
    "arm_speed": 0.5,
    "arm_current": 5.0,
    "reset_speed": 0.2,
    "reset_current": 5.0,
    "home_position": [
        -0.152, 0.068, 0.135, -1.155, 0.124, -0.361, -0.006,
        -0.291, -0.003, -0.136, -1.155, -0.124, -0.361, 0.194,
    ],
    "hand_open_position": [1.0, 1.0, 1.0, 1.0, 1.0, 0.0],
    "topic_arm_cmd": "/arm/cmd_pos",
    "topic_head_cmd": "/head/cmd_pos",
    "topic_left_hand_cmd": "/inspire_hand/ctrl/left_hand",
    "topic_right_hand_cmd": "/inspire_hand/ctrl/right_hand",
}


def load_bridge_config(config_file: str | None = None) -> dict:
    """Load config written by TienKungRobot._start_bridge()."""
    path = config_file or DEFAULT_CONFIG_FILE
    if os.path.exists(path):
        try:
            with open(path) as f:
                return json.load(f)
        except (json.JSONDecodeError, OSError) as e:
            print(f"[reset] Warning: failed to read {path}: {e}, using defaults", file=sys.stderr)
    return dict(_DEFAULTS)


class MotorResetNode(Node):
    def __init__(self, cfg: dict):
        super().__init__("motor_reset_node")
        self._cfg = cfg

        # Extract config values
        left_ids = cfg["left_arm_motor_ids"]
        right_ids = cfg["right_arm_motor_ids"]
        self._reset_speed = cfg.get("reset_speed", 0.2)
        self._reset_current = cfg.get("reset_current", 5.0)

        # Home position: first 7 = left arm, next 7 = right arm
        home = cfg.get("home_position", _DEFAULTS["home_position"])
        self._left_home = home[:len(left_ids)]
        self._right_home = home[len(left_ids):len(left_ids) + len(right_ids)]

        # Hand open position
        self._hand_open = cfg.get("hand_open_position", _DEFAULTS["hand_open_position"])

        # Topic names from config
        self.arm_pub = self.create_publisher(CmdSetMotorPosition, cfg["topic_arm_cmd"], 10)
        self.head_pub = self.create_publisher(CmdSetMotorPosition, cfg.get("topic_head_cmd", "/head/cmd_pos"), 10)
        self.right_hand_pub = self.create_publisher(JointState, cfg["topic_right_hand_cmd"], 10)
        self.left_hand_pub = self.create_publisher(JointState, cfg["topic_left_hand_cmd"], 10)

        sleep(1)
        self.reset_motors()

    def _make_motor_cmd(self, name, pos):
        return SetMotorPosition(name=name, pos=pos, spd=self._reset_speed, cur=self._reset_current)

    def _make_motor_cmd_array(self, positions, motor_ids):
        return [self._make_motor_cmd(mid, pos) for mid, pos in zip(motor_ids, positions)]

    def _make_hand_msg(self, data):
        msg = JointState()
        msg.name = [f"{i}" for i in range(1, len(data) + 1)]
        msg.position = [float(i) for i in data]
        return msg

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

    def reset_motors(self):
        # Head reset (motor IDs 1,2,3 are not in the config; kept as-is for now)
        self.push("head", [self._make_motor_cmd(1, 0.0), self._make_motor_cmd(2, 0.35), self._make_motor_cmd(3, 0.0)])

        # Open both hands
        hand_open_msg = self._make_hand_msg(self._hand_open)
        self.push("hand", hand_open_msg, side="right")
        self.push("hand", hand_open_msg, side="left")

        # Partial right arm movement (elbow) to clear pose before full reset
        right_ids = self._cfg["right_arm_motor_ids"]
        if len(right_ids) >= 2:
            self.push("arm", [self._make_motor_cmd(right_ids[1], -1.5)])  # elbow_pitch
            sleep(3)
            self.push("arm", [self._make_motor_cmd(right_ids[3], -1.5)])  # elbow_yaw
            sleep(2)

        # Full right arm reset
        self.push("arm", self._make_motor_cmd_array(self._right_home, right_ids))

        self.get_logger().info("Reset commands published to arm, head, and both hands")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--config-file", type=str, default=None,
                   help=f"桥接配置 JSON 文件路径（默认 {DEFAULT_CONFIG_FILE}）")
    return p.parse_args()


def main():
    args = parse_args()
    cfg = load_bridge_config(args.config_file)

    rclpy.init()
    node = MotorResetNode(cfg)
    sleep(1)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
