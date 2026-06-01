#!/usr/bin/env python3
"""测试 ROS2 相机图像订阅是否正常。

订阅指定 topic，收到第一帧后打印分辨率、编码等信息，
保存前 N 帧到 test_output/ 目录，超时则报错退出。

Usage:
  # 真机
  ROS_DOMAIN_ID=0 /usr/bin/python3 tool/test_ros_image.py
  # 仿真
  ROS_DOMAIN_ID=146 /usr/bin/python3 tool/test_ros_image.py
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_TOPIC = "/ob_camera_head/color/image_raw"
SAVE_COUNT = 3
TIMEOUT_S = 10.0


class ImageTestNode(Node):
    def __init__(self, topic: str, save_dir: str):
        super().__init__("test_ros_image")
        self.topic = topic
        self.save_dir = save_dir
        self.bridge = CvBridge()
        self.frame_count = 0
        self.first_frame_time = None
        self.done = False

        os.makedirs(save_dir, exist_ok=True)

        self.get_logger().info(f"Subscribing to: {topic}")
        self.sub = self.create_subscription(Image, topic, self._callback, 10)

    def _callback(self, msg: Image):
        if self.first_frame_time is None:
            self.first_frame_time = time.time()
            self.get_logger().info(
                f"First frame received: {msg.width}x{msg.height}, "
                f"encoding={msg.encoding}, step={msg.step}"
            )

        self.frame_count += 1

        if self.frame_count <= SAVE_COUNT:
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                path = os.path.join(self.save_dir, f"frame_{self.frame_count:03d}.jpg")
                cv2.imwrite(path, cv_img)
                self.get_logger().info(f"Saved: {path}")
            except Exception as e:
                self.get_logger().error(f"Failed to save frame: {e}")

        if self.frame_count == SAVE_COUNT:
            self.done = True


def main():
    parser = argparse.ArgumentParser(description="Test ROS2 camera image subscription")
    parser.add_argument("--topic", type=str, default=DEFAULT_TOPIC, help="Camera image topic")
    parser.add_argument("--timeout", type=float, default=TIMEOUT_S, help="Timeout in seconds")
    args = parser.parse_args()

    save_dir = os.path.join(SCRIPT_DIR, "test_output", "ros_image")
    rclpy.init()
    node = ImageTestNode(args.topic, save_dir)

    start = time.time()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.first_frame_time is None and (time.time() - start) > args.timeout:
                node.get_logger().error(
                    f"Timeout: no frame received on {args.topic} within {args.timeout}s"
                )
                node.destroy_node()
                rclpy.shutdown()
                sys.exit(1)
    except KeyboardInterrupt:
        pass

    if node.frame_count > 0:
        elapsed = time.time() - node.first_frame_time if node.first_frame_time else 0
        fps = node.frame_count / elapsed if elapsed > 0 else 0
        node.get_logger().info(
            f"Total frames: {node.frame_count}, avg FPS: {fps:.1f}"
        )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
