#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓放任务 + HDF5 数据采集控制器。

继承 PickPlaceController，在抓放任务基础上添加 15Hz 数据录制和 HDF5 保存。
"""

import os
import sys
import threading
from time import sleep

import numpy as np
import h5py
import cv2

# 支持直接运行和包导入两种方式
try:
    from .pick_place_controller import PickPlaceController
except ImportError:
    _dir = os.path.dirname(os.path.abspath(__file__))
    if _dir not in sys.path:
        sys.path.insert(0, _dir)
    from pick_place_controller import PickPlaceController


# 录制频率 & 图像缺帧占位尺寸
SAVE_HZ = 30.0
PLACEHOLDER_IMG_SHAPE = (360, 640, 3)
PLACEHOLDER_DEPTH_SHAPE = (360, 640)
# 任务成功阈值（苹果到盘心距离，米）
TASK_SUCCESS_DIST = 0.12


class PickPlaceSaveDataController(PickPlaceController):
    """在抓放任务基础上添加 HDF5 数据录制。"""

    # 数据缓存字段（顺序即写出顺序，便于一致性检查）
    _BUFFER_KEYS = (
        "arm_right", "hand_right", "arm_left", "hand_left",
        "action_arm_right", "action_arm_left",
        "action_hand_right", "action_hand_left",
        "img", "depth", "timestamp",
    )

    def __init__(self, node_name: str = "pick_place_save_data_node"):
        super().__init__(node_name=node_name)

        self.data_buffer = {k: [] for k in self._BUFFER_KEYS}
        self.is_saving = False
        self.dropped_frames = 0  # record_snapshot 异常计数

        # 15Hz 采样定时器
        self.save_interval = 1.0 / SAVE_HZ
        self.save_timer = self.create_timer(self.save_interval, self._timer_save_callback)

    # ── 数据录制 ──

    def start_save_data(self):
        """开始录制数据。"""
        # 清空缓存以支持复用同一节点采多条（当前 main 只采一条，仍保留语义）
        for k in self._BUFFER_KEYS:
            self.data_buffer[k].clear()
        self.dropped_frames = 0
        self.is_saving = True
        self.get_logger().info(f"Started recording data at {SAVE_HZ:.0f}Hz (timer-driven)")

    def stop_save_data(self):
        """停止录制。"""
        self.is_saving = False

    def record_snapshot(self):
        """记录一帧数据快照（原子：要么全部 append，要么全部回滚）。"""
        if not self.is_saving:
            return
        # 先准备好所有字段值，全部就绪后再 append，避免半写入导致长度错位
        try:
            now = self.get_clock().now().nanoseconds / 1e9
            snapshot = {
                "arm_right": list(self.latest_arm_right_pos),
                "arm_left": list(self.latest_arm_left_pos),
                "hand_right": list(self.latest_hand_right_pos),
                "hand_left": list(self.latest_hand_left_pos),
                "action_arm_right": list(self.latest_action_arm_right),
                "action_arm_left": list(self.latest_action_arm_left),
                "action_hand_right": list(self.latest_action_hand_right),
                "action_hand_left": list(self.latest_action_hand_left),
                "img": (
                    self.latest_img
                    if self.latest_img is not None
                    else np.zeros(PLACEHOLDER_IMG_SHAPE, dtype=np.uint8)
                ),
                "depth": (
                    self.latest_depth
                    if self.latest_depth is not None
                    else np.zeros(PLACEHOLDER_DEPTH_SHAPE, dtype=np.uint16)
                ),
                "timestamp": now,
            }
        except Exception as e:
            self.dropped_frames += 1
            self.get_logger().error(f"Error recording snapshot (dropped={self.dropped_frames}): {e}")
            return

        for k, v in snapshot.items():
            self.data_buffer[k].append(v)

    def save_data(self):
        """保存数据到 HDF5 文件。"""
        self.stop_save_data()

        length = len(self.data_buffer["arm_right"])
        if length == 0:
            self.get_logger().warn("No frames recorded, skip saving.")
            return

        # 长度一致性自检
        lens = {k: len(self.data_buffer[k]) for k in self._BUFFER_KEYS}
        if len(set(lens.values())) != 1:
            self.get_logger().error(f"Buffer length mismatch, abort save: {lens}")
            return

        ts = self.get_clock().now().seconds_nanoseconds()
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_dir = os.path.dirname(os.path.dirname(script_dir))
        dataset_root = os.path.join(project_dir, "dataset")
        dir_name = os.path.join(dataset_root, f"{ts[0]}")
        # 首次创建时一次性放权；不再每次重复 chmod root
        new_dir = not os.path.exists(dir_name)
        os.makedirs(dir_name, exist_ok=True)
        if new_dir:
            try:
                os.chmod(dir_name, 0o777)
            except PermissionError:
                pass
        filename = os.path.join(dir_name, "trajectory.hdf5")
        self.get_logger().info(f"Saving {length} frames to {filename}...")

        with h5py.File(filename, "w") as f:
            f.create_dataset("puppet/arm_right_position_align/data", data=np.array(self.data_buffer["arm_right"]))
            f.create_dataset("puppet/end_effector_right_position_align/data", data=np.array(self.data_buffer["hand_right"]))
            f.create_dataset("puppet/arm_left_position_align/data", data=np.array(self.data_buffer["arm_left"]))
            f.create_dataset("puppet/end_effector_left_position_align/data", data=np.array(self.data_buffer["hand_left"]))
            f.create_dataset("action/arm_right_position_align/data", data=np.array(self.data_buffer["action_arm_right"]))
            f.create_dataset("action/arm_left_position_align/data", data=np.array(self.data_buffer["action_arm_left"]))
            f.create_dataset("action/end_effector_right_position_align/data", data=np.array(self.data_buffer["action_hand_right"]))
            f.create_dataset("action/end_effector_left_position_align/data", data=np.array(self.data_buffer["action_hand_left"]))
            f.create_dataset("observations/timestamp", data=np.array(self.data_buffer["timestamp"]))

            # 图像 JPEG 压缩存储
            dt = h5py.special_dtype(vlen=np.dtype("uint8"))
            img_ds = f.create_dataset("camera_observations/color_images/camera_head", (length,), dtype=dt)
            for i, img_rgb in enumerate(self.data_buffer["img"]):
                img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
                success, encoded_img = cv2.imencode(".jpg", img_bgr)
                if success:
                    img_ds[i] = encoded_img.flatten()
                else:
                    self.get_logger().error(f"Failed to encode image {i}")

            # 深度 PNG 无损压缩（uint16 毫米）
            depth_ds = f.create_dataset("camera_observations/depth_images/camera_head", (length,), dtype=dt)
            for i, depth_mm in enumerate(self.data_buffer["depth"]):
                success, encoded_depth = cv2.imencode(".png", depth_mm)
                if success:
                    depth_ds[i] = encoded_depth.flatten()
                else:
                    self.get_logger().error(f"Failed to encode depth {i}")

        try:
            os.chmod(filename, 0o666)
        except PermissionError:
            pass
        self.get_logger().info(
            f"Data saved: {length} frames, dropped={self.dropped_frames}."
        )

    def _timer_save_callback(self):
        """15Hz 定时回调。"""
        self.record_snapshot()

    # ── 重写任务流程 ──

    def run_task(self):
        """完整抓放流程 + 数据保存。"""
        self.reset()
        self.start_save_data()  # ← 关键修复：之前从未开启录制
        x, y = self.random_apple()
        sleep(5)
        self.pick(x, y)
        self.place()
        self.home()
        sleep(2)
        self.stop_save_data()
        self.get_logger().info(f"Final Task Completion Check: {self.latest_task_dist:.4f}")
        if self.latest_task_dist < TASK_SUCCESS_DIST:
            self.save_data()
        else:
            self.get_logger().warn(
                f"Task not completed (apple not in plate). "
                f"latest_task_dist={self.latest_task_dist:.4f}. Data will NOT be saved."
            )
        self.reset_sim()


def main():
    import rclpy
    rclpy.init()
    node = PickPlaceSaveDataController()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run_task()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        # 关定时器，避免 shutdown 后还触发回调
        try:
            node.save_timer.cancel()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
