#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""天工 Pro 机器人控制基类。

集成 IK 求解、常量定义、arm/hand 控制原语。
任务逻辑（抓取、放置等）通过继承此类实现。

依赖：ROS2 (rclpy), ikpy, numpy
不依赖：Isaac Sim, source/ubt_sim
"""

import os
import sys
import yaml
import numpy as np
from typing import Optional
from time import sleep

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float32, Bool

try:
    from bodyctrl_msgs.msg import (
        MotorStatusMsg, MotorStatus,
        CmdSetMotorPosition, SetMotorPosition,
    )
except ImportError:
    raise ImportError(
        "bodyctrl_msgs not found. "
        "source /opt/ros/humble/setup.bash && "
        "colcon build --packages-select bodyctrl_msgs"
    )

try:
    from ikpy.chain import Chain
except ImportError:
    Chain = None

# 支持直接运行和包导入两种方式
try:
    from . import constants
except ImportError:
    _dir = os.path.dirname(os.path.abspath(__file__))
    if _dir not in sys.path:
        sys.path.insert(0, _dir)
    import constants


class RobotController(Node):
    """天工 Pro 机器人控制基类。

    子类继承此类实现具体任务逻辑，重写 _control_callback() 即可。
    """

    # ── 类常量 ──
    ID_TO_NAME = constants.ID_TO_NAME
    NAME_TO_ID = constants.NAME_TO_ID
    ID_ARM_L = constants.ID_ARM_L
    ID_ARM_R = constants.ID_ARM_R
    ARM_HOME = constants.ARM_HOME_PICK_PLACE
    HAND_OPEN = constants.HAND_OPEN
    HAND_CLOSE = constants.HAND_CLOSE

    def __init__(self, node_name: str = "robot_controller",
                 urdf_path: Optional[str] = None,
                 config_path: Optional[str] = None):
        super().__init__(node_name)

        # 加载 bridge_config.yaml
        self._config = self._load_config(config_path)

        # IK chain（实例持有，避免模块级全局变量的多实例冲突）
        self._right_arm_chain = None
        self.right_joints = list(self.ARM_HOME[7:])   # 右臂当前关节角
        self.left_joints = list(self.ARM_HOME[:7])     # 左臂当前关节角
        if urdf_path and Chain is not None:
            self._right_arm_chain = Chain.from_urdf_file(urdf_path, base_elements=["waist_yaw_link"])

        # ROS2 发布器/订阅器
        self._setup_publishers()
        self._setup_subscribers()

        # 状态追踪
        self.latest_arm_right_pos = [0.0] * 7
        self.latest_arm_left_pos = [0.0] * 7
        self.latest_hand_right_pos = [1.0] * 6
        self.latest_hand_left_pos = [1.0] * 6
        self.latest_action_arm_right = [0.0] * 7
        self.latest_action_arm_left = [0.0] * 7
        self.latest_action_hand_right = [1.0] * 6
        self.latest_action_hand_left = [1.0] * 6
        self.latest_img = None
        self.latest_depth = None  # uint16 毫米图（H, W）
        self.latest_task_dist = 1000.0

    # ──────────────── 配置加载 ────────────────

    @staticmethod
    def _load_config(config_path: Optional[str] = None) -> dict:
        """加载 bridge_config.yaml。"""
        if config_path is None:
            config_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                os.pardir, "bridges", "bridge_config.yaml"
            )
        try:
            with open(config_path) as f:
                return yaml.safe_load(f)
        except FileNotFoundError:
            return {}

    def _get_topic(self, section: str, key: str) -> str:
        """从配置读取 topic 名称，未找到则返回默认值。"""
        try:
            return self._config["topics"][section][key]["topic"]
        except (KeyError, TypeError):
            # 默认值与 bridge_config.yaml 保持一致
            defaults = {
                ("pub", "arm_cmd_pos"): "/arm/cmd_pos",
                ("pub", "head_cmd_pos"): "/head/cmd_pos",
                ("pub", "hand_r_ctrl"): "/inspire_hand/ctrl/right_hand",
                ("pub", "hand_l_ctrl"): "/inspire_hand/ctrl/left_hand",
                ("pub", "apple_offset"): "/scene/apple/offset",
                ("pub", "cmd_reset"): "/sim/cmd_reset",
                ("sub", "arm_status"): "/arm/status",
                ("sub", "hand_r_state"): "/inspire_hand/state/right_hand",
                ("sub", "hand_l_state"): "/inspire_hand/state/left_hand",
                ("sub", "image_rgb"): "/ob_camera_head/color/image_raw",
                ("sub", "image_depth"): "/ob_camera_head/depth/image_raw",
                ("sub", "arm_cmd_pos"): "/arm/cmd_pos",
                ("sub", "hand_r_ctrl"): "/inspire_hand/ctrl/right_hand",
                ("sub", "hand_l_ctrl"): "/inspire_hand/ctrl/left_hand",
                ("sub", "task_dist"): "/sim/task_completed",
            }
            return defaults.get((section, key), f"/{key}")

    # ──────────────── 发布器/订阅器 ────────────────

    def _setup_publishers(self):
        self.arm_pub = self.create_publisher(CmdSetMotorPosition, self._get_topic("pub", "arm_cmd_pos"), 10)
        self.head_pub = self.create_publisher(CmdSetMotorPosition, self._get_topic("pub", "head_cmd_pos"), 10)
        self.hand_pub = self.create_publisher(JointState, self._get_topic("pub", "hand_r_ctrl"), 10)
        self.hand_left_pub = self.create_publisher(JointState, self._get_topic("pub", "hand_l_ctrl"), 10)
        self.apple_pub = self.create_publisher(Point, self._get_topic("pub", "apple_offset"), 10)
        self.reset_pub = self.create_publisher(Bool, self._get_topic("pub", "cmd_reset"), 10)

    def _setup_subscribers(self):
        self.create_subscription(MotorStatusMsg, self._get_topic("sub", "arm_status"), self._arm_status_cb, 10)
        self.create_subscription(JointState, self._get_topic("sub", "hand_r_state"), self._hand_right_cb, 10)
        self.create_subscription(JointState, self._get_topic("sub", "hand_l_state"), self._hand_left_cb, 10)
        self.create_subscription(Image, self._get_topic("sub", "image_rgb"), self._image_cb, 10)
        self.create_subscription(Image, self._get_topic("sub", "image_depth"), self._depth_cb, 10)
        self.create_subscription(CmdSetMotorPosition, self._get_topic("sub", "arm_cmd_pos"), self._arm_cmd_cb, 10)
        self.create_subscription(JointState, self._get_topic("sub", "hand_r_ctrl"), self._hand_cmd_right_cb, 10)
        self.create_subscription(JointState, self._get_topic("sub", "hand_l_ctrl"), self._hand_cmd_left_cb, 10)
        self.create_subscription(Float32, self._get_topic("sub", "task_dist"), self._task_dist_cb, 10)

    # ──────────────── 消息构造 ────────────────

    @staticmethod
    def make_motor_cmd(name, pos, spd=None, cur=None):
        """构造单个电机位置命令。"""
        return SetMotorPosition(
            name=name, pos=pos,
            spd=spd if spd is not None else constants.DEFAULT_MOTOR_SPEED,
            cur=cur if cur is not None else constants.DEFAULT_MOTOR_CURRENT,
        )

    @staticmethod
    def make_motor_cmd_array(joints, id_offset):
        """构造一组电机位置命令（关节角 → 电机 ID 偏移）。"""
        return [
            RobotController.make_motor_cmd(i + id_offset, joints[i])
            for i in range(len(joints))
        ]

    @staticmethod
    def make_hand_msg(positions):
        """构造 6 自由度手部 JointState。"""
        msg = JointState()
        msg.name = [f"{i}" for i in range(1, 7)]
        msg.position = [float(v) for v in positions]
        return msg

    @staticmethod
    def make_uniform_hand_msg(val):
        """构造统一值 5 自由度手部 JointState。"""
        msg = JointState()
        msg.name = [f"{i}" for i in range(1, 6)]
        msg.position = [float(val) for _ in range(1, 6)]
        return msg

    # ──────────────── IK 求解 ────────────────

    @staticmethod
    def rotation_matrix(rx_deg, ry_deg, rz_deg) -> np.ndarray:
        """RPY 角度 → 旋转矩阵（Z-Y-X 顺序）。"""
        rx, ry, rz = np.radians([rx_deg, ry_deg, rz_deg])
        Rx = np.array([[1, 0, 0], [0, np.cos(rx), -np.sin(rx)], [0, np.sin(rx), np.cos(rx)]])
        Ry = np.array([[np.cos(ry), 0, np.sin(ry)], [0, 1, 0], [-np.sin(ry), 0, np.cos(ry)]])
        Rz = np.array([[np.cos(rz), -np.sin(rz), 0], [np.sin(rz), np.cos(rz), 0], [0, 0, 1]])
        return Rz @ Ry @ Rx

    def solve_ik(self, side, offset_xyz, offset_rpy=None, base_position=None, initial_guess=None):
        """通用 IK 求解接口。当前仅实现右臂。

        Args:
            side: "right" 或 "left"
            offset_xyz: 相对于 base_position 的位置偏移 [x, y, z]
            offset_rpy: 相对于 base_position 的旋转偏移（角度）[rx, ry, rz]
            base_position: FK 基准关节角（默认 self.right_joints/left_joints）
            initial_guess: IK 初始猜测关节角

        Returns:
            list[float]: 7 个关节角，或 None（求解失败）
        """
        if offset_rpy is None:
            offset_rpy = [0, 0, 0]

        if side == "right":
            return self._solve_right_arm(offset_xyz, offset_rpy, base_position, initial_guess)
        else:
            self.get_logger().warn(f"IK for side='{side}' not yet implemented")
            return None

    def _solve_right_arm(self, offset_xyz, offset_rpy, base_position, initial_guess):
        """右臂 IK 求解。"""
        if self._right_arm_chain is None:
            self.get_logger().error("IK chain not initialized (ikpy not available or urdf_path missing)")
            return None

        base_position = base_position if base_position is not None else self.right_joints
        initial_guess = initial_guess if initial_guess is not None else self.right_joints

        chain = self._right_arm_chain

        # 找到右臂链路在 chain 中的起止索引
        start_idx = None
        end_idx = None
        for i, link in enumerate(chain.links):
            if link.name == constants.RIGHT_ARM_JOINTS[0]:
                start_idx = i
            if link.name == constants.RIGHT_ARM_JOINTS[-1]:
                end_idx = i
        if start_idx is None or end_idx is None:
            self.get_logger().error("未找到右臂链路，请检查 URDF 和 RIGHT_ARM_JOINTS")
            return None

        mask = [False] * len(chain.links)
        for i in range(start_idx, end_idx + 1):
            mask[i] = True
        chain.active_links_mask = mask

        if not isinstance(offset_xyz, (list, np.ndarray)) or len(offset_xyz) != 3:
            raise ValueError(f"offset_xyz 必须为 3 维数组，当前: {offset_xyz}")

        # FK 基准
        base_joints = [0] * len(chain.links)
        if len(base_position) == 7:
            base_joints[1:8] = base_position

        # IK 初始猜测
        guess_joints = [0] * len(chain.links)
        if len(initial_guess) == 7:
            guess_joints[1:8] = initial_guess

        # 计算基准姿态的 FK
        current_frame = chain.forward_kinematics(base_joints)

        # 应用偏移量
        target_frame = current_frame.copy()
        target_frame[:3, 3] += offset_xyz
        R_offset = self.rotation_matrix(*offset_rpy)
        target_frame[:3, :3] = R_offset @ current_frame[:3, :3]

        try:
            angles = chain.inverse_kinematics(
                target_position=target_frame[:3, 3],
                target_orientation=target_frame[:3, :3],
                orientation_mode="all",
                initial_position=guess_joints,
            )
        except Exception as e:
            self.get_logger().error(f"IK inverse_kinematics 报错: {e}")
            return None

        idxs = [i for i, link in enumerate(chain.links) if link.name in constants.RIGHT_ARM_JOINTS]
        result = [angles[i] for i in idxs]
        self.right_joints = result
        return result

    # ──────────────── ARM 控制原语 ────────────────

    def move_arm(self, side, offset_xyz, offset_rpy=None, base_position=None,
                 speed=0.2, max_joint_step=0.03):
        """笛卡尔空间臂运动：IK 求解 + 关节步长限幅 + 15Hz 插值。

        Args:
            side: "right" 或 "left"
            offset_xyz: 相对于 base_position 的位置偏移
            offset_rpy: 相对于 base_position 的旋转偏移（角度）
            base_position: FK 基准关节角（默认当前关节角）
            speed: 末端最大线速度 (m/s)
            max_joint_step: 每步最大关节角增量 (rad)
        """
        if offset_rpy is None:
            offset_rpy = [0, 0, 0]

        dist = np.linalg.norm(offset_xyz)
        if dist < 1e-6:
            return

        dt = 1.0 / constants.CONTROL_LOOP_HZ
        total_time = dist / speed
        num_steps = max(int(total_time / dt), 1)

        if side == "right":
            prev_joints = list(self.right_joints)
            id_offset = 21
        else:
            prev_joints = list(self.left_joints)
            id_offset = 11

        # 先求解目标关节角
        target_joints = self.solve_ik(side, offset_xyz, offset_rpy,
                                       base_position=base_position,
                                       initial_guess=prev_joints)
        if target_joints is None:
            return

        # 插值运动
        for step in range(1, num_steps + 1):
            alpha = step / num_steps
            interp_xyz = [alpha * v for v in offset_xyz]
            interp_rpy = [alpha * v for v in offset_rpy]
            joints = self.solve_ik(side, interp_xyz, interp_rpy,
                                    base_position=base_position,
                                    initial_guess=prev_joints)
            if joints is not None:
                joints = [p + max(-max_joint_step, min(max_joint_step, j - p))
                          for j, p in zip(joints, prev_joints)]
                prev_joints = list(joints)
                cmds = self.make_motor_cmd_array(joints, id_offset)
                self.push("arm", cmds)
            sleep(dt)

        # 追加步数：限幅补偿
        while max(abs(t - p) for t, p in zip(target_joints, prev_joints)) > 1e-4:
            joints = [p + max(-max_joint_step, min(max_joint_step, t - p))
                      for t, p in zip(target_joints, prev_joints)]
            prev_joints = list(joints)
            cmds = self.make_motor_cmd_array(joints, id_offset)
            self.push("arm", cmds)
            sleep(dt)

    def move_right_arm(self, offset_xyz, offset_rpy=None, base_position=None, **kwargs):
        """移动右臂（便捷方法）。"""
        return self.move_arm("right", offset_xyz, offset_rpy, base_position, **kwargs)

    def move_left_arm(self, offset_xyz, offset_rpy=None, base_position=None, **kwargs):
        """移动左臂（便捷方法）。"""
        return self.move_arm("left", offset_xyz, offset_rpy, base_position, **kwargs)

    # ──────────────── HAND 控制原语 ────────────────

    def move_hand(self, side, target_pos, speed=0.8):
        """线性插值手部运动，15Hz。

        Args:
            side: "right" 或 "left"
            target_pos: 目标手指位置（6 维列表）
            speed: 手指最大速度（单位/s）
        """
        if side == "right":
            current_pos = list(self.latest_action_hand_right)
        else:
            current_pos = list(self.latest_action_hand_left)

        delta = [t - c for t, c in zip(target_pos, current_pos)]
        max_delta = max(abs(d) for d in delta)
        if max_delta < 1e-6:
            return

        dt = 1.0 / constants.CONTROL_LOOP_HZ
        total_time = max_delta / speed
        num_steps = max(int(total_time / dt), 1)

        for step in range(1, num_steps + 1):
            alpha = step / num_steps
            interp_pos = [c + alpha * d for c, d in zip(current_pos, delta)]
            msg = self.make_hand_msg(interp_pos)
            self.push("hand" if side == "right" else "hand_left", msg)
            sleep(dt)

    def open_hand(self, side):
        """张开手。"""
        return self.move_hand(side, self.HAND_OPEN)

    def close_hand(self, side, grip=0.3):
        """闭合手。"""
        return self.move_hand(side, [grip] * 5 + [0])

    # ──────────────── 通用控制 ────────────────

    def push(self, channel, msg):
        """分发消息到对应发布器。"""
        if channel == "arm":
            self.arm_pub.publish(CmdSetMotorPosition(cmds=msg))
        elif channel == "head":
            self.head_pub.publish(CmdSetMotorPosition(cmds=msg))
        elif channel == "hand":
            self.hand_pub.publish(msg)
        elif channel == "hand_left":
            self.hand_left_pub.publish(msg)
        else:
            self.get_logger().error(f"Unknown push channel: {channel}")

    def home(self):
        """双臂归位。"""
        cmds = (self.make_motor_cmd_array(self.left_joints, 11)
                + self.make_motor_cmd_array(self.right_joints, 21))
        self.push("arm", cmds)

    def reset(self):
        """完整复位序列：头部 → 手张开 → 臂展开 → 臂归位。"""
        self.push("head", [
            self.make_motor_cmd(1, 0.0),
            self.make_motor_cmd(2, 0.35),
            self.make_motor_cmd(3, 0.0),
        ])
        self.push("hand", self.make_hand_msg([1, 1, 1, 1, 1, 0]))
        self.push("arm", [
            self.make_motor_cmd(12, 1.5),
            self.make_motor_cmd(22, -1.5),
        ])
        sleep(2)
        self.push("arm", [
            self.make_motor_cmd(14, -1.5),
            self.make_motor_cmd(24, -1.5),
        ])
        sleep(2)
        cmds = (self.make_motor_cmd_array(list(self.ARM_HOME[:7]), 11)
                + self.make_motor_cmd_array(list(self.ARM_HOME[7:]), 21))
        self.push("arm", cmds)
        self.get_logger().info("Reset commands published")

    # ──────────────── 状态回调 ────────────────

    def _arm_status_cb(self, msg):
        """解析臂部电机状态。"""
        if not hasattr(msg, "status"):
            return
        msg_map = {m.name: m.pos for m in msg.status}

        def _extract(ids):
            return [msg_map.get(pid, 0.0) for pid in ids]

        self.latest_arm_right_pos = _extract(self.ID_ARM_R)
        self.latest_arm_left_pos = _extract(self.ID_ARM_L)

    def _arm_cmd_cb(self, msg):
        """解析臂部命令状态。"""
        if not hasattr(msg, "cmds") or not msg.cmds:
            return
        msg_map = {m.name: m.pos for m in msg.cmds}

        def _extract(ids):
            return [msg_map.get(pid, 0.0) for pid in ids]

        self.latest_action_arm_right = _extract(self.ID_ARM_R)
        self.latest_action_arm_left = _extract(self.ID_ARM_L)

    def _parse_hand_msg(self, msg):
        """解析手部 JointState 消息，返回 6 维位置列表。"""
        if len(msg.position) < 6:
            return None
        val_map = {}
        for n, p in zip(msg.name, msg.position):
            try:
                val_map[str(n)] = float(p)
            except (ValueError, TypeError):
                pass
        return [val_map.get(str(i), 0.0) for i in range(1, 7)]

    def _hand_right_cb(self, msg):
        vals = self._parse_hand_msg(msg)
        if vals:
            self.latest_hand_right_pos = vals

    def _hand_left_cb(self, msg):
        vals = self._parse_hand_msg(msg)
        if vals:
            self.latest_hand_left_pos = vals

    def _hand_cmd_right_cb(self, msg):
        pos = list(msg.position)
        if len(pos) >= 6:
            self.latest_action_hand_right = [float(p) for p in pos[:6]]
        elif len(pos) >= 5:
            self.latest_action_hand_right[:5] = [float(p) for p in pos[:5]]

    def _hand_cmd_left_cb(self, msg):
        pos = list(msg.position)
        if len(pos) >= 6:
            self.latest_action_hand_left = [float(p) for p in pos[:6]]
        elif len(pos) >= 5:
            self.latest_action_hand_left[:5] = [float(p) for p in pos[:5]]

    def _task_dist_cb(self, msg):
        self.latest_task_dist = msg.data

    def _image_cb(self, msg):
        """解码 ROS2 Image 消息。"""
        try:
            if msg.encoding == "rgb8":
                self.latest_img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            elif msg.encoding == "bgr8":
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                self.latest_img = img[..., ::-1]
        except Exception as e:
            self.get_logger().error(f"Image decode error: {e}")

    def _depth_cb(self, msg):
        """解码深度图，统一存为 uint16 毫米（H, W）。

        Isaac Sim 通常发 32FC1 米；真机 Orbbec 发 16UC1 毫米。两者都统一到 uint16 mm。
        """
        try:
            if msg.encoding == "16UC1":
                depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                self.latest_depth = depth.copy()
            elif msg.encoding == "32FC1":
                depth_m = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
                # 米 → 毫米，clip 到 uint16 范围（[0, 65.535m]）
                depth_mm = np.clip(depth_m * 1000.0, 0, 65535).astype(np.uint16)
                self.latest_depth = depth_mm
            else:
                self.get_logger().warn(f"Unsupported depth encoding: {msg.encoding}")
        except Exception as e:
            self.get_logger().error(f"Depth decode error: {e}")

    # ──────────────── 控制循环 ────────────────

    def _control_callback(self):
        """子类重写此方法实现任务逻辑。"""
        pass
