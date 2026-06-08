#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""数据集 action 回放脚本：把 Pick_up_tiangong_all 中某条 episode 的 action 原样发回真机。

用途：
  - 数据采集质量回看
  - Bridge2 / 电机控制链路真机健康检查（不依赖模型推理）
  - 复现某条 episode 场景

在 lerobot-tienkung 容器内运行：
    source /opt/ros/humble/setup.bash
    python3 /ubt_IL/scripts/deploy/replay.py --episode 0 --rate 30

前置条件：ROS2 DDS 可达（真机或 Isaac Sim 已启动）；建议先跑一次 reset.py。
"""

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Dict, Tuple

import numpy as np
import pyarrow.parquet as pq

# ROS2 依赖延迟到真正发送时再导入，方便宿主机上跑 --dry-run / --help。
rclpy = None
Node = object
JointState = None
CmdSetMotorPosition = None
SetMotorPosition = None


def _import_ros2():
    """容器内（source /opt/ros/humble/setup.bash 后）导入 ROS2 相关模块。"""
    global rclpy, Node, JointState, CmdSetMotorPosition, SetMotorPosition
    import rclpy as _rclpy
    from rclpy.node import Node as _Node
    from sensor_msgs.msg import JointState as _JointState
    from bodyctrl_msgs.msg import (
        CmdSetMotorPosition as _CmdSetMotorPosition,
        SetMotorPosition as _SetMotorPosition,
    )
    rclpy = _rclpy
    Node = _Node
    JointState = _JointState
    CmdSetMotorPosition = _CmdSetMotorPosition
    SetMotorPosition = _SetMotorPosition


# 两种已知的 26 维 action 布局
LAYOUTS: Dict[str, Dict[str, slice]] = {
    # 天工：左臂-右臂-左手-右手
    "arms_then_hands": {
        "left_arm": slice(0, 7),
        "right_arm": slice(7, 14),
        "left_hand": slice(14, 20),
        "right_hand": slice(20, 26),
    },
    # tienkung / Bridge2 传统：左臂-左手-右臂-右手
    "interleaved": {
        "left_arm": slice(0, 7),
        "left_hand": slice(7, 13),
        "right_arm": slice(13, 20),
        "right_hand": slice(20, 26),
    },
}

# robot_type → 没有 action.names 时的默认布局
DEFAULT_LAYOUT_BY_ROBOT: Dict[str, str] = {
    "tiangong": "arms_then_hands",
    "tienkung": "interleaved",
}

# 默认数据集路径：脚本位于 ubt_IL/scripts/deploy/replay.py
DEFAULT_DATASET = Path(__file__).resolve().parents[2] / "dataset" / "Pick_up_tiangong_all"

# 配置文件路径（由 TienKungRobot._start_bridge() 写出）
DEFAULT_CONFIG_FILE = "/tmp/tienkung_bridge_config.json"


# ── Inspire hand clip logic ──────────────────────────────────────────────────
# IMPORTANT: This logic must match hand_utils.inspire_clip_position() in the
# plugin package (lerobot_robot_tienkung/hand_utils.py). If you change it here,
# change it there too.

def inspire_clip_position(position: list) -> list:
    """Inspire hand clip: clip [0,1], subtract 0.2 if < 0.9, round to 1 decimal."""
    position = [float(np.clip(pos, 0.0, 1.0)) for pos in position]
    position = [pos - 0.2 if pos < 0.9 else pos for pos in position]
    return [round(pos, 1) for pos in position]


def load_bridge_config(config_file: str | None = None) -> dict:
    """Load config written by TienKungRobot._start_bridge().

    Falls back to hardcoded defaults if the config file is not available,
    allowing standalone operation without the LeRobot process running.
    """
    path = config_file or DEFAULT_CONFIG_FILE
    if os.path.exists(path):
        try:
            with open(path) as f:
                return json.load(f)
        except (json.JSONDecodeError, OSError) as e:
            print(f"[replay] Warning: failed to read {path}: {e}, using defaults", file=sys.stderr)

    # Fallback defaults (matching constants.py in the plugin)
    return {
        "left_arm_motor_ids": list(range(11, 18)),
        "right_arm_motor_ids": list(range(21, 28)),
        "arm_speed": 0.5,
        "arm_current": 5.0,
        "hand_type": "inspire",
        "topic_arm_cmd": "/arm/cmd_pos",
        "topic_left_hand_cmd": "/inspire_hand/ctrl/left_hand",
        "topic_right_hand_cmd": "/inspire_hand/ctrl/right_hand",
    }


def detect_layout(info: dict) -> Tuple[str, Dict[str, slice]]:
    """从数据集 info.json 推断 26 维 action 布局，返回 (layout_name, slices)。

    优先依靠 action.names 的模式判别（A=arm/shoulder/elbow/wrist, H=finger/thumb），
    名字缺失时退回 robot_type 默认。
    """
    feats = info.get("features", {}).get("action", {})
    names = feats.get("names")
    if names:
        if len(names) != 26:
            raise ValueError(f"action 维度异常: 期望 26, 实际 {len(names)}")
        pattern = "".join(
            "H" if ("finger" in n or "thumb" in n) else "A" for n in names
        )
        if pattern == "A" * 14 + "H" * 12:
            return "arms_then_hands", LAYOUTS["arms_then_hands"]
        if pattern == "A" * 7 + "H" * 6 + "A" * 7 + "H" * 6:
            return "interleaved", LAYOUTS["interleaved"]
        raise ValueError(
            f"无法识别的 action 布局 (pattern={pattern}), "
            f"可手动指定 --layout {{{','.join(LAYOUTS)}}}"
        )

    robot = info.get("robot_type")
    if robot in DEFAULT_LAYOUT_BY_ROBOT:
        name = DEFAULT_LAYOUT_BY_ROBOT[robot]
        return name, LAYOUTS[name]
    raise ValueError(
        f"数据集既没有 action.names 也无法从 robot_type={robot!r} 推断布局, "
        f"请手动指定 --layout {{{','.join(LAYOUTS)}}}"
    )


def load_episode_actions(dataset_root: Path, episode: int) -> Tuple[np.ndarray, dict]:
    """读取指定 episode 的 action 序列，返回 ((T,26) float32 数组, info dict)。"""
    info_path = dataset_root / "meta" / "info.json"
    if not info_path.is_file():
        raise FileNotFoundError(f"找不到数据元信息: {info_path}")
    with open(info_path) as f:
        info = json.load(f)

    parquet = dataset_root / "data" / "chunk-000" / "file-000.parquet"
    if not parquet.is_file():
        raise FileNotFoundError(f"找不到数据文件: {parquet}")

    table = pq.read_table(parquet, columns=["episode_index", "frame_index", "action"])
    df = table.to_pandas()
    ep = df[df["episode_index"] == episode].sort_values("frame_index")
    if ep.empty:
        raise ValueError(f"数据集中没有 episode_index={episode}")

    actions = np.asarray([np.asarray(a, dtype=np.float32) for a in ep["action"].tolist()])
    if actions.shape[1] != 26:
        raise ValueError(f"action 维度异常: 期望 26, 实际 {actions.shape[1]}")
    return actions, info


def make_arm_msg(left_arm, right_arm, cfg: dict) -> CmdSetMotorPosition:
    """把左右臂各 N 维拼成一条 CmdSetMotorPosition。"""
    left_motor_ids = cfg["left_arm_motor_ids"]
    right_motor_ids = cfg["right_arm_motor_ids"]
    arm_speed = cfg.get("arm_speed", 0.5)
    arm_current = cfg.get("arm_current", 5.0)

    msg = CmdSetMotorPosition()
    for motor_id, val in zip(left_motor_ids, left_arm):
        msg.cmds.append(SetMotorPosition(name=motor_id, pos=float(val), spd=arm_speed, cur=arm_current))
    for motor_id, val in zip(right_motor_ids, right_arm):
        msg.cmds.append(SetMotorPosition(name=motor_id, pos=float(val), spd=arm_speed, cur=arm_current))
    return msg


def make_hand_msg(hand6: list, cfg: dict) -> JointState:
    """Build a JointState for hand control, applying hand-type clip logic."""
    hand_type = cfg.get("hand_type", "inspire")
    if hand_type == "inspire":
        pos = inspire_clip_position(hand6)
    else:
        pos = [float(v) for v in hand6]

    msg = JointState()
    msg.name = [str(i) for i in range(1, len(pos) + 1)]
    msg.position = pos
    return msg


class MotorReplayNode:
    """ROS2 节点的包装。__init__ 内才创建底层 Node，避免模块导入期触发 ROS2。"""

    def __init__(self, slices: Dict[str, slice], cfg: dict):
        self._slices = slices
        self._cfg = cfg
        self._node = Node("motor_replay_node")
        self.arm_pub = self._node.create_publisher(CmdSetMotorPosition, cfg["topic_arm_cmd"], 10)
        self.left_hand_pub = self._node.create_publisher(JointState, cfg["topic_left_hand_cmd"], 10)
        self.right_hand_pub = self._node.create_publisher(JointState, cfg["topic_right_hand_cmd"], 10)

    def publish_frame(self, action26: np.ndarray) -> None:
        s = self._slices
        self.arm_pub.publish(make_arm_msg(action26[s["left_arm"]], action26[s["right_arm"]], self._cfg))
        self.left_hand_pub.publish(make_hand_msg(action26[s["left_hand"]].tolist(), self._cfg))
        self.right_hand_pub.publish(make_hand_msg(action26[s["right_hand"]].tolist(), self._cfg))

    def destroy(self):
        self._node.destroy_node()


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--dataset", type=Path, default=DEFAULT_DATASET,
                   help=f"数据集根目录（默认 {DEFAULT_DATASET}）")
    p.add_argument("--episode", type=int, default=0, help="要回放的 episode_index（默认 0）")
    p.add_argument("--rate", type=float, default=30.0, help="发送频率 Hz（默认 30，匹配数据集 fps）")
    p.add_argument("--start", type=int, default=0, help="起始帧索引（默认 0）")
    p.add_argument("--end", type=int, default=-1, help="结束帧索引，-1 表示到末尾（默认 -1）")
    p.add_argument("--layout", choices=list(LAYOUTS), default=None,
                   help="强制指定 action 布局，缺省时自动从 info.json 推断")
    p.add_argument("--config-file", type=str, default=None,
                   help=f"桥接配置 JSON 文件路径（默认 {DEFAULT_CONFIG_FILE}）")
    p.add_argument("--dry-run", action="store_true", help="只打印不发布")
    return p.parse_args()


def run_replay(args, actions: np.ndarray, slices: Dict[str, slice], cfg: dict) -> None:
    """按 args.rate 频率回放 actions[start:end]。"""
    start = max(0, args.start)
    end = len(actions) if args.end < 0 else min(args.end, len(actions))
    if start >= end:
        raise ValueError(f"帧范围非法: start={start} >= end={end}")

    period = 1.0 / args.rate

    if args.dry_run:
        node = None
    else:
        _import_ros2()
        rclpy.init()
        node = MotorReplayNode(slices, cfg)
        time.sleep(1.0)  # 等 publisher 上线（沿用 reset.py 的做法）

    print(f"[replay] episode={args.episode} frames=[{start},{end}) "
          f"total={end - start} rate={args.rate}Hz dry_run={args.dry_run} "
          f"config_file={args.config_file or DEFAULT_CONFIG_FILE}")

    try:
        next_t = time.monotonic()
        for i in range(start, end):
            a = actions[i]
            if args.dry_run:
                print(f"frame {i} left_arm={a[slices['left_arm']].tolist()} "
                      f"left_hand={a[slices['left_hand']].tolist()} "
                      f"right_arm={a[slices['right_arm']].tolist()} "
                      f"right_hand={a[slices['right_hand']].tolist()}")
            else:
                node.publish_frame(a)

            next_t += period
            sleep_for = next_t - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                # 落后于节奏，重新对齐基准防止越拖越多
                next_t = time.monotonic()
    except KeyboardInterrupt:
        print("\n[replay] 中断，停止发送。")
    finally:
        if node is not None:
            node.destroy()
            rclpy.shutdown()

    print(f"[replay] 完成。机器人停在第 {end - 1} 帧位置（不再下发保持指令）。")


def main():
    args = parse_args()
    cfg = load_bridge_config(args.config_file)

    try:
        actions, info = load_episode_actions(args.dataset, args.episode)
        if args.layout:
            layout_name, slices = args.layout, LAYOUTS[args.layout]
            print(f"[replay] layout={layout_name} (强制) robot_type={info.get('robot_type')}")
        else:
            layout_name, slices = detect_layout(info)
            print(f"[replay] layout={layout_name} (自动) robot_type={info.get('robot_type')}")
    except (FileNotFoundError, ValueError) as e:
        print(f"[replay] 错误: {e}", file=sys.stderr)
        sys.exit(1)
    run_replay(args, actions, slices, cfg)


if __name__ == "__main__":
    main()
