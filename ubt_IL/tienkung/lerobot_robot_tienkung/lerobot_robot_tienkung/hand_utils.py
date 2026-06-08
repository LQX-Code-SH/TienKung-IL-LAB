"""灵巧手控制工具（tienkung 插件侧）。

零外部依赖（仅 numpy + 标准库），可被 tienkung.py 直接 import。
ros2_deploy_bridge.py 运行在 Python 3.10 环境，无法 import 此模块，
需在桥接端维护一份相同的裁剪函数，并标注指向此文件为权威源。
"""

import numpy as np


def inspire_clip_position(position: list) -> list:
    """Inspire 手指裁剪：clip [0,1]，<0.9 减 0.2，round 到 1 位小数。

    此逻辑与 ros2_deploy_bridge.py 中的 inspire_clip_position() 保持同步。
    如修改此函数，须同步修改桥接端。
    """
    position = [float(np.clip(pos, 0.0, 1.0)) for pos in position]
    position = [pos - 0.2 if pos < 0.9 else pos for pos in position]
    return [round(pos, 1) for pos in position]


def inspire_clip_value(pos: float) -> float:
    """单值版本，用于 send_action 返回值。"""
    pos = float(np.clip(pos, 0.0, 1.0))
    pos = pos - 0.2 if pos < 0.9 else pos
    return round(pos, 1)


def clip_hand_position(position: list, hand_type: str = "inspire") -> list:
    """按手类型分派裁剪逻辑，可扩展。"""
    if hand_type == "inspire":
        return inspire_clip_position(position)
    # 未来扩展: elif hand_type == "brainco": return brainco_clip_position(position)
    raise ValueError(f"Unsupported hand_type: {hand_type!r}")


def clip_hand_value(pos: float, hand_type: str = "inspire") -> float:
    """单值版本，按手类型分派。"""
    if hand_type == "inspire":
        return inspire_clip_value(pos)
    raise ValueError(f"Unsupported hand_type: {hand_type!r}")
