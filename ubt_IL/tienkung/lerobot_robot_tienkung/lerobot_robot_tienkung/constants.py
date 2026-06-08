"""天工 TienKung 机器人硬件常量（tienkung 插件侧）。

借鉴 ubt_sim/teleoperation/control/constants.py 的模式。
此模块与 ros2_deploy_bridge.py 独立维护——两边运行在不同 Python 环境
（3.12 vs 3.10），不交叉导入，仅通过 ZMQ + JSON 配置通信。
"""

# ── 电机 ID ↔ 关节名映射 ──
ID_TO_NAME = {
    # Left Arm
    11: "shoulder_pitch_l_joint",
    12: "shoulder_roll_l_joint",
    13: "shoulder_yaw_l_joint",
    14: "elbow_pitch_l_joint",
    15: "elbow_yaw_l_joint",
    16: "wrist_pitch_l_joint",
    17: "wrist_roll_l_joint",
    # Right Arm
    21: "shoulder_pitch_r_joint",
    22: "shoulder_roll_r_joint",
    23: "shoulder_yaw_r_joint",
    24: "elbow_pitch_r_joint",
    25: "elbow_yaw_r_joint",
    26: "wrist_pitch_r_joint",
    27: "wrist_roll_r_joint",
}

NAME_TO_ID = {v: k for k, v in ID_TO_NAME.items()}

# ── 分组 ID 列表 ──
ID_ARM_L = [11, 12, 13, 14, 15, 16, 17]
ID_ARM_R = [21, 22, 23, 24, 25, 26, 27]

# ── 手指映射：ROS2 手指索引 → 关节名 ──
HAND_L_MAP = {
    1: "left_little_1_joint",
    2: "left_ring_1_joint",
    3: "left_middle_1_joint",
    4: "left_index_1_joint",
    5: "left_thumb_2_joint",
    6: "left_thumb_1_joint",
}

HAND_R_MAP = {
    1: "right_little_1_joint",
    2: "right_ring_1_joint",
    3: "right_middle_1_joint",
    4: "right_index_1_joint",
    5: "right_thumb_2_joint",
    6: "right_thumb_1_joint",
}

# ── 归位位姿 ──
ARM_HOME = [
    -0.152, 0.068, 0.135, -1.155, 0.124, -0.361, -0.006,  # 左臂 7 关节
    -0.291, -0.003, -0.136, -1.155, -0.124, -0.361, 0.194,  # 右臂 7 关节
]

HAND_OPEN = [1.0, 1.0, 1.0, 1.0, 1.0, 0.0]
HAND_CLOSE = [0.3, 0.3, 0.3, 0.3, 0.3, 0.0]

# ── 电机默认参数 ──
DEFAULT_ARM_SPEED = 0.5
DEFAULT_ARM_CURRENT = 5.0
DEFAULT_RESET_SPEED = 0.2
DEFAULT_RESET_CURRENT = 5.0

# ── ROS2 话题名 ──
TOPIC_ARM_CMD = "/arm/cmd_pos"
TOPIC_HEAD_CMD = "/head/cmd_pos"
TOPIC_LEFT_HAND_CMD = "/inspire_hand/ctrl/left_hand"
TOPIC_RIGHT_HAND_CMD = "/inspire_hand/ctrl/right_hand"
TOPIC_ARM_STATUS = "/arm/status"
TOPIC_LEFT_HAND_STATUS = "/inspire_hand/state/left_hand"
TOPIC_RIGHT_HAND_STATUS = "/inspire_hand/state/right_hand"
