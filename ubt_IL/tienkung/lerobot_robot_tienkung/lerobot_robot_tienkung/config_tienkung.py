from __future__ import annotations

from dataclasses import dataclass, field

from lerobot.cameras.configs import CameraConfig
from lerobot.robots.config import RobotConfig

from .constants import (
    ARM_HOME,
    DEFAULT_ARM_CURRENT,
    DEFAULT_ARM_SPEED,
    DEFAULT_RESET_CURRENT,
    DEFAULT_RESET_SPEED,
    HAND_OPEN,
    ID_ARM_L,
    ID_ARM_R,
    TOPIC_ARM_CMD,
    TOPIC_ARM_STATUS,
    TOPIC_HEAD_CMD,
    TOPIC_LEFT_HAND_CMD,
    TOPIC_LEFT_HAND_STATUS,
    TOPIC_RIGHT_HAND_CMD,
    TOPIC_RIGHT_HAND_STATUS,
)


def _make_joints(prefix: str, n: int) -> list[str]:
    """Generate joint feature names: prefix_j1.pos, prefix_j2.pos, ..."""
    return [f"{prefix}_j{i}.pos" for i in range(1, n + 1)]


@RobotConfig.register_subclass("tienkung")
@dataclass
class TienKungRobotConfig(RobotConfig):
    # ZMQ configuration (LeRobot ↔ Bridge2 internal communication)
    zmq_host: str = "127.0.0.1"
    zmq_cmd_port: int = 5559       # LeRobot PUB → Bridge2 SUB
    zmq_status_port: int = 5560    # Bridge2 PUB → LeRobot SUB
    bridge_enabled: bool = True     # Auto-start Bridge2 subprocess
    bridge_script: str = "/ubt_IL/tienkung/ros2_deploy_bridge.py"  # Path to Bridge2 script (bind-mounted)

    # ROS2 topics (real robot defaults)
    ros_namespace: str = ""
    cmd_namespace: str = ""

    # Hand type: "inspire" (currently the only supported type)
    hand_type: str = "inspire"

    # Joint group definitions (used for ZMQ message grouping)
    left_arm_joints: list[str] = field(default_factory=lambda: _make_joints("left_arm", 7))
    right_arm_joints: list[str] = field(default_factory=lambda: _make_joints("right_arm", 7))
    left_hand_joints: list[str] = field(default_factory=lambda: _make_joints("left_hand", 6))
    right_hand_joints: list[str] = field(default_factory=lambda: _make_joints("right_hand", 6))

    # Full joint ordering (determines model action/observation tensor dimension mapping).
    # Must be a permutation of the union of the 4 joint groups above.
    # 根据顺序排列的关节配置修改
    all_joints: list[str] = field(default_factory=lambda: (
        _make_joints("left_arm", 7) + _make_joints("right_arm", 7)
        + _make_joints("left_hand", 6) + _make_joints("right_hand", 6)
    ))

    # Motor IDs (1:1 mapping with arm joint groups, used by Bridge2 to address hardware)
    left_arm_motor_ids: list[int] = field(default_factory=lambda: list(ID_ARM_L))
    right_arm_motor_ids: list[int] = field(default_factory=lambda: list(ID_ARM_R))

    # Arm motion parameters
    arm_speed: float = DEFAULT_ARM_SPEED
    arm_current: float = DEFAULT_ARM_CURRENT
    reset_speed: float = DEFAULT_RESET_SPEED
    reset_current: float = DEFAULT_RESET_CURRENT

    # Hand open position (after Inspire clip, these values open the hand)
    hand_open_position: list[float] = field(default_factory=lambda: list(HAND_OPEN))

    # ROS2 topic names (command side / publish)
    topic_arm_cmd: str = TOPIC_ARM_CMD
    topic_head_cmd: str = TOPIC_HEAD_CMD
    topic_left_hand_cmd: str = TOPIC_LEFT_HAND_CMD
    topic_right_hand_cmd: str = TOPIC_RIGHT_HAND_CMD

    # ROS2 topic names (status side / subscribe)
    topic_arm_status: str = TOPIC_ARM_STATUS
    topic_left_hand_status: str = TOPIC_LEFT_HAND_STATUS
    topic_right_hand_status: str = TOPIC_RIGHT_HAND_STATUS

    # Safety
    max_relative_target: float | None = None
    disable_torque_on_disconnect: bool = True

    # Home position (14-dim: left arm 7 + right arm 7)
    home_position: list[float] = field(default_factory=lambda: list(ARM_HOME))

    # Cameras (keyed by name matching policy's expected image key, e.g. "camera_head")
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    def __post_init__(self):
        # Validate all_joints is a permutation of the union of 4 joint groups
        group_set = set(self.left_arm_joints + self.right_arm_joints
                        + self.left_hand_joints + self.right_hand_joints)
        all_set = set(self.all_joints)
        if group_set != all_set:
            raise ValueError(
                f"all_joints must be a permutation of the union of 4 joint groups. "
                f"Missing: {group_set - all_set}, Extra: {all_set - group_set}"
            )
        if len(self.all_joints) != len(all_set):
            raise ValueError(
                f"all_joints has duplicates: {len(self.all_joints)} items vs {len(all_set)} unique"
            )

        # Validate hand_type
        if self.hand_type not in ("inspire",):
            raise ValueError(
                f"hand_type must be 'inspire', got {self.hand_type!r}"
            )

        # Validate motor ID counts match joint counts
        if len(self.left_arm_motor_ids) != len(self.left_arm_joints):
            raise ValueError(
                f"left_arm_motor_ids count ({len(self.left_arm_motor_ids)}) must match "
                f"left_arm_joints count ({len(self.left_arm_joints)})"
            )
        if len(self.right_arm_motor_ids) != len(self.right_arm_joints):
            raise ValueError(
                f"right_arm_motor_ids count ({len(self.right_arm_motor_ids)}) must match "
                f"right_arm_joints count ({len(self.right_arm_joints)})"
            )

        # Validate hand_open_position matches hand joint count
        if len(self.hand_open_position) != len(self.left_hand_joints):
            raise ValueError(
                f"hand_open_position count ({len(self.hand_open_position)}) must match "
                f"left_hand_joints count ({len(self.left_hand_joints)})"
            )

        # Validate home_position dimension
        expected_home_dim = len(self.left_arm_joints) + len(self.right_arm_joints)
        if len(self.home_position) != expected_home_dim:
            raise ValueError(
                f"home_position count ({len(self.home_position)}) must match "
                f"left_arm + right_arm joint count ({expected_home_dim})"
            )

    def to_bridge_config(self) -> dict:
        """Serialize config fields needed by ros2_deploy_bridge.py (system Python 3.10).

        The bridge runs outside the LeRobot venv and cannot import this class.
        This method produces a JSON-serializable dict passed via --config CLI arg.
        """
        return {
            "zmq_cmd_port": self.zmq_cmd_port,
            "zmq_status_port": self.zmq_status_port,
            "ros_namespace": self.ros_namespace,
            "cmd_namespace": self.cmd_namespace,
            "left_arm_motor_ids": self.left_arm_motor_ids,
            "right_arm_motor_ids": self.right_arm_motor_ids,
            "arm_speed": self.arm_speed,
            "arm_current": self.arm_current,
            "hand_type": self.hand_type,
            "hand_open_position": self.hand_open_position,
            "topic_arm_cmd": self.topic_arm_cmd,
            "topic_head_cmd": self.topic_head_cmd,
            "topic_left_hand_cmd": self.topic_left_hand_cmd,
            "topic_right_hand_cmd": self.topic_right_hand_cmd,
            "topic_arm_status": self.topic_arm_status,
            "topic_left_hand_status": self.topic_left_hand_status,
            "topic_right_hand_status": self.topic_right_hand_status,
        }
