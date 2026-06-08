#!/usr/bin/env python3
"""ROS2 Deploy Bridge for LeRobot + TienKung robot.

Bridges between LeRobot (Python 3.12, ZMQ) and the robot backend via ROS2 DDS.

Runs on Python 3.10 (system) with rclpy for ROS2 communication.

Usage:
  # Via --config (recommended, from TienKungRobot._start_bridge()):
  python3 ros2_deploy_bridge.py --config '{"zmq_cmd_port":5559, ...}'

  # Legacy standalone mode (deprecated):
  python3 ros2_deploy_bridge.py --zmq_cmd_port 5559 --zmq_status_port 5560

ZMQ Internal Ports (LeRobot ↔ Bridge2):
  5559: LeRobot PUB → Bridge2 SUB (actions)
  5560: Bridge2 PUB → LeRobot SUB (status)
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import subprocess
import threading
import time
from typing import Any

import numpy as np
import zmq

logger = logging.getLogger("ros2_deploy_bridge")


# ── Inspire hand clip logic ──────────────────────────────────────────────────
# IMPORTANT: This logic must match hand_utils.inspire_clip_position() in the
# plugin package (lerobot_robot_tienkung/hand_utils.py). If you change it here,
# change it there too. The bridge cannot import the plugin (Python 3.10 vs 3.12).

def inspire_clip_position(position: list) -> list:
    """Inspire hand clip: clip [0,1], subtract 0.2 if < 0.9, round to 1 decimal."""
    position = [np.clip(float(pos), 0.0, 1.0) for pos in position]
    position = [pos - 0.2 if pos < 0.9 else pos for pos in position]
    return [round(pos, 1) for pos in position]


# ── Default config values (used when --config is not provided) ────────────────
_DEFAULT_CFG = {
    "zmq_cmd_port": 5559,
    "zmq_status_port": 5560,
    "ros_namespace": "",
    "cmd_namespace": "",
    "left_arm_motor_ids": [11, 12, 13, 14, 15, 16, 17],
    "right_arm_motor_ids": [21, 22, 23, 24, 25, 26, 27],
    "arm_speed": 0.5,
    "arm_current": 5.0,
    "hand_type": "inspire",
    "hand_open_position": [1.0, 1.0, 1.0, 1.0, 1.0, 0.0],
    "topic_arm_cmd": "/arm/cmd_pos",
    "topic_head_cmd": "/head/cmd_pos",
    "topic_left_hand_cmd": "/inspire_hand/ctrl/left_hand",
    "topic_right_hand_cmd": "/inspire_hand/ctrl/right_hand",
    "topic_arm_status": "/arm/status",
    "topic_left_hand_status": "/inspire_hand/state/left_hand",
    "topic_right_hand_status": "/inspire_hand/state/right_hand",
}


class ZMQInternalBridge:
    """ZMQ sockets for communication with LeRobot process.

    Bridge2 binds (server), LeRobot connects (client).
    """

    def __init__(self, cmd_port: int, status_port: int):
        self.context = zmq.Context()
        # SUB: receive actions from LeRobot
        self.cmd_socket = self.context.socket(zmq.SUB)
        self.cmd_socket.bind(f"tcp://*:{cmd_port}")
        self.cmd_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.cmd_socket.setsockopt(zmq.RCVHWM, 1)

        # PUB: forward status to LeRobot
        self.status_socket = self.context.socket(zmq.PUB)
        self.status_socket.bind(f"tcp://*:{status_port}")
        self.status_socket.setsockopt(zmq.SNDHWM, 1)

        logger.info("ZMQ internal bridge: cmd=%d, status=%d", cmd_port, status_port)

    def recv_action(self, timeout_ms: int = 100) -> dict | None:
        try:
            msg = self.cmd_socket.recv_json(flags=zmq.NOBLOCK)
            return msg
        except zmq.Again:
            return None

    def send_status(self, status: dict) -> None:
        try:
            self.status_socket.send_json(status, flags=zmq.NOBLOCK)
        except zmq.Again:
            logger.debug("Status send dropped: ZMQ send buffer full (SNDHWM=1)")

    def close(self) -> None:
        self.cmd_socket.close()
        self.status_socket.close()
        self.context.term()


class RealRobotBridge:
    """ROS2 DDS ↔ TienKung hardware.

    Subscribes to robot status topics, publishes command topics.
    Translates between ROS2 messages and the ZMQ internal format.

    All hardware constants (motor IDs, speeds, topic names, hand type)
    are read from the cfg dict passed at init time.
    """

    def __init__(self, zmq_bridge: ZMQInternalBridge, cfg: dict):
        self.zmq_bridge = zmq_bridge
        self._cfg = cfg

        # Extract commonly-used config values
        self._left_arm_motor_ids = cfg["left_arm_motor_ids"]
        self._right_arm_motor_ids = cfg["right_arm_motor_ids"]
        self._arm_speed = cfg.get("arm_speed", 0.5)
        self._arm_current = cfg.get("arm_current", 5.0)
        self._hand_type = cfg.get("hand_type", "inspire")

        ros_namespace = cfg.get("ros_namespace", "").rstrip("/")
        cmd_namespace = cfg.get("cmd_namespace", "").rstrip("/") if cfg.get("cmd_namespace") else ""

        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from sensor_msgs.msg import JointState

        try:
            from bodyctrl_msgs.msg import CmdSetMotorPosition, MotorStatusMsg, SetMotorPosition
            self._bodyctrl_available = True
        except ImportError:
            CmdSetMotorPosition = JointState
            MotorStatusMsg = JointState
            SetMotorPosition = JointState
            self._bodyctrl_available = False
            logger.warning("bodyctrl_msgs not available, using JointState fallback")

        self._CmdSetMotorPosition = CmdSetMotorPosition
        self._MotorStatusMsg = MotorStatusMsg
        self._SetMotorPosition = SetMotorPosition
        self._JointState = JointState

        if not rclpy.ok():
            rclpy.init()

        self._node = Node("ros2_deploy_bridge")

        # State caches
        n_arm = len(self._left_arm_motor_ids)
        n_hand = len(cfg.get("hand_open_position", [0.0] * 6))
        self._left_arm_jpos = [0.0] * n_arm
        self._right_arm_jpos = [0.0] * n_arm
        self._left_hand_pos = [0.0] * n_hand
        self._right_hand_pos = [0.0] * n_hand
        self._state_lock = threading.Lock()

        # Publishers — topic names from config
        arm_cmd_topic = f"{cmd_namespace}{cfg['topic_arm_cmd']}" if cmd_namespace else cfg["topic_arm_cmd"]
        self._arm_cmd_pub = self._node.create_publisher(CmdSetMotorPosition, arm_cmd_topic, 10)

        left_hand_topic = f"{cmd_namespace}{cfg['topic_left_hand_cmd']}" if cmd_namespace else cfg["topic_left_hand_cmd"]
        self._left_hand_pub = self._node.create_publisher(JointState, left_hand_topic, 10)

        right_hand_topic = f"{cmd_namespace}{cfg['topic_right_hand_cmd']}" if cmd_namespace else cfg["topic_right_hand_cmd"]
        self._right_hand_pub = self._node.create_publisher(JointState, right_hand_topic, 10)

        # Subscribers — topic names from config
        self._node.create_subscription(MotorStatusMsg, f"{ros_namespace}{cfg['topic_arm_status']}", self._arm_callback, 10)
        self._node.create_subscription(JointState, f"{ros_namespace}{cfg['topic_left_hand_status']}", self._left_hand_callback, 10)
        self._node.create_subscription(JointState, f"{ros_namespace}{cfg['topic_right_hand_status']}", self._right_hand_callback, 10)

        # Start executor
        self._executor = MultiThreadedExecutor(num_threads=3)
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(target=self._executor.spin, daemon=True, name="ros2_executor")
        self._executor_thread.start()

        # Action forwarding thread
        self._running = True
        self._action_thread = threading.Thread(target=self._action_loop, daemon=True, name="action_forward")
        self._action_thread.start()

        logger.info("Real robot bridge started (ns=%s, cmd_ns=%s, hand=%s, motors_L=%s, motors_R=%s)",
                    ros_namespace, cmd_namespace, self._hand_type,
                    self._left_arm_motor_ids, self._right_arm_motor_ids)

    def _arm_callback(self, msg: Any) -> None:
        if self._bodyctrl_available:
            tmp = [val.pos for val in msg.status]
        else:
            tmp = list(msg.position) if len(msg.position) > 0 else []

        if len(tmp) >= len(self._left_arm_motor_ids) + len(self._right_arm_motor_ids):
            n = len(self._left_arm_motor_ids)
            with self._state_lock:
                self._left_arm_jpos[:] = tmp[:n]
                self._right_arm_jpos[:] = tmp[n:n + len(self._right_arm_motor_ids)]
            self._publish_status()

    def _left_hand_callback(self, msg: Any) -> None:
        if len(msg.position) >= len(self._left_hand_pos):
            with self._state_lock:
                self._left_hand_pos[:] = list(msg.position)[:len(self._left_hand_pos)]
            self._publish_status()

    def _right_hand_callback(self, msg: Any) -> None:
        if len(msg.position) >= len(self._right_hand_pos):
            with self._state_lock:
                self._right_hand_pos[:] = list(msg.position)[:len(self._right_hand_pos)]
            self._publish_status()

    def _publish_status(self) -> None:
        with self._state_lock:
            status = {
                "left_arm": list(self._left_arm_jpos),
                "left_hand": list(self._left_hand_pos),
                "right_arm": list(self._right_arm_jpos),
                "right_hand": list(self._right_hand_pos),
                "ts": time.time(),
            }
        self.zmq_bridge.send_status(status)

    def _action_loop(self) -> None:
        while self._running:
            action = self.zmq_bridge.recv_action(timeout_ms=50)
            if action is not None:
                self._publish_arm_command(action)
                self._publish_hand_command("left", action.get("left_hand", []))
                self._publish_hand_command("right", action.get("right_hand", []))

    def _publish_arm_command(self, action: dict) -> None:
        left_arm = action.get("left_arm", [])
        right_arm = action.get("right_arm", [])
        if not left_arm and not right_arm:
            return

        # Validate arm dimensions
        n_left = len(self._left_arm_motor_ids)
        n_right = len(self._right_arm_motor_ids)
        if len(left_arm) != n_left or len(right_arm) != n_right:
            logger.warning(
                "Arm command dimension mismatch: left_arm=%d (expect %d), right_arm=%d (expect %d). Skipping.",
                len(left_arm), n_left, len(right_arm), n_right,
            )
            return

        target_joint = left_arm + right_arm
        msg = self._CmdSetMotorPosition()

        if self._bodyctrl_available:
            from std_msgs.msg import Header
            msg.header = Header()
            msg.header.stamp = self._node.get_clock().now().to_msg()

            for idx, val in enumerate(target_joint):
                cmd = self._SetMotorPosition()
                if idx < n_left:
                    cmd.name = self._left_arm_motor_ids[idx]
                else:
                    cmd.name = self._right_arm_motor_ids[idx - n_left]
                cmd.pos = float(val)
                cmd.spd = self._arm_speed
                cmd.cur = self._arm_current
                msg.cmds.append(cmd)
        else:
            # JointState fallback: populate name and position fields
            msg.name = [str(m) for m in self._left_arm_motor_ids + self._right_arm_motor_ids]
            msg.position = [float(v) for v in target_joint]

        self._arm_cmd_pub.publish(msg)

    def _publish_hand_command(self, hand_side: str, position: list) -> None:
        if not position:
            return

        # Apply hand-type-specific clip logic
        if self._hand_type == "inspire":
            position = inspire_clip_position(position)
        # Future: elif self._hand_type == "brainco": position = brainco_clip_position(position)

        msg = self._JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = [str(i) for i in range(1, len(position) + 1)]
        msg.position = [float(p) for p in position]

        if hand_side == "left":
            self._left_hand_pub.publish(msg)
        else:
            self._right_hand_pub.publish(msg)

    def stop(self) -> None:
        self._running = False
        if self._action_thread.is_alive():
            self._action_thread.join(timeout=2.0)
        if self._executor is not None:
            self._executor.shutdown()
        if self._executor_thread is not None and self._executor_thread.is_alive():
            self._executor_thread.join(timeout=3.0)
        if self._node is not None:
            self._node.destroy_node()
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()


def _is_alive(pid: int) -> bool:
    """Check whether a process is still running."""
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True  # process exists but we lack permission to signal it


def kill_existing_bridge() -> None:
    """Find and kill any already-running ros2_deploy_bridge processes.

    Sends SIGTERM first for graceful shutdown, then SIGKILL after 3 s if
    the process is still alive.  Waits an extra 0.5 s for ZMQ ports to be
    released before returning.
    """
    current_pid = os.getpid()

    try:
        result = subprocess.run(
            ["pgrep", "-f", "ros2_deploy_bridge"],
            capture_output=True, text=True, timeout=5,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return  # pgrep unavailable or timed out — skip check

    if result.returncode != 0:
        return  # no existing processes found

    pids = []
    for line in result.stdout.strip().splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            pid = int(line)
            if pid != current_pid:
                pids.append(pid)
        except ValueError:
            continue

    if not pids:
        return

    logger.info("Found existing bridge processes (PIDs: %s), terminating ...", pids)

    # --- graceful SIGTERM ---
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass

    # wait up to 3 s for them to exit
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        alive = [p for p in pids if _is_alive(p)]
        if not alive:
            break
        time.sleep(0.1)
    else:
        # --- force SIGKILL for stragglers ---
        for pid in alive:  # noqa: F821
            logger.warning("Force killing bridge process %d", pid)
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        time.sleep(0.1)

    # give the OS a moment to release ZMQ sockets
    time.sleep(0.5)
    logger.info("Previous bridge instances terminated.")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ROS2 Deploy Bridge for LeRobot + TienKung")
    parser.add_argument("--config", type=str, default=None,
                        help="JSON config string from TienKungRobotConfig.to_bridge_config() "
                             "(recommended). If omitted, legacy CLI args or defaults are used.")
    # Legacy args (deprecated, kept for standalone testing)
    parser.add_argument("--zmq_cmd_port", type=int, default=None,
                        help="(deprecated) ZMQ port for receiving actions from LeRobot (bind SUB)")
    parser.add_argument("--zmq_status_port", type=int, default=None,
                        help="(deprecated) ZMQ port for sending status to LeRobot (bind PUB)")
    parser.add_argument("--ros_namespace", type=str, default=None,
                        help="(deprecated) ROS2 namespace for status topics (subscribe)")
    parser.add_argument("--cmd_namespace", type=str, default=None,
                        help="(deprecated) ROS2 namespace for command topics (publish)")
    return parser.parse_args()


def main():
    args = _parse_args()

    # Build cfg dict: start from defaults, overlay --config JSON, then legacy CLI overrides
    cfg = dict(_DEFAULT_CFG)

    if args.config:
        try:
            cfg_overrides = json.loads(args.config)
            cfg.update(cfg_overrides)
        except json.JSONDecodeError as e:
            logger.error("Failed to parse --config JSON: %s", e)
            return

    # Legacy CLI arg overrides (for standalone testing without --config)
    if args.zmq_cmd_port is not None:
        cfg["zmq_cmd_port"] = args.zmq_cmd_port
    if args.zmq_status_port is not None:
        cfg["zmq_status_port"] = args.zmq_status_port
    if args.ros_namespace is not None:
        cfg["ros_namespace"] = args.ros_namespace
    if args.cmd_namespace is not None:
        cfg["cmd_namespace"] = args.cmd_namespace

    # Configure logging early so kill_existing_bridge() messages are visible
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )

    # --- kill any existing bridge before starting ---
    kill_existing_bridge()

    zmq_bridge = ZMQInternalBridge(cfg["zmq_cmd_port"], cfg["zmq_status_port"])
    robot_bridge = RealRobotBridge(zmq_bridge, cfg)

    stop_event = threading.Event()

    def signal_handler(sig, frame):
        logger.info("Received signal %s, shutting down...", sig)
        stop_event.set()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    logger.info("Bridge running (ros_ns=%s, cmd_ns=%s). Press Ctrl+C to stop.",
                cfg.get("ros_namespace", ""), cfg.get("cmd_namespace", ""))
    try:
        stop_event.wait()
    except KeyboardInterrupt:
        pass

    logger.info("Shutting down...")
    robot_bridge.stop()
    zmq_bridge.close()
    logger.info("Bridge stopped.")


if __name__ == "__main__":
    main()
