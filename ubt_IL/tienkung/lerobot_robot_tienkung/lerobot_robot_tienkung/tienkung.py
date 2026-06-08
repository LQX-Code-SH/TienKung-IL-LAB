"""TienKung dual-arm robot with Inspire dexterous hands.

Communicates with the robot backend via ZMQ through ros2_deploy_bridge.py,
which bridges to the robot through ROS2 DDS.

Joint layout is configured via TienKungRobotConfig.all_joints (default: arms_then_hands).
"""

from __future__ import annotations

import json
import logging
import subprocess
import threading
import time

import zmq

from lerobot.cameras import make_cameras_from_configs
from lerobot.robots.robot import Robot
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .config_tienkung import TienKungRobotConfig
from .hand_utils import clip_hand_value

logger = logging.getLogger(__name__)

# Path where _start_bridge() writes the config JSON for external scripts to read.
_BRIDGE_CONFIG_PATH = "/tmp/tienkung_bridge_config.json"


class TienKungRobot(Robot):
    config_class = TienKungRobotConfig
    name = "tienkung"

    def __init__(self, config: TienKungRobotConfig):
        super().__init__(config)
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)

        # Joint definitions from config (replace module-level constants)
        self._left_arm_joints = config.left_arm_joints
        self._right_arm_joints = config.right_arm_joints
        self._left_hand_joints = config.left_hand_joints
        self._right_hand_joints = config.right_hand_joints
        self._all_joints = config.all_joints

        # ZMQ state (populated in connect)
        self._zmq_context: zmq.Context | None = None
        self._cmd_socket: zmq.Socket | None = None
        self._status_socket: zmq.Socket | None = None
        self._bridge_process: subprocess.Popen | None = None

        # Thread-safe state caches
        self._state_lock = threading.Lock()
        self._left_arm_jpos: list[float] = [0.0] * len(config.left_arm_joints)
        self._right_arm_jpos: list[float] = [0.0] * len(config.right_arm_joints)
        self._left_hand_pos: list[float] = [0.0] * len(config.left_hand_joints)
        self._right_hand_pos: list[float] = [0.0] * len(config.right_hand_joints)
        self._state_ready = threading.Event()

        # Status receive thread
        self._recv_thread: threading.Thread | None = None
        self._running = False

        self._connected = False

    @property
    def observation_features(self) -> dict[str, type | tuple]:
        motors_ft = {name: float for name in self._all_joints}
        camera_ft = {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3)
            for cam in self.cameras
        }
        return {**motors_ft, **camera_ft}

    @property
    def action_features(self) -> dict[str, type]:
        return {name: float for name in self._all_joints}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        # Start Bridge2 subprocess if enabled
        if self.config.bridge_enabled:
            self._start_bridge()

        # Create ZMQ context and sockets
        self._zmq_context = zmq.Context()
        host = self.config.zmq_host

        # PUB: send actions to Bridge2
        self._cmd_socket = self._zmq_context.socket(zmq.PUB)
        self._cmd_socket.connect(f"tcp://{host}:{self.config.zmq_cmd_port}")
        self._cmd_socket.setsockopt(zmq.SNDHWM, 1)

        # SUB: receive status from Bridge2
        self._status_socket = self._zmq_context.socket(zmq.SUB)
        self._status_socket.connect(f"tcp://{host}:{self.config.zmq_status_port}")
        self._status_socket.setsockopt(zmq.RCVHWM, 1)
        self._status_socket.setsockopt_string(zmq.SUBSCRIBE, "")

        # Connect cameras (they create their own ZMQ SUB sockets for images)
        for cam in self.cameras.values():
            cam.connect()

        # Start status receive thread
        self._running = True
        self._recv_thread = threading.Thread(
            target=self._status_recv_loop, daemon=True, name="tienkung_status_recv"
        )
        self._recv_thread.start()

        # Wait for first status message
        logger.info("Waiting for Bridge2 status messages...")
        warmup_start = time.time()
        warmup_timeout = 10.0
        while time.time() - warmup_start < warmup_timeout:
            if self._state_ready.is_set():
                break
            time.sleep(0.1)

        if not self._state_ready.is_set():
            logger.warning("Timed out waiting for Bridge2 status messages.")

        self._connected = True
        logger.info("TienKungRobot connected.")

    def _start_bridge(self) -> None:
        # Stop any existing Bridge2 process first (avoid conflicts from auto-start)
        subprocess.run(["pkill", "-f", "ros2_deploy_bridge.py"], check=False)
        time.sleep(0.5)

        config_json = json.dumps(self.config.to_bridge_config())
        cmd = [
            "/usr/bin/python3", self.config.bridge_script,
            "--config", config_json,
        ]

        logger.info("Starting Bridge2: %s --config <json>", self.config.bridge_script)
        self._bridge_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

        # Write config to a known location for external scripts (replay.py, reset.py)
        try:
            with open(_BRIDGE_CONFIG_PATH, "w") as f:
                f.write(config_json)
        except OSError:
            logger.warning("Failed to write bridge config to %s", _BRIDGE_CONFIG_PATH)

        # Give bridge time to bind ZMQ ports
        time.sleep(1.0)

    def _status_recv_loop(self) -> None:
        while self._running:
            try:
                msg = self._status_socket.recv_json(flags=zmq.NOBLOCK)
                self._process_status(msg)
            except zmq.Again:
                time.sleep(0.001)
            except Exception as e:
                logger.error("Status receive error (non-fatal): %s", e)
                time.sleep(0.01)

    def _process_status(self, data: dict) -> None:
        left_arm = data.get("left_arm", [])
        left_hand = data.get("left_hand", [])
        right_arm = data.get("right_arm", [])
        right_hand = data.get("right_hand", [])

        if len(left_arm) >= len(self._left_arm_joints) and len(right_arm) >= len(self._right_arm_joints):
            with self._state_lock:
                self._left_arm_jpos[:] = left_arm[:len(self._left_arm_joints)]
                self._right_arm_jpos[:] = right_arm[:len(self._right_arm_joints)]
        if len(left_hand) >= len(self._left_hand_joints):
            with self._state_lock:
                self._left_hand_pos[:] = left_hand[:len(self._left_hand_joints)]
        if len(right_hand) >= len(self._right_hand_joints):
            with self._state_lock:
                self._right_hand_pos[:] = right_hand[:len(self._right_hand_joints)]

        self._state_ready.set()

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        obs: RobotObservation = {}

        with self._state_lock:
            for i, name in enumerate(self._left_arm_joints):
                obs[name] = self._left_arm_jpos[i]
            for i, name in enumerate(self._left_hand_joints):
                obs[name] = self._left_hand_pos[i]
            for i, name in enumerate(self._right_arm_joints):
                obs[name] = self._right_arm_jpos[i]
            for i, name in enumerate(self._right_hand_joints):
                obs[name] = self._right_hand_pos[i]

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            obs[cam_key] = cam.read_latest()

        return obs

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        # Extract action values by joint group
        left_arm = [action[name] for name in self._left_arm_joints]
        left_hand = [action[name] for name in self._left_hand_joints]
        right_arm = [action[name] for name in self._right_arm_joints]
        right_hand = [action[name] for name in self._right_hand_joints]

        # Apply safety clipping if configured
        if self.config.max_relative_target is not None:
            with self._state_lock:
                current_left = list(self._left_arm_jpos)
                current_right = list(self._right_arm_jpos)

            left_arm = self._clip_relative(left_arm, current_left, self.config.max_relative_target)
            right_arm = self._clip_relative(right_arm, current_right, self.config.max_relative_target)

        # Publish action via ZMQ
        action_msg = {
            "left_arm": left_arm,
            "left_hand": left_hand,
            "right_arm": right_arm,
            "right_hand": right_hand,
            "ts": time.time(),
        }
        try:
            self._cmd_socket.send_json(action_msg, flags=zmq.NOBLOCK)
        except zmq.Again:
            logger.warning("Action send dropped: ZMQ send buffer full (SNDHWM=1)")

        # Return the actual action sent (after clipping).
        # Apply hand clip to returned values so they match what Bridge2
        # will send to the robot.
        sent_action: RobotAction = {}
        for i, name in enumerate(self._left_arm_joints):
            sent_action[name] = left_arm[i]
        for i, name in enumerate(self._left_hand_joints):
            sent_action[name] = clip_hand_value(left_hand[i], self.config.hand_type)
        for i, name in enumerate(self._right_arm_joints):
            sent_action[name] = right_arm[i]
        for i, name in enumerate(self._right_hand_joints):
            sent_action[name] = clip_hand_value(right_hand[i], self.config.hand_type)
        return sent_action

    @staticmethod
    def _clip_relative(
        goal: list[float], current: list[float], max_diff: float
    ) -> list[float]:
        import numpy as np

        result = []
        for g, c in zip(goal, current):
            diff = np.clip(g - c, -max_diff, max_diff)
            result.append(c + diff)
        return result

    @check_if_not_connected
    def disconnect(self) -> None:
        # Optionally return to home position
        if self.config.disable_torque_on_disconnect and self._state_ready.is_set():
            logger.info("Returning to home position...")
            home_action = {
                "left_arm": self.config.home_position[:len(self._left_arm_joints)],
                "left_hand": list(self.config.hand_open_position),
                "right_arm": self.config.home_position[len(self._left_arm_joints):len(self._left_arm_joints) + len(self._right_arm_joints)],
                "right_hand": list(self.config.hand_open_position),
                "ts": time.time(),
            }
            try:
                self._cmd_socket.send_json(home_action, flags=zmq.NOBLOCK)
            except zmq.Again:
                logger.warning("Home action send dropped: ZMQ send buffer full")
            time.sleep(1.0)

        # Stop receive thread
        self._running = False
        if self._recv_thread is not None and self._recv_thread.is_alive():
            self._recv_thread.join(timeout=3.0)
            self._recv_thread = None

        # Close ZMQ sockets
        if self._cmd_socket is not None:
            self._cmd_socket.close()
            self._cmd_socket = None
        if self._status_socket is not None:
            self._status_socket.close()
            self._status_socket = None
        if self._zmq_context is not None:
            self._zmq_context.term()
            self._zmq_context = None

        # Terminate Bridge2 subprocess
        if self._bridge_process is not None:
            logger.info("Stopping Bridge2 subprocess...")
            self._bridge_process.terminate()
            try:
                self._bridge_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._bridge_process.kill()
                self._bridge_process.wait(timeout=2.0)
            self._bridge_process = None

        # Disconnect cameras
        for cam in self.cameras.values():
            cam.disconnect()

        self._connected = False
        logger.info("TienKungRobot disconnected.")
