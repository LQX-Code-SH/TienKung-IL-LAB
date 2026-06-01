#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float32
try:
    from bodyctrl_msgs.msg import MotorStatusMsg, MotorStatus, CmdSetMotorPosition, CmdMotorCtrl
except ImportError:
    print("[ERROR] bodyctrl_msgs not found. Please source the correct workspace.")
    class MotorStatusMsg: pass
    class MotorStatus: pass
    class CmdSetMotorPosition: pass
    class CmdMotorCtrl: pass

import zmq
import yaml
import os
import subprocess
import threading
import time


def load_bridge_config():
    """Load bridge configuration from bridge_config.yaml next to this script."""
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bridge_config.yaml")
    with open(config_path) as f:
        return yaml.safe_load(f)


# Map config type strings to ROS2 message classes
MSG_TYPES = {
    "CmdSetMotorPosition": CmdSetMotorPosition,
    "CmdMotorCtrl": CmdMotorCtrl,
    "JointState": JointState,
    "Point": Point,
    "Bool": Bool,
    "MotorStatusMsg": MotorStatusMsg,
    "Float32": Float32,
    "Image": Image,
}


class TiangongRosBridge(Node):
    def __init__(self):
        super().__init__('tiangong_ros_bridge')

        # Load config
        self.cfg = load_bridge_config()
        zmq_cfg = self.cfg["zmq"]

        # ZMQ setup
        self.zmq_context = zmq.Context()

        # Command PUB
        self.cmd_socket = self.zmq_context.socket(zmq.PUB)
        self.cmd_socket.bind(f"tcp://*:{zmq_cfg['cmd_port']}")

        # Status SUB
        self.status_socket = self.zmq_context.socket(zmq.SUB)
        self.status_socket.connect(f"tcp://127.0.0.1:{zmq_cfg['status_port']}")
        self.status_socket.setsockopt(zmq.RCVHWM, 1)
        self.status_socket.setsockopt_string(zmq.SUBSCRIBE, "")

        # ID Definition
        self.ID_HEAD = [1, 2, 3]
        self.ID_ARM_L = [11, 12, 13, 14, 15, 16, 17]
        self.ID_ARM_R = [21, 22, 23, 24, 25, 26, 27]
        self.ID_WAIST = [31]
        self.ID_LEG_L = [51, 52, 53, 54, 55, 56]
        self.ID_LEG_R = [61, 62, 63, 64, 65, 66]

        # Reverse Mapping for Commands (ID to Name)
        self.ID_TO_NAME = {
            # Head
            1: "head_roll_joint", 2: "head_pitch_joint", 3: "head_yaw_joint",
            # Arms
            11: "shoulder_pitch_l_joint", 12: "shoulder_roll_l_joint", 13: "shoulder_yaw_l_joint",
            14: "elbow_pitch_l_joint", 15: "elbow_yaw_l_joint", 16: "wrist_pitch_l_joint", 17: "wrist_roll_l_joint",
            21: "shoulder_pitch_r_joint", 22: "shoulder_roll_r_joint", 23: "shoulder_yaw_r_joint",
            24: "elbow_pitch_r_joint", 25: "elbow_yaw_r_joint", 26: "wrist_pitch_r_joint", 27: "wrist_roll_r_joint",
            # Waist
            31: "body_yaw_joint",
            # Legs
            51: "hip_roll_l_joint", 52: "hip_pitch_l_joint", 53: "hip_yaw_l_joint",
            54: "knee_pitch_l_joint", 55: "ankle_pitch_l_joint", 56: "ankle_roll_l_joint",
            61: "hip_roll_r_joint", 62: "hip_pitch_r_joint", 63: "hip_yaw_r_joint",
            64: "knee_pitch_r_joint", 65: "ankle_pitch_r_joint", 66: "ankle_roll_r_joint"
        }

        self.NAME_TO_ID = {v: k for k, v in self.ID_TO_NAME.items()}

        self.HAND_L_MAP = {
            1: "left_little_1_joint",
            2: "left_ring_1_joint",
            3: "left_middle_1_joint",
            4: "left_index_1_joint",
            5: "left_thumb_2_joint",
            6: "left_thumb_1_joint"
        }
        self.HAND_R_MAP = {
            1: "right_little_1_joint",
            2: "right_ring_1_joint",
            3: "right_middle_1_joint",
            4: "right_index_1_joint",
            5: "right_thumb_2_joint",
            6: "right_thumb_1_joint"
        }

        # --- Dynamic subscriptions from config ---
        SUB_CALLBACKS = {
            "arm_cmd_pos":   self.cmd_pos_cb,
            "arm_cmd_ctrl":  self.cmd_ctrl_cb,
            "head_cmd_pos":  self.cmd_pos_cb,
            "head_cmd_ctrl": self.cmd_ctrl_cb,
            "leg_cmd_pos":   self.cmd_pos_cb,
            "leg_cmd_ctrl":  self.cmd_ctrl_cb,
            "waist_cmd_pos": self.cmd_pos_cb,
            "waist_cmd_ctrl":self.cmd_ctrl_cb,
            "hand_l_ctrl":   lambda m: self.hand_cb(m, "left"),
            "hand_r_ctrl":   lambda m: self.hand_cb(m, "right"),
            "apple_offset":  self.apple_cb,
            "cmd_reset":     self.cmd_reset_cb,
        }
        for key, spec in self.cfg["topics"]["sub"].items():
            msg_type = MSG_TYPES[spec["type"]]
            cb = SUB_CALLBACKS.get(key)
            if cb:
                self.create_subscription(msg_type, spec["topic"], cb, spec.get("qos", 1))

        # --- Dynamic publishers from config ---
        self.pubs = {}
        for key, spec in self.cfg["topics"]["pub"].items():
            # image_rgb and image_depth are handled by C++ bridge, skip here
            if key in ("image_rgb", "image_depth"):
                continue
            msg_type = MSG_TYPES[spec["type"]]
            self.pubs[key] = self.create_publisher(msg_type, spec["topic"], spec.get("qos", 10))

        # Threaded polling
        self._running = True
        self._poll_thread = threading.Thread(target=self._poll_loop)
        self._poll_thread.start()
        self.get_logger().info("Tiangong ROS 2 Bridge (Control Only) Started")

        # Start C++ Image Bridge (skip with env var DISABLE_CPP_IMAGE_BRIDGE=1)
        self.cpp_bridge_process = None
        if os.environ.get("DISABLE_CPP_IMAGE_BRIDGE", "0") != "1":
            self.start_cpp_bridge()
        else:
            self.get_logger().info("C++ Image Bridge disabled by environment variable.")

    def start_cpp_bridge(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        build_script = os.path.join(script_dir, "build_cpp_bridge.sh")
        executable = os.path.join(script_dir, "zmq_image_bridge")
        cpp_source = os.path.join(script_dir, "zmq_image_bridge.cpp")
        cmake_source = os.path.join(script_dir, "CMakeLists.txt")

        # Skip build if executable exists and is newer than source files
        need_build = True
        if os.path.isfile(executable) and os.access(executable, os.X_OK):
            exe_mtime = os.path.getmtime(executable)
            src_mtime = max(os.path.getmtime(cpp_source), os.path.getmtime(cmake_source))
            if exe_mtime > src_mtime:
                need_build = False
                self.get_logger().info("C++ Bridge binary is up-to-date, skipping build.")

        if need_build:
            self.get_logger().info(f"Building C++ Bridge: {build_script} ...")
            subprocess.run(["chmod", "+x", build_script], cwd=script_dir)
            res = subprocess.run([build_script], cwd=script_dir,
                                 capture_output=True, text=True)
            if res.returncode != 0:
                self.get_logger().error(
                    f"C++ Bridge build failed:\n{res.stderr}\n"
                    "Image publishing will be unavailable. "
                    "Set DISABLE_CPP_IMAGE_BRIDGE=1 to suppress."
                )
                return

        if not os.path.isfile(executable):
            self.get_logger().error("C++ Bridge executable not found. Image publishing unavailable.")
            return

        # Pass config values to C++ bridge via CLI args
        pub_cfg = self.cfg["topics"]["pub"]
        zmq_cfg = self.cfg["zmq"]
        args = [
            executable,
            "--zmq-port",    str(zmq_cfg["image_port"]),
            "--rgb-topic",   pub_cfg["image_rgb"]["topic"],
            "--depth-topic", pub_cfg["image_depth"]["topic"],
        ]
        self.get_logger().info(f"Starting C++ Bridge: {' '.join(args)}")
        try:
            self.cpp_bridge_process = subprocess.Popen(args, cwd=script_dir)
        except Exception as e:
            self.get_logger().error(f"Failed to start C++ Bridge: {e}")
            self.cpp_bridge_process = None

    def cmd_pos_cb(self, msg):
        current_action = {}
        for cmd in msg.cmds:
            joint_name = self.ID_TO_NAME.get(cmd.name)
            if joint_name:
                current_action[joint_name] = float(cmd.pos)
        self.cmd_socket.send_json(current_action)

    def cmd_ctrl_cb(self, msg):
        current_action = {}
        for cmd in msg.cmds:
            joint_name = self.ID_TO_NAME.get(cmd.name)
            if joint_name:
                current_action[joint_name] = float(cmd.pos)
        self.cmd_socket.send_json(current_action)

    def hand_cb(self, msg, side):
        current_action = {}
        map_target = self.HAND_L_MAP if side == "left" else self.HAND_R_MAP
        for i, name in enumerate(msg.name):
            try:
                id_val = int(name)
                joint_name = map_target.get(id_val)
                if joint_name:
                    current_action[joint_name] = float(msg.position[i])
            except: pass
        self.cmd_socket.send_json(current_action)

    def apple_cb(self, msg):
        self.cmd_socket.send_json({"apple_offset": [msg.x, msg.y]})

    def cmd_reset_cb(self, msg):
        if msg.data:
             self.cmd_socket.send_json({"reset": True})

    def _poll_loop(self):
        poller = zmq.Poller()
        poller.register(self.status_socket, zmq.POLLIN)

        while self._running:
            socks = dict(poller.poll(timeout=1))
            if self.status_socket in socks:
                try:
                    msg = self.status_socket.recv_json(flags=zmq.NOBLOCK)
                    self.publish_status(msg)
                    if "task_dist" in msg:
                        f = Float32()
                        f.data = float(msg["task_dist"])
                        self.pubs["task_dist"].publish(f)
                except Exception as e:
                    pass

    def publish_status(self, data):
        # Support both old list format and new dict format
        if isinstance(data, dict) and "joint_names" in data:
            pos_map = dict(zip(data["joint_names"], data["joint_pos"]))
            finger_percentages = data.get("finger_percentages", {})
            vel_map = dict(zip(data["joint_names"], data["joint_vel"])) if "joint_vel" in data else {}
        else:
            pos_map = data
            finger_percentages = {}
            vel_map = {}

        header = self.get_clock().now().to_msg()

        def create_motor_msg(id_range):
            m_msg = MotorStatusMsg()
            m_msg.header.stamp = header
            for name, id_val in self.NAME_TO_ID.items():
                if id_val in id_range and name in pos_map:
                    s = MotorStatus()
                    s.name = id_val
                    s.pos = float(pos_map[name])
                    m_msg.status.append(s)
            return m_msg

        def create_hand_msg(hand_map):
            h_msg = JointState()
            h_msg.header.stamp = header
            h_msg.name = []
            h_msg.position = []
            h_msg.velocity = []

            for i in range(1, 7):
                sim_name = hand_map.get(i)
                h_msg.name.append(str(i))

                pos_val = 0.0
                vel_val = 0.0

                if sim_name and sim_name in finger_percentages:
                    pos_val = float(finger_percentages[sim_name])
                    if sim_name in vel_map:
                        vel_val = float(vel_map[sim_name])

                h_msg.position.append(pos_val)
                h_msg.velocity.append(vel_val)
                h_msg.effort.append(0.0)

            return h_msg

        self.pubs["arm_status"].publish(create_motor_msg(self.ID_ARM_L + self.ID_ARM_R))
        self.pubs["head_status"].publish(create_motor_msg(self.ID_HEAD))
        self.pubs["leg_status"].publish(create_motor_msg(self.ID_LEG_L + self.ID_LEG_R))
        self.pubs["waist_status"].publish(create_motor_msg(self.ID_WAIST))

        self.pubs["hand_l_state"].publish(create_hand_msg(self.HAND_L_MAP))
        self.pubs["hand_r_state"].publish(create_hand_msg(self.HAND_R_MAP))

    def stop(self):
        self._running = False
        self._poll_thread.join()
        if self.cpp_bridge_process:
            self.get_logger().info("Stopping C++ Bridge...")
            self.cpp_bridge_process.terminate()
            try:
                self.cpp_bridge_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.cpp_bridge_process.kill()

if __name__ == '__main__':
    rclpy.init()
    node = TiangongRosBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.stop()
    node.destroy_node()
    rclpy.shutdown()
