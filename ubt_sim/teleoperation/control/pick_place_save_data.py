#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState, Image, CompressedImage,JointState
from bodyctrl_msgs.msg import MotorStatusMsg, MotorStatus, CmdSetMotorPosition,SetMotorPosition
from std_msgs.msg import Float32, Bool
import h5py
import cv2

import numpy as np
from typing import Optional
from time import sleep
import threading

# 获取脚本所在目录，确保路径一致性
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# 逆解相关
try:
  from ikpy.chain import Chain
  from ikpy.link import URDFLink
except ImportError:
  print("[ERROR] 需要安装ikpy库: pip install ikpy")
  Chain = None

URDF_PATH = os.path.join(SCRIPT_DIR, './right_arm.urdf')
right_arm_joints = [
    "shoulder_pitch_r_joint",
    "shoulder_roll_r_joint",
    "shoulder_yaw_r_joint",
    "elbow_pitch_r_joint",
    "elbow_yaw_r_joint",
    "wrist_pitch_r_joint",
    "wrist_roll_r_joint"
  ]
robot_chain = Chain.from_urdf_file(URDF_PATH, base_elements=["waist_yaw_link"])

def get_rotation_matrix(rx, ry, rz):
    # 输入角度，转为弧度
    rx, ry, rz = np.radians([rx, ry, rz])
    # 绕X轴
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx), np.cos(rx)]
    ])
    # 绕Y轴
    Ry = np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)]
    ])
    # 绕Z轴
    Rz = np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz), np.cos(rz), 0],
        [0, 0, 1]
    ])
    # 按照 Z-Y-X 顺序合成旋转矩阵 (即 RPY)
    return Rz @ Ry @ Rx

def solve_right_arm(offset_xyz, offset_rpy=[0, 0, 0], base_position=[0]*7, initial_guess=[0]*7):
  """求解右臂IK。

  Args:
      offset_xyz: 相对于base_position的位置偏移
      offset_rpy: 相对于base_position的旋转偏移(角度)
      base_position: FK基准关节角，偏移量基于此位姿计算
      initial_guess: IK初始猜测关节角，用于求解器收敛
  """
  start_idx = None
  end_idx = None
  for i, link in enumerate(robot_chain.links):
    if link.name == right_arm_joints[0]:
      start_idx = i
    if link.name == right_arm_joints[-1]:
      end_idx = i
  if start_idx is None or end_idx is None:
    raise RuntimeError("未找到右臂链路，请检查URDF和right_arm_joints")
  mask = [False]*len(robot_chain.links)
  for i in range(start_idx, end_idx+1):
    mask[i] = True
  robot_chain.active_links_mask = mask
  # 保证offset_xyz为3维
  if not isinstance(offset_xyz, (list, np.ndarray)) or len(offset_xyz) != 3:
    raise ValueError(f"offset_xyz必须为3维[x, y, z]数组，当前: {offset_xyz}")

  # FK基准: 用base_position计算偏移起点
  base_joints = [0] * len(robot_chain.links)
  if len(base_position) == 7:
      base_joints[1:8] = base_position
  else:
      print(f"[IK调试] Warning: base_position长度为{len(base_position)}, 期望7。使用全0初始化。")

  # IK初始猜测: 用initial_guess加速收敛
  guess_joints = [0] * len(robot_chain.links)
  if len(initial_guess) == 7:
      guess_joints[1:8] = initial_guess
  else:
      print(f"[IK调试] Warning: initial_guess长度为{len(initial_guess)}, 期望7。使用全0初始化。")

  # 1. 计算基准姿态的FK (偏移量基于此位姿)
  current_frame = robot_chain.forward_kinematics(base_joints)

  # 2. 应用偏移量 (offset_xyz是相对于base_position的增量)
  target_frame = current_frame.copy()
  target_frame[:3, 3] += offset_xyz

  # 应用旋转偏移 (RPY角度 -> 旋转矩阵)
  # offset_rpy 单位是角度(degree)
  R_offset = get_rotation_matrix(*offset_rpy)
  # 将旋转叠加在基准姿态上
  target_frame[:3, :3] = R_offset @ current_frame[:3, :3]

  try:
    angles = robot_chain.inverse_kinematics(
        target_position=target_frame[:3, 3],
        target_orientation=target_frame[:3, :3],
        orientation_mode="all",
        initial_position=guess_joints
    )
  except Exception as e:
    print("[IK调试] inverse_kinematics报错:", e)
    raise
  idxs = [i for i, link in enumerate(robot_chain.links) if link.name in right_arm_joints]
  return [angles[i] for i in idxs]

left_handPos = [-0.152,0.068,0.135,-1.155,0.124,-0.361,-0.006 ] # start 11
right_handPos = [-0.291,-0.003,-0.136,-1.155,-0.124,-0.361,0.194 ] # start 21

ID_TO_NAME = {
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

def c(name,pos):
  return SetMotorPosition(name=name, pos=pos, spd=0.2, cur=5.0)

def cArr(arr,offset):
  return [c(i+offset, arr[i]) for i in range(len(arr))]

def h_a(data):
  msg = JointState()
  msg.name = [f"{i}" for i in range(1,7)]
  msg.position = [float(i) for i in data]
  return msg

def h(val):
  msg = JointState()
  msg.name = [f"{i}" for i in range(1,6)]
  msg.position = [float(val) for i in range(1,6)]
  return msg

class Pick_placeNode(Node):
  def solve_right_arm(self, offset_xyz, offset_rpy=[0, 0, 0], base_position=None, initial_guess=None) -> Optional[list]:
    try:
      base_position = base_position if base_position is not None else self.right_joints
      initial_guess = initial_guess if initial_guess is not None else self.right_joints
      self.right_joints = solve_right_arm(offset_xyz, offset_rpy, base_position=base_position, initial_guess=initial_guess)
      return self.right_joints
    except Exception as e:
      self.get_logger().error(f"IK求解失败: {e}")
      return None
    
  def move_right_arm(self, offset_xyz, offset_rpy=[0, 0, 0], base_position=None, speed=0.2, max_joint_step=0.03):
    """Move right arm with Cartesian-space interpolation.

    Interpolates offset_xyz/offset_rpy, solves IK at each step, then pushes.

    Args:
        base_position: FK基准关节角，偏移量基于此位姿计算(默认self.right_joints)
        speed: 末端最大线速度 (m/s)，决定插值步数和总时长
        max_joint_step: 每步最大关节角增量(rad)，限幅防振荡
    """
    dist = np.linalg.norm(offset_xyz)
    if dist < 1e-6:
      return
    total_time = dist / speed
    dt = 1.0 / 15.0  # 15Hz 控制频率
    num_steps = max(int(total_time / dt), 1)
    prev_joints = list(self.right_joints)  # 上一步IK结果，作为初始猜测
    # 先求解目标关节角，用于判断是否到达
    target_joints = self.solve_right_arm(offset_xyz, offset_rpy, base_position=base_position, initial_guess=prev_joints)
    if target_joints is None:
      return
    for step in range(1, num_steps + 1):
      alpha = step / num_steps
      interp_xyz = [alpha * v for v in offset_xyz]
      interp_rpy = [alpha * v for v in offset_rpy]
      joints = self.solve_right_arm(interp_xyz, interp_rpy, base_position=base_position, initial_guess=prev_joints)
      if joints is not None:
        # 关节增量限幅: 每步不超过max_joint_step
        joints = [p + max(-max_joint_step, min(max_joint_step, j - p)) for j, p in zip(joints, prev_joints)]
        prev_joints = list(joints)
        cmds = cArr(joints, 21)
        self.push('arm', cmds)
      sleep(dt)
    # 追加步数：如果限幅导致未到达目标，继续以max_joint_step步进
    while max(abs(t - p) for t, p in zip(target_joints, prev_joints)) > 1e-4:
      joints = [p + max(-max_joint_step, min(max_joint_step, t - p)) for t, p in zip(target_joints, prev_joints)]
      prev_joints = list(joints)
      cmds = cArr(joints, 21)
      self.push('arm', cmds)
      sleep(dt)

  def move_hand(self, target_pos, speed=0.8):
    """Move right hand with interpolation.

    Args:
        target_pos: 目标手指位置(6维列表)
        speed: 手指最大速度(单位/s)，决定插值步数和总时长
    """
    current_pos = list(self.latest_action_hand_right)
    delta = [t - c for t, c in zip(target_pos, current_pos)]
    max_delta = max(abs(d) for d in delta)
    if max_delta < 1e-6:
      return
    total_time = max_delta / speed
    dt = 1.0 / 15.0  # 15Hz 控制频率
    num_steps = max(int(total_time / dt), 1)
    for step in range(1, num_steps + 1):
      alpha = step / num_steps
      interp_pos = [c + alpha * d for c, d in zip(current_pos, delta)]
      msg = h_a(interp_pos)
      self.push('hand', msg)
      sleep(dt)

  def push(self, type, msg):
    self.get_logger().info(f'Publishing to {type} with data: {msg}')
    if type == 'arm':
      self.arm_pub.publish(CmdSetMotorPosition(cmds=msg))
    elif type == 'head':
      self.head_pub.publish(CmdSetMotorPosition(cmds=msg))
    elif type == "hand":
      self.hand_pub.publish(msg)
    elif type == "hand_left":
      self.hand_left_pub.publish(msg)
    else:
      self.get_logger().error(f'Unknown type: {type}')

  def __init__(self):
    super().__init__('pick_place_node')
    # 创建发布者，用于发送控制命令  
    self.arm_pub = self.create_publisher(CmdSetMotorPosition, '/arm/cmd_pos', 10)
    self.head_pub = self.create_publisher(CmdSetMotorPosition, '/head/cmd_pos', 10)
    self.hand_pub = self.create_publisher(JointState, '/inspire_hand/ctrl/right_hand', 10)
    self.hand_left_pub = self.create_publisher(JointState, '/inspire_hand/ctrl/left_hand', 10)
    self.apple_pub = self.create_publisher(Point, '/scene/apple/offset', 10)
    self.reset_pub = self.create_publisher(Bool, '/sim/cmd_reset', 10)
    self.task_completed_sub = self.create_subscription(Float32, '/sim/task_completed', self.task_completed_cb, 10)
    
    # 独立订阅各话题，各自回调更新 latest_* 变量
    self.create_subscription(MotorStatusMsg, '/arm/status', self.cmd_pos_cb, 10)
    self.create_subscription(JointState, '/inspire_hand/state/right_hand', self.hand_right_cb, 10)
    self.create_subscription(JointState, '/inspire_hand/state/left_hand', self.hand_left_cb, 10)
    self.create_subscription(Image, '/ob_camera_head/color/image_raw', self.img_cb, 10)
    self.create_subscription(CmdSetMotorPosition, '/arm/cmd_pos', self.arm_cmd_pos_cb, 10)
    self.create_subscription(JointState, '/inspire_hand/ctrl/right_hand', self.hand_cmd_cb, 10)
    self.create_subscription(JointState, '/inspire_hand/ctrl/left_hand', self.hand_left_cmd_cb, 10)

    self.save_interval = 1.0 / 15.0  # 15Hz
    # Timer 驱动 15Hz 采样
    self.save_timer = self.create_timer(self.save_interval, self.timer_save_callback)
    # 数据缓存map
    self.data_buffer = {
       "arm_right": [],
       "hand_right": [],
       "arm_left": [],
       "hand_left": [],
       "action_arm_right": [],
       "action_arm_left": [],
       "action_hand_right": [],
       "action_hand_left": [],
       "img": [],
       "timestamp": [],
    }
    self.is_saving = False
    # Store latest states
    # Right Arm: 7 joints (21-27)
    self.latest_arm_right_pos = [0.0] * 7
    self.latest_arm_left_pos = [0.0] * 7
    # Right Hand: 6 joints
    self.latest_hand_right_pos = [1.0] * 6
    self.latest_hand_left_pos = [1.0] * 6
    # Latest action (command) data
    self.latest_action_arm_right = [0.0] * 7
    self.latest_action_arm_left = [0.0] * 7
    self.latest_action_hand_right = [1.0] * 6
    self.latest_action_hand_left = [1.0] * 6
    self.latest_img = None
    self.latest_task_dist = 1000.0
    
    self.right_joints = right_handPos

  def timer_save_callback(self):
    if self.is_saving:
        self.record_snapshot()
        now = self.get_clock().now().nanoseconds / 1e9
        if 'timestamp' in self.data_buffer:
            self.data_buffer['timestamp'].append(now)

  def run_task(self):
    """
    执行主要的拾取放置和数据采集任务。
    由于spin在独立线程运行，这里可以使用阻塞式调用(sleep)而不影响回调接收。
    """
    self.reset()
    x,y = self.random_apple()
    sleep(5)
    # self.start_save_data()
    self.pick(x,y)
    self.place()
    self.home()
    sleep(2) 
    self.get_logger().info(f"Final Task Completion Check: {self.latest_task_dist:.4f}")
    if self.latest_task_dist < 0.12:  # 任务完成的距离阈值 (根据实际情况调整)
      self.save_data()
    else:
      self.is_saving = False
      self.get_logger().warn(f"Task not completed (apple not in plate). latest_task_dist={self.latest_task_dist:.4f}. Data will NOT be saved.")

    self.reset_sim()

  def reset_sim(self):
    msg = Bool()
    msg.data = True
    self.reset_pub.publish(msg)
    self.get_logger().info("Sent simulation reset command")

  def start_save_data(self):
    self.is_saving = True
    self.get_logger().info("Started recording data at 15Hz (timer-driven)")

  def record_snapshot(self):
    if not self.is_saving: return
    try:
        # 1. Right Arm Data
        self.data_buffer['arm_right'].append(list(self.latest_arm_right_pos))
        self.data_buffer['arm_left'].append(list(self.latest_arm_left_pos))
        # 2. Right Hand Data
        self.data_buffer['hand_right'].append(list(self.latest_hand_right_pos))
        self.data_buffer['hand_left'].append(list(self.latest_hand_left_pos))
        # 3. Action Data
        self.data_buffer['action_arm_right'].append(list(self.latest_action_arm_right))
        self.data_buffer['action_arm_left'].append(list(self.latest_action_arm_left))
        self.data_buffer['action_hand_right'].append(list(self.latest_action_hand_right))
        self.data_buffer['action_hand_left'].append(list(self.latest_action_hand_left))
        # 4. Image
        if self.latest_img is not None:
             self.data_buffer['img'].append(self.latest_img)
        else:
             self.data_buffer['img'].append(np.zeros((360, 640, 3), dtype=np.uint8)) 
        
    except Exception as e:
        self.get_logger().error(f"Error recording snapshot: {e}")

  def save_data(self):
    self.is_saving = False

    # Use clock for filename
    ts = self.get_clock().now().seconds_nanoseconds()
    # 保存到项目根目录下的 dataset/ 子目录
    PROJECT_DIR = os.path.dirname(os.path.dirname(SCRIPT_DIR))
    dataset_root = os.path.join(PROJECT_DIR, "dataset")
    dirName = os.path.join(dataset_root, f"{ts[0]}")
    os.makedirs(dirName, exist_ok=True)
    os.chmod(dirName, 0o777)
    os.chmod(dataset_root, 0o777)
    filename = os.path.join(dirName, "trajectory.hdf5")
    self.get_logger().info(f"Saving data to {filename}...")
    
    with h5py.File(filename, 'w') as f:
        l = len(self.data_buffer['arm_right'])
        print(f"Data length: {l} snapshots")
        if l == 0: return

        f.create_dataset('puppet/arm_right_position_align/data', data=np.array(self.data_buffer['arm_right']))
        f.create_dataset('puppet/end_effector_right_position_align/data', data=np.array(self.data_buffer['hand_right']))
        f.create_dataset('puppet/arm_left_position_align/data', data=np.array(self.data_buffer['arm_left']))
        f.create_dataset('puppet/end_effector_left_position_align/data', data=np.array(self.data_buffer['hand_left']))
        f.create_dataset('action/arm_right_position_align/data', data=np.array(self.data_buffer['action_arm_right']))
        f.create_dataset('action/arm_left_position_align/data', data=np.array(self.data_buffer['action_arm_left']))
        f.create_dataset('action/end_effector_right_position_align/data', data=np.array(self.data_buffer['action_hand_right']))
        f.create_dataset('action/end_effector_left_position_align/data', data=np.array(self.data_buffer['action_hand_left']))
        f.create_dataset('observations/timestamp', data=np.array(self.data_buffer['timestamp']))
        # Key 3: camera_observations/color_images/camera_head
        # Encode images to jpg bytes for storage
        compressed_len = len(self.data_buffer['img'])
        if compressed_len > 0:
            dt = h5py.special_dtype(vlen=np.dtype('uint8'))
            img_ds = f.create_dataset('camera_observations/color_images/camera_head', (compressed_len,), dtype=dt)
            
            for i, img_rgb in enumerate(self.data_buffer['img']):
                # Convert RGB (stored) to BGR (required for opencv encoding)
                img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
                success, encoded_img = cv2.imencode('.jpg', img_bgr)
                if success:
                     # Remove extra dim if present (n, 1) -> (n,)
                     img_ds[i] = encoded_img.flatten()
                else:
                     self.get_logger().error(f"Failed to encode image {i}")

    os.chmod(filename, 0o666)
    self.get_logger().info(f"Data saved successfully.")

  def cmd_pos_cb(self, msg):
    # Msg: MotorStatusMsg
    # Extract Right Arm (IDs 21-27)
    # Assuming msg.motors is a list of objects with .name and .pos
    if hasattr(msg, 'status'): # Check if list exists
        # Create a temp map for this message
        msg_map = {m.name: m.pos for m in msg.status}
        
        # IDs for Right Arm
        right_arm_ids = [21, 22, 23, 24, 25, 26, 27]
        new_pos = []
        for pid in right_arm_ids:
            new_pos.append(msg_map.get(pid, 0.0))
        self.latest_arm_right_pos = new_pos

        left_arm_ids = [11, 12, 13, 14, 15, 16, 17]
        new_pos = []
        for pid in left_arm_ids:
            new_pos.append(msg_map.get(pid, 0.0))
        self.latest_arm_left_pos = new_pos

  def arm_cmd_pos_cb(self, msg):
    if hasattr(msg, 'cmds') and msg.cmds:
        msg_map = {m.name: m.pos for m in msg.cmds}

        right_arm_ids = [21, 22, 23, 24, 25, 26, 27]
        new_pos = []
        for pid in right_arm_ids:
            new_pos.append(msg_map.get(pid, 0.0))
        self.latest_action_arm_right = new_pos

        left_arm_ids = [11, 12, 13, 14, 15, 16, 17]
        new_pos = []
        for pid in left_arm_ids:
            new_pos.append(msg_map.get(pid, 0.0))
        self.latest_action_arm_left = new_pos

  def hand_cb(self, msg):
    # Msg: JointState for Right Hand
    # Names should be "1" to "6"
    # We want 6 floats
    if len(msg.position) >= 6:
        val_map = {}
        for n, p in zip(msg.name, msg.position):
            try:
                val_map[str(n)] = float(p)
            except: pass
            
        # Extract 1..6
        hand_vals = []
        for i in range(1, 7):
            hand_vals.append(val_map.get(str(i), 0.0))
            
        return hand_vals
    
  def hand_right_cb(self, msg):
    self.latest_hand_right_pos = self.hand_cb(msg)

  def hand_left_cb(self, msg):
    self.latest_hand_left_pos = self.hand_cb(msg)

  def hand_cmd_cb(self, msg):
    pos = list(msg.position)
    if len(pos) >= 6:
        self.latest_action_hand_right = [float(p) for p in pos[:6]]
    elif len(pos) >= 5:
        self.latest_action_hand_right[:5] = [float(p) for p in pos[:5]]

  def hand_left_cmd_cb(self, msg):
    pos = list(msg.position)
    if len(pos) >= 6:
        self.latest_action_hand_left = [float(p) for p in pos[:6]]
    elif len(pos) >= 5:
        self.latest_action_hand_left[:5] = [float(p) for p in pos[:5]]

  def task_completed_cb(self, msg):
    self.latest_task_dist = msg.data

  def img_cb(self, msg):
    try:
        if msg.encoding == 'rgb8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            self.latest_img = img
        elif msg.encoding == 'bgr8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            img = img[..., ::-1] # BGR to RGB
            self.latest_img = img
    except Exception as e:
        self.get_logger().error(f"Img decode error: {e}")

  def random_apple(self):
    x,y = np.random.uniform(-0.025, 0.025), np.random.uniform(-0.05, 0.0)
    # x,y = 0.0, 0.0
    self.get_logger().info(f"Randomizing apple position with offset: x={x:.2f}, y={y:.2f}")
    self.apple_pub.publish(Point(x=x, y=y, z=0.0))
    return x,y

  def pick(self, x, y):
    self.start_save_data()
    self.move_right_arm([0, 0, 0.05],[0.0,0.0,0.0],base_position=right_handPos)
    sleep(0.5)
    self.move_right_arm([-0.08 + x, -0.1 + y, 0.05],[0.0,0.0,0.0],base_position=right_handPos)
    sleep(0.5)
    self.move_right_arm([-0.08 + x, -0.13 + y, 0.05],[0.0,4.0,20.0],base_position=right_handPos)
    sleep(0.5)  # 等待运动完成
    self.move_right_arm([-0.08 + x, -0.13 + y, -0.07],[0.0,4.0,20.0],base_position=right_handPos)
    sleep(0.5)  # 等待运动完成
    self.move_right_arm([-0.05 + x, 0.0 + y, -0.09],[0.0,4.0,20.0],base_position=right_handPos)
    
    sleep(0.5)  # 等待运动完成
    self.move_hand([0.3,0.3,0.3,0.3,0.3,0])
    sleep(0.5)  # 等待运动完成
    self.move_right_arm([0, 0, 0.20],[0.0,4.0,20.0],base_position=right_handPos)  
  def pick_new(self, x, y):
    # x向前y向左，旋转右手定则
    # 手部抬起避免碰到苹果
    self.start_save_data()
    self.move_right_arm([0, 0, 0.03],[-60,10.0,0.0],base_position=right_handPos) 
    self.move_hand([1,1,1,1,1,1]) 
    sleep(1.5)
    self.move_right_arm([x - 0.08, y - 0.1, 0.03],[-40.0,10.0,20.0],base_position=right_handPos) 
    self.move_hand([1,1,1,1,1,0]) 
    sleep(2)  
    self.move_right_arm([x -0.08, y - 0.1, -0.08],[-10.0,10.0,20.0],base_position=right_handPos)
    sleep(1)
    self.move_hand([0.9,0.85,0.8,0.75,0.9,0])
    self.move_right_arm([x -0.04, y - 0.05, -0.09],[-10.0,10.0,20.0],base_position=right_handPos)
    sleep(1)
    self.move_hand([0.3,0.3,0.3,0.3,0.3,0])
    sleep(1)
    self.move_right_arm([x, y, 0.15],[-10.0,10.0,20.0],base_position=right_handPos)
    sleep(1)
  def place(self):
    self.move_right_arm([-0.00, 0.15,0],[0,15.0,30.0],base_position=right_handPos)
    self.move_right_arm([-0.00, 0.15,-0.00],[0,20.0,30.0],base_position=right_handPos)
    self.move_hand([1,1,1,1,1,1])
    
  def home(self):
    self.push('arm', cArr(left_handPos, 11) + cArr(right_handPos, 21))

  def reset(self):
    # arm参数
    self.push('head', [c(1, 0.0),c(2, 0.35),c(3, 0.0)])
    self.push('hand', h_a([1,1,1,1,1,0]))
    self.push('arm', [c(12, 1.5),c(22, -1.5)])
    sleep(2) 
    self.push('arm', [c(14, -1.5),c(24, -1.5)])
    sleep(2)  # 等待消息发送完成
    self.push('arm', cArr(left_handPos, 11) + cArr(right_handPos, 21))
    self.get_logger().info('Reset commands published to /arm/cmd_pos and /head/cmd_pos')

def main():
  rclpy.init()
  node = Pick_placeNode()
   # 在单独的线程中运行spin，处理回调
  spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
  spin_thread.start()
  try:
    node.run_task()
  except KeyboardInterrupt:
    pass
  finally:
      sleep(1) 
  rclpy.shutdown()

if __name__ == '__main__':
  main()

# y -> 右 x -> 前 z -> 上
