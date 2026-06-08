# TienKung-IL-LAB

优必选天工机器人模仿学习工具链（开发中）

## 项目简介

本项目基于 [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim) 与 [LeRobot](https://github.com/huggingface/lerobot) 框架开发，为天工（TienKung）机器人提供完整的模仿学习工具链，涵盖以下核心能力：

| 能力 | 说明 | 状态 |
|------|------|------|
| 🌐 ROS 仿真环境 | 高逼真度 Isaac Sim 仿真，支持遥操作与数据采集 | ✅ 已完成 |
| 🎮 遥控操作 | 键盘/空间鼠标等设备遥操作，仿真与真机统一接口 | 🚧 开发中 |
| 📦 数据采集与转换 | HDF5 / LeRobot 格式数据采集，格式转换与清洗 | ✅ 转换已完成 / 🚧 采集开发中 |
| 🧠 模型训练 | 基于 LeRobot 的模仿学习策略训练 | ✅ 已完成 |
| 🤖 真机部署 | 模型推理与真机控制部署 | ✅ 已完成 |

仿真环境已开发完成，具体介绍与使用说明请参考 [ubt_sim/README.md](ubt_sim/README.md)。

## 仿真模块
### 快速开始

```bash
# 1. 构建并启动容器
cd docker/isaac_sim
bash run.sh build && bash run.sh start && bash run.sh init && bash run.sh check
# 若真机模式启动容器：ROS_DOMAIN_ID=0 bash run.sh start

# 2. 启动仿真（自动启动 ROS2-ZMQ 桥接）
bash run.sh bash
bash scripts/start_sim.sh 
# 真机模式启动仿真+桥接：ROS_DOMAIN_ID=0 bash scripts/start_sim.sh
# 按R机器人可复位

# 3. 数据采集（同一容器内，用系统 Python 3.10）
/usr/bin/python3 /ubt_sim/teleoperation/control/reset.py  # 机器人回零
/usr/bin/python3 /ubt_sim/teleoperation/control/pick_place_save_data.py  # 单次
bash /ubt_sim/teleoperation/control/save_data.sh                         # 批量
```
注意：使用**echo $ROS_DOMAIN_ID**和**ros2 topic list**检查当前模式仿真/真机，以及桥接是否启动。

### 模式说明

通过 `ROS_DOMAIN_ID` 区分仿真与真机：

| 模式 | ROS_DOMAIN_ID | 说明 |
|------|--------------|------|
| 仿真 | 146 | ZMQ 桥接连接 Isaac Sim |
| 真机 | 0 | ZMQ 桥接连接真实机器人 |

## 模型训练

基于 LeRobot ACT 策略，数据集来源于仿真采集的 HDF5 或真机遥操作数据，训练在 `lerobot-tienkung` 容器内完成。`bash run.sh check` 用于环境健康检查。

### 快速开始

```bash
# 1. 构建并启动训练容器（自动启动 Bridge2）
cd ubt_IL/docker
bash run.sh build && bash run.sh start && bash run.sh check

# 2. 数据转换：HDF5 -> LeRobot 格式（宿主机 conda 环境）
bash ubt_IL/scripts/convert/convert.sh
# 自定义：SRC_ROOT=path/to/hdf5 TGT_PATH=path/to/out REPO_ID=my_dataset \
#          bash ubt_IL/scripts/convert/convert.sh

# 3. 训练（容器内）
bash ubt_IL/docker/run.sh bash
bash /ubt_IL/scripts/deploy/train.sh
# 自定义：DATASET_ROOT=/ubt_IL/dataset/my_data STEPS=10000 \
#          OUTPUT_DIR=/ubt_IL/model/my_model bash /ubt_IL/scripts/deploy/train.sh
```

### 关键参数

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `DATASET_ROOT` | `/ubt_IL/dataset/real_merged` | LeRobot 数据集路径 |
| `OUTPUT_DIR` | `/ubt_IL/model/real_pick_place_act` | 模型输出路径 |
| `STEPS` | `50000` | 训练总步数（每 10k 保存 checkpoint） |
| `BATCH_SIZE` | `8` | 批次大小 |

Checkpoint 路径：`{OUTPUT_DIR}/checkpoints/last/pretrained_model`。

详细参数、ACT 配置、数据可视化命令见 [ubt_IL/README.md](ubt_IL/README.md#3-模型训练)。

## 真机部署

天工真机部署需 `ROS_DOMAIN_ID=0`，部署机与机器人需同网段（如 `192.168.41.x`）。架构为容器内 LeRobot 通过 ZMQ 与 Bridge2 通信，相机由机器人端 ImageServer 提供 JPEG 流。

### 快速开始

```bash
# 0. 网络配置：编辑 ubt_IL/docker/fastdds_no_shm.xml 第二个 <address>，
#    改为本机 IP（如 192.168.41.99），保证与机器人在同一网段，随后重启容器
cd ubt_IL/docker
bash run.sh restart

# 1. 机器人端启动相机服务（仅真机部署需要）
scp ubt_IL/scripts/deploy/camera/image_server.py nvidia@192.168.41.2:~
ssh nvidia@192.168.41.2 'python3 image_server.py'

# 2. 容器内复位 + 推理
bash run.sh bash
/usr/bin/python3 /ubt_IL/scripts/deploy/reset.py     # 机器人回零
POLICY_PATH=/ubt_IL/model/test_model ZMQ_HOST=192.168.41.2 DURATION=60 \
  bash /ubt_IL/scripts/deploy/rollout.sh

# 3. （可选）数据集回放校验链路
/usr/bin/python3 /ubt_IL/scripts/deploy/replay.py \
  --dataset /ubt_IL/dataset/real_grasp_bottle --episode 0 --rate 30
```

### 关键参数

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `POLICY_PATH` | `.../real_pick_place_act/.../pretrained_model` | 模型 checkpoint |
| `ZMQ_HOST` | `127.0.0.1` | ImageServer 地址（真机改机器人 IP） |
| `DURATION` | `60` | 运行时长（秒） |
| `FPS` | `15` | 控制环频率（与训练 fps 对齐） |

注意：`ubt_IL/docker/fastdds_no_shm.xml` 中的 IP 必须改为本机 IP，否则 ROS 无法与真机通信。详细架构图、26 维向量布局见 [ubt_IL/CLAUDE.md](ubt_IL/CLAUDE.md)；完整部署参数与 `lerobot-rollout` CLI 调用见 [ubt_IL/README.md](ubt_IL/README.md#4-模型部署)。
