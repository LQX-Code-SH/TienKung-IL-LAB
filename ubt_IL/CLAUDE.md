# ubt_IL — UBTECH 模仿学习训练部署平台

基于 LeRobot 的天工双臂+灵巧手机器人模仿学习训练与部署平台。

## 架构

两层桥接架构，LeRobot 通过 ZMQ 与 Bridge2 通信，Bridge2 通过 ROS2 DDS 与天工硬件通信：

```
LeRobot (3.12) ──ZMQ 5559/5560──► Bridge2 (3.10) ──ROS2 DDS──► 天工硬件
                │
                └──ZMQ 5558──► ImageServer (独立进程) ──► 相机硬件
```

- **Bridge2** (`tienkung/ros2_deploy_bridge.py`): ZMQ 5559(action)/5560(state) <-> ROS2 DDS
- **ImageServer** (`scripts/deploy/image_server.py`): 相机硬件 → ZMQ 5558 (JPEG) → ImageServerCamera

## 目录结构

| 目录 | 内容 | 运行环境 | 说明 |
|------|------|----------|------|
| `lerobot/` | LeRobot 源码（最小副本） | 容器内 | 上游 HuggingFace LeRobot，脱离 git |
| `tienkung/` | 天工插件 | 容器内 | 独立 pip 包，注册为 `tienkung` 类型 |
| `docker/` | 容器构建与管理 | 宿主机 | `env.sh` + `run.sh` 统一管理 |
| `dataset/` | 转换后 LeRobot 数据集 | — | HDF5→LeRobot 格式 |
| `model/` | 训练/测试模型权重 | — | ACT 策略 checkpoint |
| `scripts/convert/` | 数据转换脚本 + 配置 | 宿主机 conda | HDF5 → LeRobot 格式转换 |
| `scripts/deploy/` | 训练、部署、工具脚本 | 容器内 | train.sh, rollout.sh, reset.py, test_zmq.py |

## Quick Commands

```bash
# 构建镜像 + 启动容器（自动启动 Bridge2）
cd docker
bash run.sh build && bash run.sh start

# 进入容器
bash run.sh bash

# 环境健康检查
bash run.sh check

# 停止容器
bash run.sh stop

# 数据转换（宿主机 conda 环境）
bash scripts/convert/convert.sh

# 训练（容器内）
bash /ubt_IL/scripts/deploy/train.sh

# 部署（容器内）
bash /ubt_IL/scripts/deploy/rollout.sh

# 机器人复位（容器内）
source /opt/ros/humble/setup.bash
python3 /ubt_IL/scripts/deploy/reset.py
```

## 容器内路径

| 路径 | 说明 |
|------|------|
| `/ubt_IL/lerobot` | LeRobot 源码（bind mount） |
| `/ubt_IL/tienkung/` | 天工插件（bind mount） |
| `/ubt_IL/dataset/` | 数据集（bind mount） |
| `/ubt_IL/model/` | 模型权重（bind mount） |
| `/ubt_IL/scripts/` | 脚本（bind mount） |
| `/lerobot/.venv/` | 基础镜像 venv（不动） |
| `/ubt_IL/tienkung/ros2_deploy_bridge.py` | Bridge2 脚本（bind-mount 自动可用） |

## 26-Dim State/Action Vector

```
[0-6] 左臂(7) | [7-12] 左手(6) | [13-19] 右臂(7) | [20-25] 右手(6)
```

ZMQ 消息: `{"left_arm":[7], "left_hand":[6], "right_arm":[7], "right_hand":[6], "ts":...}`

## Critical Constraints

- **ROS2 Domain ID = 0**（真机），可通过 `DOMAIN_ID` 环境变量覆盖
- **系统 Python 3.10**（`/usr/bin/python3`），ROS2 Bridge2 使用
- **LeRobot Python 3.12**（`/lerobot/.venv/bin/python`），训练/推理使用
- **numpy<2**（cv_bridge 兼容性），**opencv-python-headless<4.10**
- **天工插件与 LeRobot 源码相互独立**：插件通过 `@register_subclass` 注册，无需修改上游代码

## 嵌套文档

- [tienkung/CLAUDE.md](tienkung/CLAUDE.md) — 天工插件技术架构（ZMQ 端口、ROS2 话题、ACT 配置、灵巧手逻辑、插件注册）
- [tienkung/README.md](tienkung/README.md) — 部署操作指南（快速开始、训练、部署、验证）
