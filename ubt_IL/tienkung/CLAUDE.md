# TienKung Plugin — Technical Architecture

## Architecture Overview

Dual-process bridge design: LeRobot (Python 3.12) communicates with `tienkung/ros2_deploy_bridge.py` (Bridge2, Python 3.10) via ZMQ. Bridge2 interfaces with the TienKung robot hardware via ROS2 DDS. Images come from a separate ImageServer process.

```
LeRobot Inference (Python 3.12)       Bridge2 (Python 3.10)
  ZMQ PUB → 5559 (actions)       →    ZMQ SUB ← 5559
  ZMQ SUB ← 5560 (status)        ←    ZMQ PUB → 5560
                                         │
                                    ROS2 DDS
                                         │
                                  ┌──────┴──────┐
                                  │             │
                               TienKung HW   Other ROS2 Nodes

LeRobot Inference (Python 3.12)       ImageServer (独立进程)
  ZMQ SUB ← 5558 (images)        ←    ZMQ PUB → 5558
                                         │
                                    Camera Hardware
                                    (Orbbec/OpenCV)
```

## ZMQ Port Reference

### LeRobot ↔ Bridge2

| Port | Direction | Purpose |
|------|-----------|---------|
| 5559 | LeRobot PUB → Bridge2 SUB | 26-dim action commands |
| 5560 | Bridge2 PUB → LeRobot SUB | 26-dim joint state |

### LeRobot ↔ ImageServer

| Port | Direction | Purpose |
|------|-----------|---------|
| 5558 | ImageServer PUB → ImageServerCamera SUB | Camera images (JPEG) |

## 26-Dim State/Action Vector

Matches v0.1 training data order:

```
Index  0-6:   Left arm (7 dims)   → left_arm_joint_1.pos ~ left_arm_joint_7.pos
Index  7-12:  Left hand (6 dims)   → left_hand_joint_1.pos ~ left_hand_joint_6.pos
Index 13-19:  Right arm (7 dims)   → right_arm_joint_1.pos ~ right_arm_joint_7.pos
Index 20-25:  Right hand (6 dims)  → right_hand_joint_1.pos ~ right_hand_joint_6.pos
```

ZMQ internal message format:
```json
{
  "left_arm": [7 floats],
  "left_hand": [6 floats],
  "right_arm": [7 floats],
  "right_hand": [6 floats],
  "ts": <timestamp>
}
```

Action splitting in Bridge2:
- Arms: `left_arm + right_arm` → `CmdSetMotorPosition` (left motor IDs 11–17, right 21–27)
- Left hand: `left_hand` → `JointState` with Inspire clip logic
- Right hand: `right_hand` → `JointState` with Inspire clip logic

Inspire hand clip logic (Bridge2, matching v0.1):
```python
position = [np.clip(pos, 0, 1) for pos in position]
position = [pos - 0.2 if pos < 0.9 else pos for pos in position]
position = [round(pos, 1) for pos in position]
```

## ROS2 Topics (Bridge2)

### Subscriptions

| Topic | Message Type | Purpose |
|------|-------------|---------|
| `/arm/status` | `MotorStatusMsg` | Dual-arm joint state (14 dims) |
| `/inspire_hand/state/left_hand` | `JointState` | Left hand state (6 dims) |
| `/inspire_hand/state/right_hand` | `JointState` | Right hand state (6 dims) |

Topic names can be overridden via `--ros_namespace` CLI arg.

### Publications

| Topic | Message Type | Purpose |
|------|-------------|---------|
| `/arm/cmd_pos` | `CmdSetMotorPosition` | Dual-arm joint control |
| `/inspire_hand/ctrl/left_hand` | `JointState` | Left hand control |
| `/inspire_hand/ctrl/right_hand` | `JointState` | Right hand control |

Topic names can be overridden via `--cmd_namespace` CLI arg.

## Model Config Reference (ACT)

| Parameter | Value | Notes |
|-----------|-------|-------|
| `chunk_size` | 100 | Predict 100 future action steps per inference |
| `n_action_steps` | 100 | Execute 100 steps before re-inference |
| `n_obs_steps` | 1 | Use only current frame |
| `vision_backbone` | ResNet18 | Visual encoder |
| `dim_model` | 512 | Transformer dimension |
| `use_vae` | true | VAE latent encoding |

ACT produces 100-step action sequences per inference, executed fully before the next inference. At fps=15, one inference covers ~6.7s. GPU sync inference: ~50–100ms/step, real-world control rate ~5–10 Hz. Use `--inference.type=rtc` for async / higher frequency.

## Container Environment

The `entrypoint.sh` auto-installs on container start:

| Component | Method | Location |
|-----------|--------|----------|
| lerobot | `uv pip install -e /ubt_IL/lerobot` | venv (editable) |
| tienkung plugin | `uv pip install -e /ubt_IL/tienkung/lerobot_robot_tienkung` | venv (editable) |
| ROS2 Humble | apt (pre-installed in image) | `/opt/ros/humble/` |
| bodyctrl_msgs | deb (pre-installed in image) | `/opt/ros/humble/` |

Both lerobot and the plugin are installed in editable mode — source changes take effect immediately.

### Container Management

All container operations use `run.sh` subcommands (see `docker/env.sh` for env vars):

```bash
bash run.sh build          # Build image
bash run.sh start          # Create/start container + Bridge2 (idempotent)
bash run.sh stop           # Stop container (Bridge2 first)
bash run.sh restart        # Restart container
bash run.sh bash           # Enter container shell
bash run.sh rm             # Remove container
bash run.sh bridge-start   # Start Bridge2
bash run.sh bridge-stop    # Stop Bridge2
bash run.sh check          # Environment health check
```

### Environment Variables

| Variable | Default | Purpose |
|----------|---------|---------|
| `DOMAIN_ID` | `0` | ROS_DOMAIN_ID (真机默认 0) |

## Plugin Registration

The plugin registers via `make_device_from_device_class` fallback in `__init__.py`. No modifications to upstream lerobot source are required.

## Default Home Position (14-dim)

```python
[-0.152, 0.068, 0.135, -1.155, 0.124, -0.361, -0.006,
 -0.291, -0.003, -0.136, -1.155, -0.124, -0.361, 0.194]
```
