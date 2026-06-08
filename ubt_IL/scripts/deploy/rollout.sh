#!/bin/bash
# 天工机器人部署（rollout）脚本
# 在 lerobot-tienkung 容器内运行
# 前置条件：Bridge2 已启动（由 TienKungRobot.connect() 自动启动，或手动 /usr/bin/python3 /ubt_IL/tienkung/ros2_deploy_bridge.py）
set -e

# === 配置 ===
POLICY_PATH="${POLICY_PATH:-/ubt_IL/model/real_pick_place_act/checkpoints/last/pretrained_model}"
STRATEGY="${STRATEGY:-base}"
FPS="${FPS:-15}"
DURATION="${DURATION:-60}"
TASK="${TASK:-pick and place}"
# ZMQ_HOST="${ZMQ_HOST:-192.168.41.2}" # 真机地址
ZMQ_HOST="${ZMQ_HOST:-127.0.0.1}" # 仿真器地址

cd /ubt_IL/lerobot

/lerobot/.venv/bin/lerobot-rollout \
    --strategy.type="$STRATEGY" \
    --policy.path="$POLICY_PATH" \
    --robot.type=tienkung \
    --robot.bridge_enabled=true \
    --robot.cameras="{head: {type: image_server, server_address: '${ZMQ_HOST}', port: 5558, offset_x: 0, width: 640, height: 360, fps: $FPS, display: true}}" \
    --task="$TASK" \
    --fps="$FPS" \
    --duration="$DURATION"
