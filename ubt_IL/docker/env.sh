#!/bin/bash
# Docker environment variables for LeRobot TienKung container
CONTAINER_NAME="lerobot-tienkung"
IMAGE="lerobot-tienkung:humble"
BASE_IMAGE="swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/huggingface/lerobot-gpu:latest"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PLUGIN_DIR="${PROJECT_ROOT}/tienkung"
HF_HOME="/ubt_IL/.cache/huggingface"
PIP_MIRROR="-i https://pypi.tuna.tsinghua.edu.cn/simple"
UV_INDEX_URL="https://pypi.tuna.tsinghua.edu.cn/simple"
DOMAIN_ID="${DOMAIN_ID:-0}"
