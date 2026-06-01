#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/_build"

echo "=== zmq_image_bridge build ==="

# ---- 1. Auto-detect and source ROS 2 ----
if [ -z "$ROS_DISTRO" ]; then
    for distro in humble jazzy rolling iron; do
        setup="/opt/ros/${distro}/setup.bash"
        if [ -f "$setup" ]; then
            echo "Sourcing ROS 2 ${distro}..."
            source "$setup" || true
            break
        fi
    done
fi

if [ -z "$ROS_DISTRO" ]; then
    echo "[ERROR] No ROS 2 distribution found. Install ROS 2 or source setup.bash first."
    exit 1
fi

echo "Using ROS_DISTRO=${ROS_DISTRO}"

# ---- 2. Clean stale build if ROS distro changed ----
if [ -f "${BUILD_DIR}/ros_distro.txt" ]; then
    if [ "$(cat "${BUILD_DIR}/ros_distro.txt")" != "$ROS_DISTRO" ]; then
        echo "ROS_DISTRO changed, cleaning build directory..."
        rm -rf "$BUILD_DIR"
    fi
fi

# ---- 3. Install missing dependencies (works with or without sudo) ----
MISSING_PKGS=()
if ! pkg-config --exists libzmq 2>/dev/null; then
    MISSING_PKGS+=(libzmq3-dev)
fi
if ! command -v cmake &>/dev/null; then
    MISSING_PKGS+=(cmake g++ make)
fi

if [ ${#MISSING_PKGS[@]} -gt 0 ]; then
    APT_CMD="apt-get install -y -qq ${MISSING_PKGS[*]}"
    if command -v sudo &>/dev/null; then
        sudo apt-get update -qq && sudo $APT_CMD
    elif [ "$(id -u)" -eq 0 ]; then
        apt-get update -qq && $APT_CMD
    else
        echo "[ERROR] Missing packages: ${MISSING_PKGS[*]}. No sudo available and not root."
        echo "        Install manually: apt-get install ${MISSING_PKGS[*]}"
        exit 1
    fi
fi

# ---- 4. Build with CMake (handles all ROS 2 link deps automatically) ----
echo "Configuring with CMake..."
mkdir -p "$BUILD_DIR"
echo "$ROS_DISTRO" > "${BUILD_DIR}/ros_distro.txt"
cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release 2>&1

echo "Compiling..."
cmake --build "$BUILD_DIR" -j"$(nproc)" 2>&1

# ---- 5. Copy executable next to script ----
cp -f "${BUILD_DIR}/zmq_image_bridge" "${SCRIPT_DIR}/zmq_image_bridge"
echo "Build successful: ${SCRIPT_DIR}/zmq_image_bridge"
