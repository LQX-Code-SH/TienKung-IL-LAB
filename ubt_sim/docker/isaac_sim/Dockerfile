FROM nvcr.nju.edu.cn/nvidia/isaac-lab:2.2.0

# ---- ROS2 Humble (清华大学镜像源) ----
RUN apt-get update && apt-get install -y --no-install-recommends software-properties-common curl && \
    add-apt-repository universe && \
    curl -sSL https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-cv-bridge \
    ros-humble-message-filters \
    ros-humble-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*

# ---- ROS2 子进程所需 Python 3.10 包 ----
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip && rm -rf /var/lib/apt/lists/* && \
    /usr/bin/python3 -m pip install --no-cache-dir -i https://pypi.tuna.tsinghua.edu.cn/simple \
    "numpy<2" pyzmq h5py ikpy "opencv-python-headless<4.10" pyyaml empy lark protobuf

# ---- C++ 图像桥接编译依赖 ----
RUN apt-get update && apt-get install -y --no-install-recommends g++ make libzmq3-dev && \
    rm -rf /var/lib/apt/lists/*
