FROM swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/huggingface/lerobot-gpu:latest

USER root
ENV DEBIAN_FRONTEND=noninteractive

# 清华 TUNA apt 源
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates \
    && sed -i 's|http://archive.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list \
    && sed -i 's|http://security.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list \
    && rm -rf /var/lib/apt/lists/*

# ROS2 Humble 源 + 安装
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common curl \
    && add-apt-repository -y universe \
    && curl -sSL https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
       https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
       | tee /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-message-filters \
    ros-humble-rclcpp \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-builtin-interfaces \
    ros-humble-rosidl-default-runtime \
    libyaml-cpp-dev \
    libxkbcommon-x11-0 libvulkan1 mesa-vulkan-drivers \
    libgtk-3-0 libcanberra-gtk3-module \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# bodyctrl_msgs (预编译 deb)
COPY docker/ros2_msgs/ros-humble-bodyctrl-msgs_0.0.1-1_amd64.deb /tmp/
RUN dpkg -i /tmp/ros-humble-bodyctrl-msgs_0.0.1-1_amd64.deb \
    && rm -f /tmp/ros-humble-bodyctrl-msgs_0.0.1-1_amd64.deb

# 系统 Python 依赖 + rospython 别名
RUN /usr/bin/python3 -m pip install --upgrade pip \
    && /usr/bin/python3 -m pip install --break-system-packages --no-cache-dir \
    --retries 5 --timeout 60 \
    -i https://pypi.tuna.tsinghua.edu.cn/simple \
    pyzmq "numpy<2" "opencv-python<4.10" ikpy h5py \
    && ln -sf /usr/bin/python3 /usr/local/bin/rospython

# Fast-DDS config: disable shared memory (fixes ROS2 data delivery inside Docker)
COPY docker/fastdds_no_shm.xml /opt/fastdds_no_shm.xml

# Bridge2 is bind-mounted via /ubt_IL/tienkung/ros2_deploy_bridge.py (no COPY needed)
COPY docker/entrypoint.sh /opt/entrypoint.sh
RUN chmod +x /opt/entrypoint.sh

USER user_lerobot
WORKDIR /ubt_IL

ENTRYPOINT ["/opt/entrypoint.sh"]
CMD ["bash"]
