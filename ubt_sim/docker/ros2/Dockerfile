FROM swr.cn-north-4.myhuaweicloud.com/ddn-k8s/docker.io/ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=146

# 先安装 ca-certificates，再替换 apt 为清华源
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates \
    && sed -i 's|http://archive.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list \
    && sed -i 's|http://security.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list \
    && apt-get update && rm -rf /var/lib/apt/lists/*

# 添加 ROS2 源（清华 TUNA 镜像）
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    curl \
    && add-apt-repository -y universe \
    && curl -sSL https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list \
    && rm -rf /var/lib/apt/lists/*

# 安装 ROS2 Humble 及项目依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-cv-bridge \
    ros-humble-message-filters \
    python3-pip \
    g++ make libzmq3-dev \
    iproute2 \
    && rm -rf /var/lib/apt/lists/*

# 安装 Python 依赖（必须使用系统 Python）
RUN /usr/bin/python3 -m pip install --no-cache-dir \
    -i https://pypi.tuna.tsinghua.edu.cn/simple \
    'numpy<2' pyzmq h5py ikpy 'opencv-python-headless<4.10' pyyaml empy lark protobuf

# 安装预编译的 bodyctrl 消息包
COPY teleoperation/msgs/ros-humble-bodyctrl-msgs_0.0.1-1_amd64.deb /tmp/
RUN apt-get update && apt-get install -y /tmp/ros-humble-bodyctrl-msgs_0.0.1-1_amd64.deb \
    && rm /tmp/*.deb && rm -rf /var/lib/apt/lists/*

WORKDIR /ubt_sim

# 入口：source ROS2 环境（bodyctrl 消息已通过 deb 安装到 /opt/ros/humble）
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
