# 引数でHumble とIronを選べるようにする
ARG ROS_DISTRO=humble
FROM althack/ros2:${ROS_DISTRO}-base

# 言語設定を聞かれないようにする
ARG DEBIAN_FRONTEND=noninteractive

# 必要なパッケージをインストール（nano追加）
RUN apt-get update \
 && apt-get install -y \
    build-essential \
    cmake \
    git-all \
    python3-pip \
    nano \
 && rm -rf /var/lib/apt/lists/*

# librealsenseとrealsense-rosをインストール
RUN apt-get update \
 && apt-get install -y \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-realsense2-camera \
 && rm -rf /var/lib/apt/lists/*

# デバッグ用のツールをインストール
RUN apt-get update \
 && apt-get install -y \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt-image-view \
 && rm -rf /var/lib/apt/lists/*

# 🔽 colcon インストールを追加
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*


# GPU対応PyTorchと他Pythonパッケージのインストール
RUN pip3 install numpy==1.24.4 \
    torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu118 \
    ultralytics opencv-python

RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> ~/.bashrc

CMD ["bash"]
