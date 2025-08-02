# syntax=docker/dockerfile:1
FROM ros:humble
ENV DEBIAN_FRONTEND=noninteractive
ARG ROS_WS=/workspace/ros2_ws

# 1) 基本ツール
RUN apt-get update -qq && apt-get install -y --no-install-recommends \
        git curl gnupg2 lsb-release \
        python3-pip python3-rosdep \
        python3-colcon-common-extensions \
        build-essential && \
    rm -rf /var/lib/apt/lists/*
RUN pip install "numpy>=2.0" \
    pip install "stable-baselines3[extra]" \
    pip install "mujoco" \
    pip install --upgrade stable_baselines3 matplotlib
# 2) GPG 鍵更新（2025-05-28 rollover）
RUN set -eux; \
    apt-key del F42ED6FBAB17C654 || true; \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg; \
    apt-get update -qq

# 3) rosdep
RUN test -f /etc/ros/rosdep/sources.list.d/20-default.list || rosdep init && \
    rosdep update

# 4) CRANE-X7 Humble ブランチ取得
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}/src
RUN git clone -b humble https://github.com/rt-net/crane_x7_ros.git && \
    git clone -b humble https://github.com/rt-net/crane_x7_description.git

# 5) 依存解決 & ビルド
#    RealSense/Gazebo は deb が無いので必要なら後でソース追加
ENV SKIP_KEYS="ros_gz gz_ros2_control realsense2_description realsense2_camera"
SHELL ["/bin/bash", "-c"]
WORKDIR ${ROS_WS}
RUN source /opt/ros/humble/setup.bash && \
    rosdep install -r -y -i --from-paths . --rosdistro humble --skip-keys "$SKIP_KEYS" && \
    colcon build --symlink-install && \
    apt-get clean && rm -rf /var/lib/apt/lists/* ~/.cache

# 6) 起動時に source
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /workspace/ros2_ws/install/setup.bash' >> /root/.bashrc

WORKDIR /workspace
CMD ["bash"]
