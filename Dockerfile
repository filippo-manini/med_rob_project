FROM ros:humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    git build-essential nano \
    python3-colcon-common-extensions python3-rosdep \
    ros-humble-rviz2 ros-humble-rqt ros-humble-tf2-ros ros-humble-tf2-tools \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
 && echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> /root/.bashrc

WORKDIR /workspace
