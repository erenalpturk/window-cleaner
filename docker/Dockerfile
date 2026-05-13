FROM ros:humble

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rviz2 \
        ros-humble-rqt \
        ros-humble-rqt-common-plugins \
        ros-humble-rqt-image-view \
        ros-humble-nav2-bringup \
        ros-humble-navigation2 \
        ros-humble-ros-gz \
        ros-humble-cv-bridge \
        ros-humble-image-transport \
        ros-humble-image-transport-plugins \
        ros-humble-vision-opencv \
        ros-humble-xacro \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-publisher-gui \
        ros-humble-robot-state-publisher \
        ros-humble-teleop-twist-keyboard \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-tf2-tools \
        ros-humble-tf2-ros \
        ros-humble-foxglove-bridge \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep \
        sudo \
        git \
        vim \
        less \
        wget \
        curl \
        mesa-utils \
        libgl1-mesa-glx \
        libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
        opencv-python \
        numpy \
        scipy \
        matplotlib

RUN groupadd --gid ${USER_GID} ${USERNAME} 2>/dev/null || true \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} -s /bin/bash \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /workspace

COPY --chmod=755 entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
