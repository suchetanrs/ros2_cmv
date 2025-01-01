FROM osrf/ros:jazzy-desktop-full

ENV ROS_DISTRO=jazzy
ENV ROS_WS=/root/ros2_ws

RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y \
    python3-colcon-common-extensions \
    build-essential \
    git \
    ranger \
    nano \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p $ROS_WS/src

WORKDIR $ROS_WS

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash"

CMD ["bash"]