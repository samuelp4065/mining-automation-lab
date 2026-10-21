FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-realsense2-camera \
    ros-humble-rtabmap-ros \
    ros-humble-imu-filter-madgwick \
    ros-humble-robot-localization \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-depthimage-to-laserscan \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-rmw-cyclonedds-cpp \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /ws
CMD ["bash"]
