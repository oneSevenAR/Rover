FROM osrf/ros:humble-desktop

# Install all simulation, SLAM, mapping, stereo pipelines, and GPU testing packages
RUN apt-get update && apt-get install -y \
    ros-humble-joint-state-publisher \
    ros-humble-slam-toolbox \
    ros-humble-teleop-twist-keyboard \
    ros-humble-nav2-map-server \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-ros-gz \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-image-pipeline \
    ros-humble-rtabmap-ros \
    ros-humble-rtabmap-slam \
    ros-humble-robot-localization \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Default to your workspace folder when the container starts
WORKDIR /workspace/Rover