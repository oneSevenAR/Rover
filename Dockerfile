FROM osrf/ros:humble-desktop

# Prevent apt-get from freezing the build with interactive user prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies (Alphabetized for easy reading and duplicate prevention)
RUN apt-get update && apt-get install -y --no-install-recommends \
    mesa-utils \
    ros-humble-filters \
    ros-humble-grid-map-core \
    ros-humble-grid-map-filters \
    ros-humble-grid-map-msgs \
    ros-humble-grid-map-ros \
    ros-humble-grid-map-rviz-plugin \
    ros-humble-image-pipeline \
    ros-humble-joint-state-publisher \
    ros-humble-message-filters \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server \
    ros-humble-navigation2 \
    ros-humble-pcl-conversions \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-rclcpp-components \
    ros-humble-robot-localization \
    ros-humble-ros-gz \
    ros-humble-rtabmap-ros \
    ros-humble-rtabmap-slam \
    ros-humble-slam-toolbox \
    ros-humble-teleop-twist-keyboard \
    ros-humble-tf2-eigen \
    ros-humble-spatio-temporal-voxel-layer \
    && rm -rf /var/lib/apt/lists/*

# Default to your workspace folder when the container starts
WORKDIR /workspace/Rover

# Automatically source ROS 2 and your workspace so you don't have to do it manually
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "if [ -f /workspace/Rover/install/setup.bash ]; then source /workspace/Rover/install/setup.bash; fi" >> /root/.bashrc

# Ensure we drop into a bash shell upon container launch
CMD ["bash"]