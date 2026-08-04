1. Build and Start the Container (Host Terminal 1)
Run these commands on your host machine in the directory containing your Dockerfile and workspace. This builds the image and spins up the container with the necessary GPU and X11 GUI forwarding flags.

Bash
# 1. Allow Docker to communicate with your host's display server
xhost +local:root

# 2. Build the Docker image (replace 'rover_sim' with your preferred image name)
docker build -t rover_sim .

# 3. Launch the container with GPU passthrough and volume mounting
docker run -it --rm --net=host --gpus all --privileged \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd):/workspace/Rover \
    rover_sim bash
2. Build the Workspace and Launch (Container Terminal 1)
Once inside the container, compile your ROS 2 workspace and spin up the simulation environment (Gazebo + RViz + Stereo Pipeline).

Bash
# 1. Navigate to the workspace
cd /workspace/Rover

# 2. Build the packages
colcon build --symlink-install
# 3. Source the ROS 2 underlay and your local workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# 4. Launch the simulation
ros2 launch rover_description stereo_sim.launch.py
3. Start Teleop (Host/Container Terminal 2)
To drive the rover, you need to open a second terminal, inject it into the running container, and start the keyboard teleop node.

Bash
# 1. Open a NEW terminal on your host machine and find the container ID
docker ps

# 2. Exec into the running container (replace <CONTAINER_ID> with the actual ID)
docker exec -it <CONTAINER_ID> bash

# 3. Source the workspace inside this new terminal session
cd /workspace/Rover
source /opt/ros/humble/setup.bash
source install/setup.bash

# 4. Run the standard ROS 2 keyboard teleop node
ros2 run teleop_twist_keyboard teleop_twist_keyboard
Use the I, J, K, L, and , keys in this second terminal to drive the rover. Since your launch file already bridges /cmd_vel from ROS 2 into Gazebo, the rover will start moving, and you will see the stereo point cloud shift in RViz as the cameras pan across the red and blue boxes.


# 5. Reset the point cloud memory (in a new terminal while rviz is running)
ros2 service call /rtabmap/reset std_srvs/srv/Empty

# 6. Reset costmaps
# global
ros2 service call /global_costmap/global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{}"
# local
ros2 service call /local_costmap/local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{}"