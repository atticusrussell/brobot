#!/bin/bash
set -e

WORKSPACE_DIR=$(pwd)

# install rplidar ros2 drivers
sudo apt install -y ros-$ROS_DISTRO-rplidar-ros
cd /tmp
wget https://raw.githubusercontent.com/allenh1/rplidar_ros/ros2/scripts/rplidar.rules
sudo cp rplidar.rules /etc/udev/rules.d/

# Make colcon ignore non-robot packages (simulation and X11 dependencies)
cd "$WORKSPACE_DIR"
touch src/brobot_gazebo/COLCON_IGNORE
touch src/brobot_viz/COLCON_IGNORE
touch src/catbot_simulation/COLCON_IGNORE


# Download and install micro-ROS
cd "$WORKSPACE_DIR"
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup src/micro_ros_setup
sudo apt install python3-vcstool build-essential
sudo apt update && rosdep update
rosdep install --from-path src --ignore-src -y
colcon build --packages-skip brobot brobot_base brobot_bringup brobot_description brobot_navigation
source install/setup.bash

# Setup micro-ROS agent
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/setup.bash

# Install brobot packages
rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent
colcon build
source install/setup.bash
