#!/bin/bash
set -e

echo "Setting up ROS 2 Jazzy rover environment..."

# 1. Install dependencies
sudo apt update
sudo apt install -y \
    ros-jazzy-ros-base \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-robot-localization \
    ros-jazzy-xacro \
    python3-serial \
    i2c-tools \
    build-essential

# 2. Setup workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 3. Clone sllidar_ros2
if [ ! -d "sllidar_ros2" ]; then
    echo "Cloning sllidar_ros2..."
    git clone https://github.com/Slamtec/sllidar_ros2.git
else
    echo "sllidar_ros2 already cloned."
fi

# 4. Setup Lidar udev rules
echo "Setting up Lidar udev rules..."
sudo cp ~/ros2_ws/src/sllidar_ros2/scripts/rplidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# 5. Update .bashrc
echo "Updating ~/.bashrc..."
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

if ! grep -q "source ~/ros2_ws/install/setup.bash" ~/.bashrc; then
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
fi

if ! grep -q "export ROS_DOMAIN_ID=42" ~/.bashrc; then
    echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
fi

echo "=================================================="
echo "         Rover Environment Setup Complete!        "
echo "=================================================="
echo "Next steps:"
echo "1. Source your terminal: source ~/.bashrc"
echo "2. Build the workspace: cd ~/ros2_ws && colcon build"
echo "3. Source the install: source install/setup.bash"
echo "4. Launch: ros2 launch rover_bringup robot_launch.py"
