#!/bin/bash
set -e

echo "Setting up Intel RealSense RGBD Camera for ROS 2 Jazzy over WiFi..."

# Check if the package is installed
if ! dpkg -l | grep -q ros-jazzy-realsense2-camera; then
    echo "Installing realsense2_camera package (requires sudo)..."
    sudo apt-get update
    sudo apt-get install -y ros-jazzy-realsense2-camera
else
    echo "realsense2_camera package is already installed."
fi

# Check and install udev rules for RealSense
if [ ! -f "/etc/udev/rules.d/99-realsense-libusb.rules" ]; then
    echo "RealSense udev rules not found. Installing them (requires sudo)..."
    sudo curl -sSL https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules -o /etc/udev/rules.d/99-realsense-libusb.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "========================================================="
    echo " PLEASE UNPLUG AND RE-PLUG YOUR REALSENSE CAMERA NOW! "
    echo " After doing so, press Enter to continue..."
    echo "========================================================="
    read -p ""
fi

# Ensure ROS_DOMAIN_ID is configured
if [ -z "$ROS_DOMAIN_ID" ]; then
    export ROS_DOMAIN_ID=42
    echo "Set ROS_DOMAIN_ID=42 for this terminal (defaulting to 42 for WiFi discovery)."
else
    echo "Using existing ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
fi

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

echo "Starting the RealSense camera node..."
echo "To view in RViz over WiFi on your client machine:"
echo " 1. Make sure your client machine is on the same WiFi network."
echo " 2. Set the same ROS_DOMAIN_ID on the client (export ROS_DOMAIN_ID=$ROS_DOMAIN_ID)."
echo " 3. Run RViz2 and add the /camera/camera/depth/image_rect_raw or /camera/camera/depth/color/points topic."

# Run the camera node with depth and pointcloud enabled
ros2 launch realsense2_camera rs_launch.py \
    depth_module.depth_profile:=640x480x30 \
    rgb_camera.color_profile:=640x480x30 \
    enable_pointcloud:=true \
    align_depth.enable:=true \
    pointcloud.enable:=true
