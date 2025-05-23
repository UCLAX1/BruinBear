#!/bin/bash

# Build all packages
colcon build --symlink-install

# Source the build
source install/local_setup.bash

# Run first package
# CHANGE CONFIG FILE PATH FOR YOUR MACHINE
ros2 launch realsense2_camera rs_launch.py config_file:=/home/sara/Documents/BruinBear/ros2_ws/realsense_params.yaml &

# Run the second package in the background
ros2 run sensing depth &

# ros2 run sensing face &

# Wait for all background processes to finish
wait

