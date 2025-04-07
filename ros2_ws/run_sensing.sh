#!/bin/bash

# Build all packages
colcon build --symlink-install

# Source the build
source install/local_setup.bash

# Run first package
ros2 launch realsense2_camera rs_launch.py &

# Run the second package in the background
ros2 run sensing testing &

# Wait for all background processes to finish
wait

