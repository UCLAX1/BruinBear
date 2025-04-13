#!/bin/bash

export PYTHONPATH=/home/yourname/venv/lib/python3.10/site-packages:$PYTHONPATH
export ROS_PYTHON_VERSION=3
export ROS_PYTHON_INTERPRETER=/home/yourname/venv/bin/python3.10
colcon build --packages-select controls
source install/setup.bash
ros2 launch controls controls.launch.py