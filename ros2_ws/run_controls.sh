#!/bin/bash

colcon build --packages-select controls
source install/setup.bash
ros2 launch controls controls.launch.py