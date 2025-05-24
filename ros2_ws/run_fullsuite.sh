colcon build --symlink-install
source install/setup.bash
ros2 launch controls controls.launch.py &
ros2 launch realsense2_camera rs_launch.py config_file:=realsense_params.yaml &
ros2 run sensing depth &

wait