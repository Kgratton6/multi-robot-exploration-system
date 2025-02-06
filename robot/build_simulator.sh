source /opt/ros/humble/setup.bash
colcon build --cmake-args -DBUILD_TESTING=ON
source install/setup.bash

export LIBGL_ALWAYS_SOFTWARE=1

ros2 launch ros_gz_example_bringup diff_drive.launch.py
