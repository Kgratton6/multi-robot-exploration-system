source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --cmake-args -DBUILD_TESTING=ON

export LIBGL_ALWAYS_SOFTWARE=1

ros2 launch ros_gz_example_bringup diff_drive.launch.py
