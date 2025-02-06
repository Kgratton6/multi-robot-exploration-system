# À COMPLETER

source /opt/ros/humble/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
colcon build --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
# launch la simul = ros2 launch ros_gz_example_bringup diff_drive.launch.py