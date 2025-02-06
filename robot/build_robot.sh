# /bin/bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch limo_base limo_base.launch.py # build the robot