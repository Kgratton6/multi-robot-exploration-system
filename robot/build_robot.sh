# /bin/bash
cd robot
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build
source install/setup.bash
ros2 launch robot robot.launch.py