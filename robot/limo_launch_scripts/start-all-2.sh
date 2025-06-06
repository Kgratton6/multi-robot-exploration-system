# /bin/bash

cd ..
cd limo || exit 1

pacmd set-default-sink alsa_output.usb-0c76_USB_PnP_Audio_Device-00.analog-stereo
export ROS_DOMAIN_ID=102
sudo chmod 666 /dev/ttyTHS1

source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build
source install/setup.bash

ros2 launch limo_bringup limo_start.launch.py id:=limo2 pub_odom_tf:=false & sleep 7

ros2 launch limo_bringup cartographer2.launch.py id:=limo2 & sleep 5

ros2 launch limo_bringup navigation22.launch.py id:=limo2

