import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        # LED controller node
        Node(
            package='ros_gz_peripherals',
            executable='led_controller',
            name='led_controller_node',
            output='screen',
        ),
        # Sound controller node
        Node(
            package='ros_gz_peripherals',
            executable='sound_controller',
            name='sound_controller_node',
            output='screen',
        ),
    ])
