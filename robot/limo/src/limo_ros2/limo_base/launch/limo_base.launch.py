import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'id', default_value='102robot1',
        description='Namespace for the robot'
    )

    port_name_arg = DeclareLaunchArgument('port_name', default_value='ttyTHS1',
                                          description='USB bus name, e.g. ttyTHS1')
    odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value='odom',
                                           description='Odometry frame ID')
    base_link_frame_arg = DeclareLaunchArgument('base_frame', default_value='base_link',
                                                description='Base link frame ID')
    odom_topic_arg = DeclareLaunchArgument('odom_topic_name', default_value='odom',
                                           description='Odometry topic name')
    odom_tf_arg = DeclareLaunchArgument('pub_odom_tf', default_value='true',
                                        description='Publish odometry TF (true/false)')
    sim_control_rate_arg = DeclareLaunchArgument('control_rate', default_value='50',
                                                 description='Simulation control loop update rate')

    # Define the Limo base node
    limo_base_node = Node(
        package='limo_base',
        executable='limo_base',
        output='screen',
        emulate_tty=True,
        namespace=LaunchConfiguration('id'),
        parameters=[{
            'port_name': LaunchConfiguration('port_name'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'odom_topic_name': [LaunchConfiguration('id'), '/', LaunchConfiguration('odom_topic_name')],
            'pub_odom_tf': LaunchConfiguration('pub_odom_tf'),
            'control_rate': LaunchConfiguration('control_rate'),
        }]
    )

    return LaunchDescription([
        namespace_arg,
        port_name_arg,
        odom_frame_arg,
        base_link_frame_arg,
        odom_topic_arg,
        odom_tf_arg,
        sim_control_rate_arg,
        limo_base_node
    ])
