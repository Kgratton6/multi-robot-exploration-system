from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    robot_speed_arg = DeclareLaunchArgument(
        'robot_speed',
        default_value='0.2', 
        description='Vitesse pour le move_controller'
    )
    
    # Move controllers pour les 2 robots
    move_controller_robot1 = Node(
        package='control',
        executable='move_controller',
        name='move_controller_robot1',
        output='screen',
        parameters=[{'speed': LaunchConfiguration('robot_speed'), 'robot_id': 'robot1'}],
        remappings=[('/cmd_vel', '/robot1/cmd_vel')]
    )
    move_controller_robot2 = Node(
        package='control',
        executable='move_controller',
        name='move_controller_robot2',
        output='screen',
        parameters=[{'speed': LaunchConfiguration('robot_speed'), 'robot_id': 'robot2'}],
        remappings=[('/cmd_vel', '/robot2/cmd_vel')]
    )
    
    # Nœud de communication central
    communication_controller_node = Node(
        package='communication',
        executable='communication_controller',
        name='communication_controller',
        output='screen'
    )
    
    # Nœuds d'identification pour les 2 robots
    identify_node_robot1 = Node(
        package='identification',
        executable='identify_node',
        name='identify_node_robot1',
        output='screen',
        parameters=[{'robot_id': 'robot1'}]
    )
    identify_node_robot2 = Node(
        package='identification',
        executable='identify_node',
        name='identify_node_robot2',
        output='screen',
        parameters=[{'robot_id': 'robot2'}]
    )
    
    ld = LaunchDescription()
    ld.add_action(robot_speed_arg)
    ld.add_action(move_controller_robot1)
    ld.add_action(move_controller_robot2)
    ld.add_action(communication_controller_node)
    ld.add_action(identify_node_robot1)
    ld.add_action(identify_node_robot2)

    return ld
