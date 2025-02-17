from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Define launch configurations from arguments
    background_r = LaunchConfiguration('background_r')
    background_g = LaunchConfiguration('background_g')
    background_b = LaunchConfiguration('background_b')
    # Declare launch arguments
    background_r_launch_arg = DeclareLaunchArgument(
        "background_r", default_value="0"
    )
    background_g_launch_arg = DeclareLaunchArgument(
        "background_g", default_value="255"
    )
    background_b_launch_arg = DeclareLaunchArgument(
        "background_b", default_value="0"
    )
    # turtlesim_node with parameters
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim',
        parameters=[{
            'background_r': background_r,
            'background_g': background_g,
            'background_b': background_b
        }]
    )
    # turtle_teleop_key node
    turtle_teleop_node = Node(
        package='turtlesim',
        executable='turtle_teleop_key',
        name='teleop',
        prefix=["xterm -e"],
        output="screen"
    )
    return LaunchDescription([
        # Include the launch arguments
        background_r_launch_arg,
        background_g_launch_arg,
        background_b_launch_arg,
        # Launch the nodes
        turtlesim_node,
        turtle_teleop_node
    ])
