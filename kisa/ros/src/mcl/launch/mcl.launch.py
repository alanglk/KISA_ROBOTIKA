import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from launch_ros.substitutions import FindPackageShare

from launch.actions import ExecuteProcess

from glob import glob

def generate_launch_description():
    ld = LaunchDescription()
    map_file_path = os.path.join(
        get_package_share_directory('mcl'),
        'maps',
        'simple.yaml'
    )

    # parameters for the mcl node and the map server
    mcl_config = os.path.join(
        get_package_share_directory('mcl'),
        'config',
        'mcl.yaml'
    )

    map_server_cmd = Node(
            package='nav2_map_server',
            namespace = '',
            executable='map_server',
            parameters=[{'yaml_filename': map_file_path},
                        {'use_sim_time': True}],   
        )
    
    lifecycle_nodes = ['map_server']#, 'amcl']
    use_sim_time = True
    autostart = True

    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])

    map2odom_pub = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=[
            "0", "0", "0",          # Translation (x, y, z)
            "0", "0", "0", "1",     # Quaternion (qx, qy, qz, qw)
            "map",                  # Parent frame (frame_id)
            "odom"                  # Child frame (child_frame_id)
        ]
    )
    
    odom2map_node = Node(
        package = 'mcl',
        name='odom2map_node',
        executable = 'odom2map_node',
        # parameters = [mcl_config],
        output = "screen"
    )

    mcl_node = Node(
        package = 'mcl',
        name='mcl_node',
        executable = 'mcl_node',
        parameters = [mcl_config],
        output = "screen"
    )

    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory('mcl'), 'rviz', 'mcl.rviz')],
    )

    
    ld.add_action(start_lifecycle_manager_cmd)
    #static map to odom publisher
    ld.add_action(map2odom_pub)

    # The map server is defined in the params.yaml file. 
    ld.add_action(map_server_cmd)    
    ld.add_action(mcl_node)
    
    ld.add_action(rviz)
    return ld

