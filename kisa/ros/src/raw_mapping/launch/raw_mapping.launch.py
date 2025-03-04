#import os
#from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        return LaunchDescription([
		Node(
		    package="raw_mapping",
		    executable="raw_mapping_node",
		    parameters=[
		        {"laser_topic": "/scan"},
				{"odom_topic": "/rosbot_base_controller/odom"},
                {"laser_frame": "laser"},
                {"rplidar_type": "s2"},
		    ],
		    output='screen',
		    emulate_tty=True,
		)
        ])
    
