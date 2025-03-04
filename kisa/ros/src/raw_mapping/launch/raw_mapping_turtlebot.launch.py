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
				{"odom_topic": "odom"},
                {"laser_frame": "laser_link"},
                {"rplidar_type": "turtle"},
		    ],
		    output='screen',
		    emulate_tty=True,
		)
        ])
    
