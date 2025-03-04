#import os
#from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
        return LaunchDescription([
		Node(
		    package="occgrid",
		    executable="occgrid_mapping",
		    parameters=[
		        {"laser_topic": "/scan"},
				{"odom_topic": "/rosbot_base_controller/odom"},
                {"laser_frame": "laser"},
				{"msize_x": 22.0},
				{"msize_y": 22.0},
				{"mresolution": 0.05},
				{"p_occ": 0.75},
				{"p_free": 0.45},
				{"p_prior": 0.5},
				{"k": 2}
		    ],
		    output='screen',
		    emulate_tty=True,
		)
        ])


    