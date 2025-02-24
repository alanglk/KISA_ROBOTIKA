#import os
#from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	blob_segmentation = Node(
		package="blob_segmentation",
		executable="segment_blob",
		parameters=[
		{"image_topic": "color/image_raw"},
		{"color_pose_topic": "blob_segment/color_pose"},
		],
		output='screen',
		emulate_tty=True,
	)

	return LaunchDescription([ blob_segmentation ])
    
