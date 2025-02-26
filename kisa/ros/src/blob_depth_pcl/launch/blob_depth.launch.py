from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='blob_depth_pcl',
            executable='server',
            name='depth_server',
            parameters=[{'depth_pcl_topic': '/depth/color/points'},
                       {'image_width': 320},
                       {'image_height': 240}
                       ],
            output='screen',                       
        ),
        Node(
            package='blob_depth_pcl',
            executable='client',
            name='depth_client',
            parameters=[{'image_topic': '/color/image_raw'},
                       {'image_width': 320},
                       {'image_height': 240}
                       ],
            output='screen',
        ),
    ])
