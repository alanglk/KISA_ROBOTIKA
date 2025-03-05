from launch import LaunchDescription
from launch_ros.actions import Node
import numpy as np
angle_increment = 0.003929446917027235
def generate_launch_description():
    
    return LaunchDescription([
        # scan_angle_inc: 0.1745 rads equivalent to 1 deg
        # scan_angle_inc: 0.003929446917027235 original (4 readings per degree)
        Node(
            package='landmarks',
            executable='landmarks_node',
            name='landmarks_node',
            parameters=[{'frame_id': 'base_link'},
                        {'laser_count': 1600},
                        {'beam_angle_min': -np.pi},
                        {'beam_angle_max': np.pi},
                        {'scan_angle_inc': 0.003926991}, #pi/800
                        {'range_min': 0.1},
                        {'range_max': 3.5}
                        ],
            # output='screen',
        ),
        Node(
            package='landmarks',
            executable='draw_map',
            name='draw_map_node',
            output='screen'
        )
    ])