import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from launch_ros.substitutions import FindPackageShare

from glob import glob

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('ekf_localization'),
        'config',
        'params.yaml'
    )
    landmark_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('landmarks'), 
            '/launch', '/landmarks.launch.py'])
    )    
    ekf_node = Node(
        package = 'ekf_localization',
        name='ekf_loc_unknown',
        executable = 'ekf_loc_unknown',
        parameters = [config],
        output = "screen"
    )
    vis_node = Node(
        package = 'ekf_localization',
        name='ekfloc_visualizer',
        executable = 'ekfplot',
        parameters = [config],
        output = 'screen'
        
    )
    ld.add_action(landmark_detection)
    ld.add_action(ekf_node)
    ld.add_action(vis_node)
    return ld