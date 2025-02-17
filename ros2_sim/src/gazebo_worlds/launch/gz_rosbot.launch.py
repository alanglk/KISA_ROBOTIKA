#!/usr/bin/env python3

# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os
from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def launch_setup(context, *args, **kwargs):
    # Retrieve the arguments with context
    gz_headless_mode = LaunchConfiguration("gz_headless_mode").perform(context)
    gz_log_level = LaunchConfiguration("gz_log_level").perform(context)
    
    # Append ".sdf" to the world argument dynamically
    world_name = LaunchConfiguration("world").perform(context)
    gz_world = os.path.join(
        get_package_share_directory("gazebo_worlds"),
        "worlds",
        world_name + ".sdf"
    )
    # Set robot position for each world
    rosbot_pose_simple = ["-4.0", "-4.0", "0.0"]
    rosbot_pose_ellipse = ["0.0", "-2.0", "0.0"]
    rosbot_pose_square = ["-3.0", "-4.0", "0.0"]
    rosbot_pose_landmarks = ["3.0", "-3.0", "1.57079"]
    rosbot_pose_yahboom_track = ["-0.16", "-2.6", "-3.1416"]
    
    gz_args = f"-r -v {gz_log_level} {gz_world}"
    if eval(gz_headless_mode):
        gz_args = "--headless-rendering -s " + gz_args

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    # Check if the rosbot argument is set to True
    robot_name = LaunchConfiguration("rosbot").perform(context)

    # Get robot position from launch parameters, and if not exist set default values
    robot_pos_x = 0.0
    robot_pos_y = 0.0
    robot_pos_yaw = 0.0

    robot_launch_pos_x = LaunchConfiguration("x").perform(context)
    robot_launch_pos_y = LaunchConfiguration("y").perform(context)
    robot_launch_pos_yaw = LaunchConfiguration("yaw").perform(context)

    print(robot_launch_pos_x, robot_launch_pos_y, robot_launch_pos_yaw) 
    robot_include = None
    if robot_name == "True":
        if world_name == "simple":
                robot_pos_x = rosbot_pose_simple[0]
                robot_pos_y = rosbot_pose_simple[1]
                robot_pos_yaw = rosbot_pose_simple[2]
        elif world_name == "ellipse":
                robot_pos_x = rosbot_pose_ellipse[0]
                robot_pos_y = rosbot_pose_ellipse[1]
                robot_pos_yaw = rosbot_pose_ellipse[2]
        elif world_name == "square":
                robot_pos_x = rosbot_pose_square[0]
                robot_pos_y = rosbot_pose_square[1]
                robot_pos_yaw = rosbot_pose_square[2]
        elif world_name == "yahboom_track": 
                robot_pos_x = rosbot_pose_yahboom_track[0]
                robot_pos_y = rosbot_pose_yahboom_track[1]
                robot_pos_yaw = rosbot_pose_yahboom_track[2]
        elif world_name == "landmarks": 
                robot_pos_x = rosbot_pose_landmarks[0]
                robot_pos_y = rosbot_pose_landmarks[1]
                robot_pos_yaw = rosbot_pose_landmarks[2]

        if robot_launch_pos_x != "0.0":
            robot_pos_x = robot_launch_pos_x
        if robot_launch_pos_y != "0.0":
            robot_pos_y = robot_launch_pos_y
        if robot_launch_pos_yaw != "0.0":
            robot_pos_yaw = robot_launch_pos_yaw
        

        # Include robot launch file if robot is 'rosbot'
        robot_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('rosbot_gazebo'),
                    'launch/spawn.launch.py'),
            ),
            # Get robot position from launch parameters
            launch_arguments={
                "x": robot_pos_x,
                "y": robot_pos_y,
                "z": "0.0",
                "roll": "0.0",
                "pitch": "0.0",
                "yaw": robot_pos_yaw
            }.items(),
        )

    if robot_include:
        return [gz_sim, robot_include]
    else:
        return [gz_sim]


def generate_launch_description():

    # Declare arguments that can be set from the command line or defaults
    declare_world_arg = DeclareLaunchArgument(
        "world",
        default_value="simple_world",  # No .sdf here
        description="World name without the .sdf extension.",
    )
    
    declare_robot_arg = DeclareLaunchArgument(
        "rosbot",
        default_value="True",
        description="Name of the robot to be spawned (e.g., 'rosbot').",
    )

    declare_gz_headless_mode = DeclareLaunchArgument(
        "gz_headless_mode",
        default_value="False",
        description="Run the simulation in headless mode. Useful when a GUI is not needed or to reduce the amount of calculations.",
        choices=["True", "False"],
    )

    declare_gz_log_level = DeclareLaunchArgument(
        "gz_log_level",
        default_value="1",
        description="Adjust the level of console output.",
        choices=["0", "1", "2", "3", "4"],
    )

    declare_x_arg = DeclareLaunchArgument(
        "x",
        default_value="0.0",
        description="Initial x-position of the robot."
    )

    declare_y_arg = DeclareLaunchArgument(
        "y",
        default_value="0.0",
        description="Initial y-position of the robot."
    )

    declare_yaw_arg = DeclareLaunchArgument(
        "yaw",
        default_value="0.0",
        description="Initial yaw orientation of the robot in radians."
    )


    return LaunchDescription(
        [
            declare_world_arg,
            declare_robot_arg,  # Declare the robot argument
            declare_gz_headless_mode,
            declare_gz_log_level,
            declare_x_arg,
            declare_y_arg,
            declare_yaw_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )


