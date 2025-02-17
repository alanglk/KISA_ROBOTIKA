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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution


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

    gz_args = f"-r -v {gz_log_level} {gz_world}"
    if eval(gz_headless_mode):
        gz_args = "--headless-rendering -s " + gz_args

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    return [gz_sim]


def generate_launch_description():
    # Declare arguments that can be set from the command line or defaults
    declare_world_arg = DeclareLaunchArgument(
        "world",
        default_value="simple_world",  # No .sdf here
        description="World name without the .sdf extension.",
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

    return LaunchDescription(
        [
            declare_world_arg,
            declare_gz_headless_mode,
            declare_gz_log_level,
            OpaqueFunction(function=launch_setup),
        ]
    )

