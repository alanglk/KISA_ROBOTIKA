# Copyright 2021 Open Source Robotics Foundation, Inc.
# Copyright 2023 Intel Corporation. All Rights Reserved.
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

import launch_pytest
import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from test_utils import BringupTestNode

robot_names = ["robot1", "robot2", "robot3"]


@launch_pytest.fixture
def generate_test_description():
    rosbot_bringup = FindPackageShare("rosbot_bringup")
    actions = []
    for i in range(len(robot_names)):
        bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        rosbot_bringup,
                        "launch",
                        "bringup.launch.py",
                    ]
                )
            ),
            launch_arguments={
                "use_sim": "False",
                "mecanum": "False",
                "use_gpu": "False",
                "namespace": robot_names[i],
            }.items(),
        )

        actions.append(bringup_launch)

    return LaunchDescription(actions)


@pytest.mark.launch(fixture=generate_test_description)
def test_multirobot_bringup_startup_success():
    for robot_name in robot_names:
        rclpy.init()
        try:
            node = BringupTestNode("test_bringup", namespace=robot_name)
            node.create_test_subscribers_and_publishers()
            node.start_publishing_fake_hardware()

            node.start_node_thread()
            msgs_received_flag = node.odom_msg_event.wait(timeout=20.0)
            assert msgs_received_flag, (
                f"Expected {robot_name}/odometry/filtered message but it was not received. "
                "Check robot_localization!"
            )

        finally:
            rclpy.shutdown()
