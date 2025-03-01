# Copyright 2018 Open Source Robotics Foundation, Inc.
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

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import os
import launch.actions


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory(
        "nav2_autonomy")

    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="nav2_autonomy",
                executable="object_transform_publisher",
                name="object_transform_publisher",
                output="screen",
            ),
            launch_ros.actions.Node(
                package="nav2_autonomy",
                executable="sim_object_detector",
                name="sim_object_detector",
                output="screen",
            ),
        ]
    )
