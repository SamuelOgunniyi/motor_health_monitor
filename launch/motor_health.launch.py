# Copyright 2025 Samuel Ogunniyi
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    package_dir = get_package_share_directory('motor_health_monitor')
    config_file = os.path.join(package_dir, 'config', 'motor_health.yaml')

    return LaunchDescription([
        ComposableNodeContainer(
            name='motor_health_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='motor_health_monitor',
                    plugin='motor_health_monitor::MotorHealthNode',
                    name='motor_health_monitor',
                    parameters=[config_file] if os.path.exists(config_file) else []
                )
            ],
            output='screen'
        )
    ])
