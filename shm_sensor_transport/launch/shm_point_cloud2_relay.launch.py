# Copyright 2026 Maik Knof
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    input_topic = LaunchConfiguration('input_topic')

    container = ComposableNodeContainer(
        name='shm_point_cloud2_relay_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='shm_sensor_transport',
                plugin='shm_sensor_transport::ShmPointCloud2RelayComponent',
                name='shm_point_cloud2_relay',
                parameters=[
                    {
                        'common.input_topic': input_topic,
                    }
                ],
            )
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('input_topic', default_value='/points'),
            container,
        ]
    )
