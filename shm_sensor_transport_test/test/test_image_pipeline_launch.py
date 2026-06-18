# Copyright 2026 Maik Knof
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

import socket
import threading
import time

import launch
import pytest
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from shm_sensor_transport_py import ShmSubscriber
from shm_sensor_transport_py.loaders import RosImageLoader


PAYLOAD_SIZE = 4096
FRAME_COUNT = 20
NORMAL_TOPIC = '/pipeline/normal/image_raw'
SHM_TOPIC = '/pipeline/shm/image_raw'


def expected_payload(sequence: int) -> bytes:
    return bytes(((sequence + index) % 251 for index in range(PAYLOAD_SIZE)))


def make_container(mode: str):
    nodes = [
        ComposableNode(
            package='shm_sensor_transport_test',
            plugin='shm_sensor_transport_test::BenchmarkImagePublisherComponent',
            name=f'{mode}_pipeline_publisher',
            parameters=[{
                'topic': NORMAL_TOPIC if mode == 'normal' else SHM_TOPIC,
                'frames': FRAME_COUNT,
                'payload_size': PAYLOAD_SIZE,
                'rate_hz': 40.0,
                'start_delay_sec': 0.5,
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        )
    ]
    if mode == 'shm':
        nodes.append(
            ComposableNode(
                package='shm_sensor_transport',
                plugin='shm_sensor_transport::ShmImageRelayComponent',
                name='shm_pipeline_relay',
                parameters=[{
                    'common.input_topic': SHM_TOPIC,
                    'common.shm_name': '/ros2_shm_pipeline_test',
                    'common.slot_count': 4,
                    'common.slot_size_bytes': PAYLOAD_SIZE,
                    'common.publish_status': False,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        )

    return ComposableNodeContainer(
        name=f'{mode}_pipeline_test_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=nodes,
        output='screen',
    )


class PipelineProbe(Node):
    """Validate normal and shared-memory payloads from C++ component publishers."""

    def __init__(self, mode: str):
        super().__init__(f'shm_image_{mode}_pipeline_probe')
        self.mode = mode
        self.received = []
        if mode == 'normal':
            self.subscription = self.create_subscription(
                Image,
                NORMAL_TOPIC,
                self._on_normal_image,
                qos_profile_sensor_data,
            )
        elif mode == 'shm':
            self.subscription = [
                ShmSubscriber(
                    node=self,
                    topic=SHM_TOPIC,
                    loader=RosImageLoader(),
                    callback=lambda msg, _meta: self._on_shm_payload(0, msg),
                ),
                ShmSubscriber(
                    node=self,
                    topic=SHM_TOPIC,
                    loader=RosImageLoader(),
                    callback=lambda msg, _meta: self._on_shm_payload(1, msg),
                ),
            ]
        else:
            raise ValueError(f'unknown mode: {mode}')

    def complete(self) -> bool:
        if self.mode == 'normal':
            return 0 in self.received
        return 0 in self.received and 1 in self.received

    def _on_normal_image(self, msg: Image) -> None:
        if self._valid_payload(msg):
            self.received.append(0)

    def _on_shm_payload(self, index: int, msg: Image) -> None:
        if self._valid_payload(msg):
            self.received.append(index)

    @staticmethod
    def _valid_payload(msg: Image) -> bool:
        try:
            sequence = int(msg.header.frame_id)
        except ValueError:
            return False
        return bytes(msg.data) == expected_payload(sequence)


def run_phase(mode: str) -> PipelineProbe:
    launch_service = launch.LaunchService()
    launch_service.include_launch_description(launch.LaunchDescription([make_container(mode)]))
    probe = PipelineProbe(mode)
    executor = SingleThreadedExecutor()
    executor.add_node(probe)

    def spin_until_done() -> None:
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < deadline and not probe.complete():
            executor.spin_once(timeout_sec=0.05)
        launch_service.shutdown()

    spin_thread = threading.Thread(target=spin_until_done, daemon=True)
    spin_thread.start()
    try:
        launch_service.run()
    finally:
        spin_thread.join(timeout=5.0)
        executor.remove_node(probe)
    return probe


def require_udp_transport() -> None:
    try:
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.bind(('127.0.0.1', 0))
    except OSError as error:
        pytest.skip(f'ROS launch integration requires UDP sockets: {error}')
    finally:
        try:
            udp_socket.close()
        except UnboundLocalError:
            pass


def test_cpp_component_publishers_to_python_subscribers():
    require_udp_transport()
    rclpy.init()
    try:
        normal_probe = run_phase('normal')
        try:
            assert normal_probe.complete()
        finally:
            normal_probe.destroy_node()

        shm_probe = run_phase('shm')
        try:
            assert shm_probe.complete()
        finally:
            shm_probe.destroy_node()
    finally:
        rclpy.shutdown()
