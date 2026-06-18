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

import os

from sensor_msgs.msg import Image, PointCloud2

from shm_sensor_transport_py.publisher import ShmPublisher
from shm_sensor_transport_py.shm_handle import ShmHandle


class FakeRosPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeNode:
    def __init__(self):
        self.publishers = []

    def create_publisher(self, msg_type, topic, qos_profile):
        publisher = FakeRosPublisher()
        self.publishers.append((msg_type, topic, qos_profile, publisher))
        return publisher


def test_image_publisher_writes_payload_readable_by_handle():
    node = FakeNode()
    publisher = ShmPublisher(
        node,
        "/camera/image_raw",
        msg_type=Image,
        slot_count=2,
        slot_size_bytes=16,
        shm_name=f"/ros2_shm_py_image_{os.getpid()}",
    )

    image = Image()
    image.header.frame_id = "camera"
    image.height = 1
    image.width = 4
    image.encoding = "mono8"
    image.step = 4
    image.data = [1, 2, 3, 4]

    assert publisher.publish(image)
    assert ShmHandle().copy_payload(publisher.last_metadata) == bytes([1, 2, 3, 4])
    assert node.publishers[0][1] == "/camera/image_raw/_shm"
    publisher.close()


def test_pointcloud2_publisher_writes_payload_readable_by_handle():
    node = FakeNode()
    publisher = ShmPublisher(
        node,
        "/points",
        msg_type=PointCloud2,
        slot_count=2,
        slot_size_bytes=16,
        shm_name=f"/ros2_shm_py_cloud_{os.getpid()}",
    )

    cloud = PointCloud2()
    cloud.header.frame_id = "lidar"
    cloud.height = 1
    cloud.width = 1
    cloud.point_step = 4
    cloud.row_step = 4
    cloud.is_dense = True
    cloud.data = [5, 6, 7, 8]

    assert publisher.publish(cloud)
    assert ShmHandle().copy_payload(publisher.last_metadata) == bytes([5, 6, 7, 8])
    assert node.publishers[0][1] == "/points/_shm"
    publisher.close()
