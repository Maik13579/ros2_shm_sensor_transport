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

import pytest
from sensor_msgs.msg import CompressedImage, Image, PointCloud2

from shm_sensor_transport_py.loaders import (
    RosCompressedImageLoader,
    RosImageLoader,
    RosPointCloud2Loader,
)
from shm_sensor_transport_py.publisher import ShmPublisher
from shm_sensor_transport_py.shm_handle import ShmHandle
from shm_sensor_transport_py.subscriber import ShmSubscriber


class FakeRosPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeNode:
    def __init__(self):
        self.publishers = []
        self.subscriptions = []
        self.logger = FakeLogger()

    def create_publisher(self, msg_type, topic, qos_profile):
        publisher = FakeRosPublisher()
        self.publishers.append((msg_type, topic, qos_profile, publisher))
        return publisher

    def create_subscription(self, msg_type, topic, callback, qos_profile):
        self.subscriptions.append((msg_type, topic, callback, qos_profile))
        return self.subscriptions[-1]

    def get_logger(self):
        return self.logger


class FakeLogger:
    def __init__(self):
        self.debug_messages = []
        self.exception_messages = []

    def debug(self, message):
        self.debug_messages.append(message)

    def exception(self, message):
        self.exception_messages.append(message)


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


def test_compressed_image_publisher_writes_payload_readable_by_handle():
    node = FakeNode()
    publisher = ShmPublisher(
        node,
        "/camera/image_raw/compressed",
        msg_type=CompressedImage,
        slot_count=2,
        slot_size_bytes=32,
        shm_name=f"/ros2_shm_py_compressed_image_{os.getpid()}",
    )

    image = CompressedImage()
    image.header.frame_id = "camera"
    image.format = "jpeg"
    image.data = [255, 216, 1, 2, 255, 217]

    assert publisher.publish(image)
    assert ShmHandle().copy_payload(publisher.last_metadata) == bytes([255, 216, 1, 2, 255, 217])
    assert publisher.last_metadata.format == "jpeg"
    assert node.publishers[0][1] == "/camera/image_raw/compressed/_shm"
    publisher.close()


def test_image_publisher_feeds_subscriber_callback():
    node = FakeNode()
    publisher = ShmPublisher(
        node,
        "/camera/direct_image",
        msg_type=Image,
        slot_count=2,
        slot_size_bytes=16,
        shm_name=f"/ros2_shm_py_direct_image_{os.getpid()}",
    )
    received = []
    subscriber = ShmSubscriber(
        node=node,
        topic="/camera/direct_image",
        loader=RosImageLoader(),
        callback=lambda msg, meta: received.append((msg, meta)),
    )

    image = Image()
    image.header.frame_id = "camera"
    image.height = 1
    image.width = 4
    image.encoding = "mono8"
    image.step = 4
    image.data = [10, 11, 12, 13]

    assert publisher.publish(image)
    subscriber._metadata_callback(publisher.last_metadata)

    assert len(received) == 1
    assert received[0][0].header.frame_id == "camera"
    assert bytes(received[0][0].data) == bytes([10, 11, 12, 13])
    assert received[0][1].payload_size == 4
    subscriber.close()
    publisher.close()


def test_compressed_image_publisher_feeds_subscriber_callback():
    node = FakeNode()
    publisher = ShmPublisher(
        node,
        "/camera/direct_compressed_image",
        msg_type=CompressedImage,
        slot_count=2,
        slot_size_bytes=32,
        shm_name=f"/ros2_shm_py_direct_compressed_image_{os.getpid()}",
    )
    received = []
    subscriber = ShmSubscriber(
        node=node,
        topic="/camera/direct_compressed_image",
        loader=RosCompressedImageLoader(),
        callback=lambda msg, meta: received.append((msg, meta)),
    )

    image = CompressedImage()
    image.header.frame_id = "camera"
    image.format = "png"
    image.data = [137, 80, 78, 71, 1, 2]

    assert publisher.publish(image)
    subscriber._metadata_callback(publisher.last_metadata)

    assert len(received) == 1
    assert received[0][0].header.frame_id == "camera"
    assert received[0][0].format == "png"
    assert bytes(received[0][0].data) == bytes([137, 80, 78, 71, 1, 2])
    assert received[0][1].payload_size == 6
    subscriber.close()
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


def test_pointcloud2_publisher_feeds_subscriber_callback():
    node = FakeNode()
    publisher = ShmPublisher(
        node,
        "/direct/points",
        msg_type=PointCloud2,
        slot_count=2,
        slot_size_bytes=16,
        shm_name=f"/ros2_shm_py_direct_cloud_{os.getpid()}",
    )
    received = []
    subscriber = ShmSubscriber(
        node=node,
        topic="/direct/points",
        loader=RosPointCloud2Loader(),
        callback=lambda msg, meta: received.append((msg, meta)),
    )

    cloud = PointCloud2()
    cloud.header.frame_id = "lidar"
    cloud.height = 1
    cloud.width = 1
    cloud.point_step = 4
    cloud.row_step = 4
    cloud.is_dense = True
    cloud.data = [20, 21, 22, 23]

    assert publisher.publish(cloud)
    subscriber._metadata_callback(publisher.last_metadata)

    assert len(received) == 1
    assert received[0][0].header.frame_id == "lidar"
    assert bytes(received[0][0].data) == bytes([20, 21, 22, 23])
    assert received[0][1].payload_size == 4
    subscriber.close()
    publisher.close()


def test_publisher_rejects_oversized_payload_when_resize_disabled():
    node = FakeNode()
    publisher = ShmPublisher(
        node,
        "/camera/oversized",
        msg_type=Image,
        slot_count=1,
        slot_size_bytes=4,
        allow_resize=False,
        shm_name=f"/ros2_shm_py_oversized_{os.getpid()}",
    )

    image = Image()
    image.height = 1
    image.width = 4
    image.encoding = "mono8"
    image.step = 4
    image.data = [1, 2, 3, 4]
    assert publisher.publish(image)

    image.width = 8
    image.step = 8
    image.data = [1, 2, 3, 4, 5, 6, 7, 8]
    with pytest.raises(ValueError, match="payload is larger"):
        publisher.publish(image)
    publisher.close()


def test_publisher_resizes_payload_slot_when_allowed():
    node = FakeNode()
    publisher = ShmPublisher(
        node,
        "/camera/resize",
        msg_type=Image,
        slot_count=1,
        slot_size_bytes=0,
        allow_resize=True,
        shm_name=f"/ros2_shm_py_resize_{os.getpid()}",
    )

    image = Image()
    image.height = 1
    image.width = 4
    image.encoding = "mono8"
    image.step = 4
    image.data = [1, 2, 3, 4]
    assert publisher.publish(image)
    assert publisher.last_metadata.slot_size == 4

    image.width = 8
    image.step = 8
    image.data = [1, 2, 3, 4, 5, 6, 7, 8]
    assert publisher.publish(image)
    assert publisher.last_metadata.slot_size == 8
    publisher.close()


def test_publisher_unlinks_shared_memory_on_close():
    node = FakeNode()
    publisher = ShmPublisher(
        node,
        "/camera/unlink",
        msg_type=Image,
        slot_count=1,
        slot_size_bytes=4,
        shm_name=f"/ros2_shm_py_unlink_{os.getpid()}",
    )

    image = Image()
    image.height = 1
    image.width = 4
    image.encoding = "mono8"
    image.step = 4
    image.data = [1, 2, 3, 4]
    assert publisher.publish(image)
    path = f"/dev/shm/{publisher.shm_name[1:]}"
    assert os.path.exists(path)

    publisher.close()
    assert not os.path.exists(path)
