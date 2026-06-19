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

import struct

import numpy as np
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

from shm_sensor_transport_py.loaders import (
    NumpyImageLoader,
    PointCloud2NumpyLoader,
    RawBytesLoader,
    RosCompressedImageLoader,
    RosImageLoader,
    RosPointCloud2Loader,
)


class ImageMeta:
    header = Header()
    height = 2
    width = 2
    encoding = "rgb8"
    step = 6
    is_bigendian = False


class CompressedImageMeta:
    header = Header()
    format = "jpeg"


class CloudMeta:
    header = Header()
    height = 1
    width = 2
    is_bigendian = False
    is_dense = True
    point_step = 12
    row_step = 24
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]


def test_raw_bytes_loader_returns_payload():
    payload = b"payload"
    assert RawBytesLoader().from_bytes(payload, object()) == payload


def test_numpy_image_loader_shapes_rgb_image():
    payload = bytes(range(12))
    image = NumpyImageLoader().from_bytes(payload, ImageMeta())
    assert image.shape == (2, 2, 3)
    assert image.dtype == np.uint8
    assert image[1, 1, 2] == 11


def test_pointcloud2_numpy_loader_builds_structured_array():
    payload = struct.pack("<ffffff", 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    cloud = PointCloud2NumpyLoader().from_bytes(payload, CloudMeta())
    assert cloud.shape == (1, 2)
    assert cloud["x"][0, 0] == np.float32(1.0)
    assert cloud["z"][0, 1] == np.float32(6.0)


def test_ros_image_loader_reconstructs_message():
    payload = bytes(range(12))
    msg = RosImageLoader().from_bytes(payload, ImageMeta())
    assert msg.height == ImageMeta.height
    assert msg.width == ImageMeta.width
    assert msg.encoding == ImageMeta.encoding
    assert bytes(msg.data) == payload


def test_ros_compressed_image_loader_reconstructs_message():
    payload = b"\xff\xd8compressed-jpeg\xff\xd9"
    msg = RosCompressedImageLoader().from_bytes(payload, CompressedImageMeta())
    assert msg.header == CompressedImageMeta.header
    assert msg.format == CompressedImageMeta.format
    assert bytes(msg.data) == payload


def test_ros_pointcloud2_loader_reconstructs_message():
    payload = struct.pack("<ffffff", 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    msg = RosPointCloud2Loader().from_bytes(payload, CloudMeta())
    assert msg.height == CloudMeta.height
    assert msg.width == CloudMeta.width
    assert msg.point_step == CloudMeta.point_step
    assert bytes(msg.data) == payload
