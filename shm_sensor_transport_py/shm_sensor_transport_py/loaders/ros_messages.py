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

from sensor_msgs.msg import Image, PointCloud2
from shm_sensor_transport_interfaces.msg import ShmImage, ShmPointCloud2

from shm_sensor_transport_py.loaders.base import Loader


class RosImageLoader(Loader):
    """Reconstruct a sensor_msgs.msg.Image from SHM metadata and payload bytes."""

    msg_type = ShmImage

    def from_bytes(self, data: bytes, meta):
        msg = Image()
        msg.header = meta.header
        msg.height = meta.height
        msg.width = meta.width
        msg.encoding = meta.encoding
        msg.is_bigendian = int(meta.is_bigendian)
        msg.step = meta.step
        msg.data = data
        return msg


class RosPointCloud2Loader(Loader):
    """Reconstruct a sensor_msgs.msg.PointCloud2 from SHM metadata and payload bytes."""

    msg_type = ShmPointCloud2

    def from_bytes(self, data: bytes, meta):
        msg = PointCloud2()
        msg.header = meta.header
        msg.height = meta.height
        msg.width = meta.width
        msg.fields = list(meta.fields)
        msg.is_bigendian = meta.is_bigendian
        msg.point_step = meta.point_step
        msg.row_step = meta.row_step
        msg.data = data
        msg.is_dense = meta.is_dense
        return msg
