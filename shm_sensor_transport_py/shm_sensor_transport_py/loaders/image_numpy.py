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

import numpy as np

from shm_sensor_transport_interfaces.msg import ShmImage

from shm_sensor_transport_py.loaders.base import Loader


class NumpyImageLoader(Loader):
    """Convert shared-memory image payloads into NumPy arrays."""

    msg_type = ShmImage

    _ENCODINGS = {
        "mono8": (np.uint8, 1),
        "8UC1": (np.uint8, 1),
        "rgb8": (np.uint8, 3),
        "bgr8": (np.uint8, 3),
        "rgba8": (np.uint8, 4),
        "bgra8": (np.uint8, 4),
        "mono16": (np.uint16, 1),
        "16UC1": (np.uint16, 1),
        "32FC1": (np.float32, 1),
    }

    def from_bytes(self, data: bytes, meta):
        if meta.encoding not in self._ENCODINGS:
            raise ValueError(f"Unsupported image encoding: {meta.encoding}")
        dtype, channels = self._ENCODINGS[meta.encoding]
        dtype = np.dtype(dtype).newbyteorder(">" if meta.is_bigendian else "<")
        row_bytes = int(meta.step)
        height = int(meta.height)
        width = int(meta.width)
        item_size = dtype.itemsize
        expected_row = width * channels * item_size

        array = np.frombuffer(data, dtype=np.uint8)
        # ROS image rows can be padded; trim each row to the active pixel region.
        rows = array.reshape(height, row_bytes)
        rows = rows[:, :expected_row]
        image = rows.reshape(height, width, channels).view(dtype)
        image = image.reshape(height, width, channels)
        if channels == 1:
            return image.reshape(height, width)
        return image
