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
from sensor_msgs.msg import PointField
from shm_sensor_transport_interfaces.msg import ShmPointCloud2

from shm_sensor_transport_py.loaders.base import Loader


class PointCloud2NumpyLoader(Loader):
    """Convert PointCloud2 payload bytes into a structured NumPy array."""

    msg_type = ShmPointCloud2

    _DATATYPES = {
        PointField.INT8: ("i1", 1),
        PointField.UINT8: ("u1", 1),
        PointField.INT16: ("i2", 2),
        PointField.UINT16: ("u2", 2),
        PointField.INT32: ("i4", 4),
        PointField.UINT32: ("u4", 4),
        PointField.FLOAT32: ("f4", 4),
        PointField.FLOAT64: ("f8", 8),
    }

    def from_bytes(self, data: bytes, meta):
        dtype = self._make_dtype(meta)
        height = int(meta.height)
        width = int(meta.width)
        point_step = int(meta.point_step)
        row_step = int(meta.row_step)

        if row_step == width * point_step:
            return np.frombuffer(data, dtype=dtype, count=height * width).reshape(height, width)

        # PointCloud2 rows may contain padding, so decode each row independently.
        rows = []
        for row_index in range(height):
            row_offset = row_index * row_step
            row = data[row_offset : row_offset + (width * point_step)]
            rows.append(np.frombuffer(row, dtype=dtype, count=width))
        return np.vstack(rows)

    def _make_dtype(self, meta):
        names = []
        formats = []
        offsets = []
        endian = ">" if meta.is_bigendian else "<"

        for field in meta.fields:
            if field.datatype not in self._DATATYPES:
                raise ValueError(f"Unsupported PointField datatype: {field.datatype}")
            base_format, size = self._DATATYPES[field.datatype]
            names.append(field.name)
            if field.count == 1:
                formats.append(endian + base_format)
            else:
                formats.append((endian + base_format, int(field.count)))
            offsets.append(int(field.offset))

        return np.dtype(
            {
                "names": names,
                "formats": formats,
                "offsets": offsets,
                "itemsize": int(meta.point_step),
            }
        )
