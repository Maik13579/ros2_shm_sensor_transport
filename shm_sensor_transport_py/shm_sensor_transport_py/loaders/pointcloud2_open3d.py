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

from shm_sensor_transport_py.loaders.pointcloud2_numpy import PointCloud2NumpyLoader


class Open3DPointCloudLoader(PointCloud2NumpyLoader):
    """Convert XYZ PointCloud2 payloads into an Open3D point cloud."""

    def from_bytes(self, data: bytes, meta):
        try:
            import open3d as o3d
        except ImportError as error:
            raise ImportError("Open3DPointCloudLoader requires the open3d Python package") from error

        structured = super().from_bytes(data, meta).reshape(-1)
        for field_name in ("x", "y", "z"):
            if field_name not in structured.dtype.names:
                raise ValueError("Open3D conversion requires x, y, and z fields")
        xyz = np.column_stack((structured["x"], structured["y"], structured["z"])).astype(np.float64)
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(xyz)
        return cloud
