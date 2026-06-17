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

import rclpy
from rclpy.node import Node

from shm_sensor_transport_py import ShmSubscriber
from shm_sensor_transport_py.loaders import Open3DPointCloudLoader


class PointCloudConsumer(Node):
    def __init__(self):
        super().__init__("shm_pointcloud_open3d_consumer")
        self._subscriber = ShmSubscriber(
            node=self,
            topic="/points",
            loader=Open3DPointCloudLoader(),
            callback=self._on_cloud,
        )

    def _on_cloud(self, cloud, meta):
        self.get_logger().info(
            f"Received Open3D point cloud with {len(cloud.points)} points from {meta.shm_name}",
            throttle_duration_sec=5.0,
        )


def main():
    rclpy.init()
    node = PointCloudConsumer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
