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
from shm_sensor_transport_py.loaders import CvImageLoader


class ImageConsumer(Node):
    def __init__(self):
        super().__init__("shm_image_cv2_consumer")
        self._subscriber = ShmSubscriber(
            node=self,
            topic="/camera/image_raw",
            loader=CvImageLoader(),
            callback=self._on_image,
        )

    def _on_image(self, image, meta):
        self.get_logger().info(
            f"Received {meta.encoding} image {image.shape} from {meta.shm_name}",
            throttle_duration_sec=5.0,
        )


def main():
    rclpy.init()
    node = ImageConsumer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
