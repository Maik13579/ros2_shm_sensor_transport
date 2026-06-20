# shm_sensor_transport_py

`shm_sensor_transport_py` is the Python API for the shared-memory sensor
transport. It can publish image, compressed-image, and point-cloud payloads to
the shared-memory path and subscribe to metadata produced by C++ or Python
publishers.

## Publisher

Publish a normal ROS image, compressed-image, or point-cloud message directly
through shared memory:

```python
from sensor_msgs.msg import Image
from shm_sensor_transport_py import ShmPublisher

pub = ShmPublisher(
    node,
    '/camera/image_raw',
    msg_type=Image,
    slot_count=8,
    slot_size_bytes=0,
)

pub.publish(image_msg)
```

`slot_size_bytes=0` infers the slot size from the first published message. Set a
fixed value for known fixed-size streams to make memory use explicit.

## Subscriber

Subscribe with the normal sensor topic. `ShmSubscriber` appends `/_shm` when
needed and leaves topics that already end with `/_shm` unchanged.

```python
import rclpy
from rclpy.node import Node

from shm_sensor_transport_py import ShmSubscriber
from shm_sensor_transport_py.loaders import RosImageLoader


class ImageConsumer(Node):
    def __init__(self):
        super().__init__('image_consumer')
        self.sub = ShmSubscriber(
            node=self,
            topic='/camera/image_raw',
            loader=RosImageLoader(),
            callback=self.on_image,
        )

    def on_image(self, msg, meta):
        self.get_logger().info(
            f'received {msg.width}x{msg.height} from {meta.shm_name}')


rclpy.init()
node = ImageConsumer()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
```

## Loaders

ROS-message loaders reconstruct standard sensor messages:

```python
from shm_sensor_transport_py.loaders import (
    RosCompressedImageLoader,
    RosImageLoader,
    RosPointCloud2Loader,
)
```

Use `RosPointCloud2Loader` for `sensor_msgs/msg/PointCloud2`:

```python
from shm_sensor_transport_py import ShmSubscriber
from shm_sensor_transport_py.loaders import RosPointCloud2Loader

sub = ShmSubscriber(
    node=node,
    topic='/points',
    loader=RosPointCloud2Loader(),
    callback=on_cloud,
)
```

Use `RosCompressedImageLoader` for `sensor_msgs/msg/CompressedImage`. It
preserves `header`, `format`, and compressed `data` bytes without decoding or
recompressing.

Additional loaders are available for NumPy arrays, OpenCV-style image arrays,
Open3D point clouds, and raw bytes. Optional third-party libraries must be
available in the runtime environment before using those optional loaders.

## Examples

This package includes example subscribers:

```text
examples/image_cv2_subscriber.py
examples/pointcloud_open3d_subscriber.py
```
