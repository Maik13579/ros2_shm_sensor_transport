# shm_sensor_transport

`shm_sensor_transport` is a ROS 2 transport path for high-bandwidth, intra-host
sensor streams. It is intended for image, compressed-image, and point-cloud
pipelines where local C++ and Python processes can move large payload bytes
through shared memory instead of sending a full serialized
`sensor_msgs/Image`, `sensor_msgs/CompressedImage`, or
`sensor_msgs/PointCloud2` message on every callback.

## Why

Many robotics perception systems split sensor drivers, perception nodes, and
debugging tools across multiple processes on the same host. ROS 2 keeps that
split productive, but very large sensor messages can become expensive when every
local consumer receives the full payload through the normal DDS topic path. The
data is already local to the machine, yet it still has to move through
serialization, message construction, and downstream conversion before user code
can work with the pixels or points.

This package adds a local fast path for supported sensor messages:

- C++ and Python publishers can write raw payload bytes directly into a
  fixed-size shared-memory ring buffer.
- Publishers emit a small ROS 2 metadata message that identifies the shared
  memory object, slot, sequence, and sensor layout.
- Shared-memory subscribers receive the metadata, copy the selected payload
  bytes from shared memory, validate that the slot was not overwritten during the
  copy, and pass a reconstructed message or loader output to user code.

Subscribers still copy the payload before invoking callbacks. That copy is
intentional: it gives user code normal object lifetimes while allowing the writer
to keep reusing ring-buffer slots.

## Architecture

The repository is split into focused ROS 2 packages:

- `shm_sensor_transport_interfaces`: metadata message definitions shared by the
  C++ and Python packages.
- `shm_sensor_transport`: C++ relay components, shared-memory ring-buffer
  implementation, and direct C++ publisher/subscriber API.
- `shm_sensor_transport_py`: Python publisher/subscriber API, shared-memory
  handles, and loader plugins.
- `image_transport_shm`: `image_transport` plugin that exposes the
  shared-memory image path through the standard image transport API.
- `point_cloud_transport_shm`: `point_cloud_transport` plugin that exposes the
  shared-memory point-cloud path through the standard point cloud transport API.
- `adapted_image_transport`: header-only type-adapted image publisher and
  subscriber wrappers with public `image_transport` compatibility and a private
  same-process intra-process path.
- `adapted_point_cloud_transport`: header-only type-adapted point-cloud
  publisher and subscriber wrappers with public `point_cloud_transport`
  compatibility and a private same-process intra-process path.
- `shm_sensor_transport_rviz`: RViz integration for inspecting streams carried
  by the shared-memory transport.
- `shm_sensor_transport_test`: test support and integration coverage for the
  shared-memory transport packages.

Direct shared-memory publishing uses one metadata topic and one POSIX shared
memory object:

```mermaid
flowchart LR
    pub["ShmPublisher<br/>Image, CompressedImage, or PointCloud2"]
    shm[("POSIX shared memory<br/>payload ring")]
    meta["/_shm metadata topic<br/>ShmImage, ShmCompressedImage, or ShmPointCloud2"]
    sub["ShmSubscriber<br/>reconstructed ROS message"]

    pub -- "writes msg.data" --> shm
    pub -- "publishes slot metadata" --> meta
    meta --> sub
    shm -- "copies validated payload" --> sub
```

```text
Sensor driver process
  └── ShmPublisher
        ├── accepts sensor_msgs/Image, sensor_msgs/CompressedImage, or sensor_msgs/PointCloud2
        ├── writes msg.data into the next ring-buffer slot
        └── publishes /camera/image_raw/_shm as hidden ShmImage metadata

POSIX shared memory
  └── /dev/shm/ros2_shm_camera_image_raw_<hash>

C++ or Python consumer process
  └── ShmSubscriber
        ├── accepts /camera/image_raw and subscribes to /camera/image_raw/_shm
        ├── opens and caches the shared-memory object
        ├── copies the slot payload into callback-owned memory
        ├── validates slot sequence counters
        └── calls the user callback with a reconstructed message or loader output
```

When a sensor driver already publishes a normal ROS topic and cannot use
`ShmPublisher` directly, a relay component can adapt that topic:

```text
Existing sensor driver process
  └── publishes /camera/image_raw as sensor_msgs/Image

Relay component
  ├── subscribes to /camera/image_raw
  ├── allocates /dev/shm/ros2_shm_camera_image_raw_<hash>
  ├── writes msg.data into the next ring-buffer slot
  └── publishes /camera/image_raw/_shm as hidden ShmImage metadata
```

Compressed images and point clouds follow the same pattern using
`sensor_msgs/CompressedImage` with `ShmCompressedImage` metadata and
`sensor_msgs/PointCloud2` with `ShmPointCloud2` metadata. Compressed image
payloads stay compressed: the transport preserves `format` and moves `data`
bytes through shared memory unchanged.

Publishers derive the metadata topic from the normal topic as `<topic>/_shm`.
This keeps metadata on a predictable hidden ROS topic.

## Shared Memory Model

Each shared-memory publisher owns one shared-memory object. The object contains:

```text
SharedMemoryHeader
SlotHeader[slot_count]
PayloadSlot[slot_count]
```

Every payload slot has the same configured size. If `slot_size_bytes` is zero,
the publisher infers the slot size from the first received message. A slot
sequence counter is odd while the writer is updating the slot and even once the payload is
complete. Readers accept a copy only when the sequence value before and after the
copy is identical and even.

This gives latest-frame behavior suitable for high-rate sensor streams. It does
not provide reliable history for frames whose slots have already been reused.

## Compatibility

The shared-memory stream is a local transport based on hidden ROS metadata
topics:

```text
/camera/image_raw/_shm   shm_sensor_transport_interfaces/ShmImage

/camera/image_raw/compressed/_shm
                         shm_sensor_transport_interfaces/ShmCompressedImage

/points/_shm             shm_sensor_transport_interfaces/ShmPointCloud2
```

Direct `ShmPublisher` does not publish the original `sensor_msgs/Image` or
`sensor_msgs/CompressedImage` or `sensor_msgs/PointCloud2` topic. If you need
the normal sensor topic to remain available for ROS tools or remote subscribers,
publish it separately or use a relay with an existing publisher.

## Type adapters

ROS 2 [REP-2007](https://ros.org/reps/rep-2007.html) type adapters let C++
publishers and subscribers use application-specific data types while preserving
a normal ROS message type at the topic boundary.

This repository includes `adapted_image_transport` and
`adapted_point_cloud_transport`, header-only wrappers around the standard
`image_transport` and `point_cloud_transport` APIs. Same-process consumers can
receive the adapted C++ type through ROS 2 intra-process communication, while
external consumers and ROS tools continue to use the normal public transport
topics.

## Usage

Publish directly to shared memory from C++ or Python when you control the sensor
publisher code. Use the same topic name on the publisher and subscriber; the API
maps it to a hidden metadata topic:

```text
/camera/image_raw/_shm   shm_sensor_transport_interfaces/ShmImage
```

### Python

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

Subscribe with the normal sensor topic. `ShmSubscriber` appends `/_shm` when
needed, and leaves topics that already end with `/_shm` unchanged. To reconstruct
normal ROS image messages:

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
        self.get_logger().info(f'received {msg.width}x{msg.height} from {meta.shm_name}')


rclpy.init()
node = ImageConsumer()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
```

Use `RosPointCloud2Loader` for `sensor_msgs/PointCloud2` messages:

```python
from shm_sensor_transport_py.loaders import RosPointCloud2Loader

sub = ShmSubscriber(
    node=node,
    topic='/points',
    loader=RosPointCloud2Loader(),
    callback=on_cloud,
)
```

Use `RosCompressedImageLoader` for `sensor_msgs/CompressedImage` messages. It
reconstructs `header`, `format`, and the original compressed `data` bytes
without decoding or recompressing them:

```python
from shm_sensor_transport_py.loaders import RosCompressedImageLoader

sub = ShmSubscriber(
    node=node,
    topic='/camera/image_raw/compressed',
    loader=RosCompressedImageLoader(),
    callback=on_compressed_image,
)
```

Other loaders are available when user code wants NumPy arrays, OpenCV-style image
arrays, Open3D point clouds, or raw bytes instead of ROS message objects.

### C++

Publish a normal ROS image message directly through shared memory:

```cpp
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "shm_sensor_transport/shm_publisher.hpp"

auto pub = std::make_unique<shm_sensor_transport::ShmImagePublisher>(
  node,
  "/camera/image_raw",
  shm_sensor_transport::ShmPublisherOptions{});

pub->publish(image_msg);
```

Include `shm_sensor_transport/shm_subscriber.hpp` and keep the subscriber object
alive for as long as the node should receive frames:

```cpp
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "shm_sensor_transport/shm_subscriber.hpp"

class ImageConsumer : public rclcpp::Node
{
public:
  ImageConsumer()
  : rclcpp::Node("image_consumer")
  {
    sub_ = std::make_unique<shm_sensor_transport::ShmImageSubscriber>(
      this,
      "/camera/image_raw",
      [this](
        sensor_msgs::msg::Image::UniquePtr msg,
        const shm_sensor_transport_interfaces::msg::ShmImage & meta)
      {
        RCLCPP_INFO(
          get_logger(), "received %ux%u from %s",
          msg->width, msg->height, meta.shm_name.c_str());
      });
  }

private:
  std::unique_ptr<shm_sensor_transport::ShmImageSubscriber> sub_;
};
```

For compressed images, use `shm_sensor_transport::ShmCompressedImageSubscriber`
with a `sensor_msgs::msg::CompressedImage::UniquePtr` callback. For point
clouds, use `shm_sensor_transport::ShmPointCloud2Subscriber` with a
`sensor_msgs::msg::PointCloud2::UniquePtr` callback. The C++ subscriber accepts
the same normal topic names as the Python API and also appends `/_shm` unless
the topic already points at the metadata topic.

## image_transport and point_cloud_transport

The optional `image_transport_shm` and `point_cloud_transport_shm` packages let
standard ROS transport users select the shared-memory path with transport name
`shm`. These packages depend on the transport frameworks; the core
`shm_sensor_transport` package does not.

Use `image_transport` with `TransportHints(..., "shm")`:

```cpp
#include <image_transport/image_transport.hpp>
#include <image_transport/transport_hints.hpp>

image_transport::ImageTransport image_transport(node);

auto publisher = image_transport.advertise("/camera/image_raw", 1);
auto hints = image_transport::TransportHints(node.get(), "shm");
auto subscriber = image_transport.subscribe(
  "/camera/image_raw",
  1,
  [](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
    // Normal Image callback.
  },
  nullptr,
  &hints);

publisher.publish(image_msg);
```

Use `point_cloud_transport` with `TransportHints("shm")`:

```cpp
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <point_cloud_transport/transport_hints.hpp>

point_cloud_transport::PointCloudTransport point_cloud_transport(node);

auto publisher = point_cloud_transport.advertise("/points", 1);
auto hints = point_cloud_transport::TransportHints("shm");
auto subscriber = point_cloud_transport.subscribe(
  "/points",
  1,
  [](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
    // Normal PointCloud2 callback.
  },
  nullptr,
  &hints);

publisher.publish(cloud_msg);
```

Both plugins publish or subscribe to the same hidden metadata topic convention:
`<base_topic>/_shm`.

SHM-specific options are ROS parameters on the node that creates the transport
publisher or subscriber. The parameter namespace is derived from the metadata
topic by removing the leading slash and replacing slashes with dots. For
`/camera/image_raw`, configure publisher options under
`camera.image_raw._shm`:

```yaml
camera_node:
  ros__parameters:
    camera:
      image_raw:
        _shm:
          slot_count: 8
          slot_size_bytes: 8294400
          allow_resize: false
          warn_on_oversized_frame: true
```

Subscriber-side rate limiting uses the same namespace on the subscribing node:

```yaml
consumer_node:
  ros__parameters:
    camera:
      image_raw:
        _shm:
          rate_limit_hz: 30.0
```

Point clouds use the same convention. For `/points`, set
`points._shm.slot_size_bytes`, `points._shm.slot_count`, and the other options
on the point cloud publisher node.

## Relay Components

Use a relay when an existing sensor driver already publishes
`sensor_msgs/Image`, `sensor_msgs/CompressedImage`, or
`sensor_msgs/PointCloud2` and you want to add the shared-memory fast path
without modifying that driver. The original sensor topic continues to come from
the existing publisher; the relay only subscribes to it and publishes
shared-memory metadata.

```mermaid
flowchart LR
    sensor["Sensor driver component<br/>normal sensor publisher"]
    relay["ShmImageRelayComponent<br/>ShmCompressedImageRelayComponent<br/>or ShmPointCloud2RelayComponent"]
    shm[("POSIX shared memory<br/>payload ring")]
    meta["/_shm metadata topic"]
    sub["ShmSubscriber"]

    sensor -- "intra-process sensor_msgs" --> relay
    relay -- "writes msg.data" --> shm
    relay -- "publishes slot metadata" --> meta
    meta --> sub
    shm -- "copies validated payload" --> sub
```

For best relay performance, load the sensor driver component and relay component
into the same component container with intra-process communication enabled:

```python
ComposableNode(
    package='your_sensor_driver_package',
    plugin='your_sensor_driver_package::CameraDriverComponent',
    name='camera_driver',
    parameters=[{
        'image_topic': '/camera/image_raw',
    }],
    extra_arguments=[{'use_intra_process_comms': True}],
),
ComposableNode(
    package='shm_sensor_transport',
    plugin='shm_sensor_transport::ShmImageRelayComponent',
    name='shm_image_relay',
    parameters=[{
        'common.input_topic': '/camera/image_raw',
        'common.slot_count': 8,
        'common.slot_size_bytes': 0,
        'common.publish_status': False,
    }],
    extra_arguments=[{'use_intra_process_comms': True}],
)
```

For compressed images, use
`shm_sensor_transport::ShmCompressedImageRelayComponent` and pass
`/camera/image_raw/compressed` to the C++ or Python subscriber. For point clouds,
use `shm_sensor_transport::ShmPointCloud2RelayComponent` and pass `/points` to
the C++ or Python subscriber.

`common.slot_size_bytes` controls the fixed payload capacity of each shared
memory slot. Keep it at `0` to infer the size from the first frame, or set it
explicitly for known fixed-size streams. `common.publish_status` is optional and
defaults to `false`; set it to `true` and provide `common.status_topic` only when
you want periodic transport status messages.

## Benchmarks

The recorded benchmark compares a normal Python `sensor_msgs/Image` subscriber
against a Python `ShmSubscriber` fed by a C++ publisher and relay loaded into one
component container with intra-process communication enabled. Both paths return
ROS image messages to Python and validate deterministic payload bytes, so the
numbers focus on transport cost rather than application logic.

Across the recorded runs, the shared-memory path reduced mean latency and CPU
time most clearly for large payloads. With a 1 MiB image stream at 120 Hz, the
shared-memory subscriber measured about `0.8 ms` mean latency versus `4.7 ms`
for the normal Python subscriber, with lower CPU use in the benchmark process.
At 4 MiB and 30 Hz, the normal subscriber dropped best-effort samples while the
shared-memory path received all requested frames and used substantially less CPU.

See [BENCHMARK.md](BENCHMARK.md) for the exact commands, tables, and additional
payload/rate settings.

## Limits

- Only intra-host communication is supported.
- The relay still receives the original ROS 2 sensor message.
- Subscribers copy payload bytes before invoking user callbacks.
- Overwritten ring-buffer slots are dropped, not recovered.
- Maximum efficiency comes from direct sensor-driver integration with
  `ShmPublisher` rather than a relay subscribed to an existing topic.
