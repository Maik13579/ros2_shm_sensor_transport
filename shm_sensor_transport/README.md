# shm_sensor_transport

`shm_sensor_transport` contains the C++ implementation of the shared-memory
transport path. It provides the shared-memory ring buffer, direct C++ publishers
and subscribers, and relay components for existing ROS sensor topics.

The optional `image_transport` and `point_cloud_transport` integrations live in
separate packages.

## Direct C++ API

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

Subscribe with the same normal sensor topic. The subscriber appends `/_shm`
unless the provided topic already points at the metadata topic:

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

Use `ShmCompressedImagePublisher` and `ShmCompressedImageSubscriber` for
`sensor_msgs/msg/CompressedImage`. Use `ShmPointCloud2Publisher` and
`ShmPointCloud2Subscriber` for `sensor_msgs/msg/PointCloud2`.

## Relay Components

Use a relay when an existing sensor driver already publishes a normal ROS sensor
topic and you want to add the shared-memory fast path without modifying that
driver.

Available component plugins:

```text
shm_sensor_transport::ShmImageRelayComponent
shm_sensor_transport::ShmCompressedImageRelayComponent
shm_sensor_transport::ShmPointCloud2RelayComponent
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

For compressed images, use `ShmCompressedImageRelayComponent` and subscribe to
`/camera/image_raw/compressed`. For point clouds, use
`ShmPointCloud2RelayComponent` and subscribe to `/points`.

## Relay Launch Files

This package installs launch files for standalone relay use:

```text
shm_image_relay.launch.py
shm_compressed_image_relay.launch.py
shm_point_cloud2_relay.launch.py
```

Typical arguments map directly to the `common.*` relay parameters:

```bash
ros2 launch shm_sensor_transport shm_image_relay.launch.py \
  input_topic:=/camera/image_raw \
  slot_count:=8 \
  slot_size_bytes:=0
```

## Parameters

Relay components use grouped parameters under `common`:

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `common.input_topic` | varies by relay | Normal ROS sensor topic to read. |
| `common.slot_count` | `8` | Number of shared-memory ring slots. |
| `common.slot_size_bytes` | `0` | Fixed payload capacity per slot, or infer from first frame when zero. |
| `common.allow_resize` | `true` | Allow resizing when an oversized frame arrives. |
| `common.warn_on_oversized_frame` | `true` | Log when a frame exceeds the configured slot size. |
| `common.publish_status` | `false` | Publish transport status messages. |
| `common.status_topic` | derived | Optional status topic override. |

Direct publishers use `ShmPublisherOptions` for the same slot sizing behavior.
