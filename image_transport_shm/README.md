# image_transport_shm

`image_transport_shm` provides an `image_transport` plugin named `shm` for
`sensor_msgs/msg/Image` streams. It lets existing `image_transport` users select
the shared-memory path while keeping the normal image transport API.

The plugin publishes and subscribes to hidden metadata topics using the same
convention as the direct C++ and Python APIs:

```text
<base_topic>/_shm
```

## Usage

Select the `shm` transport with `TransportHints`:

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

## Parameters

SHM-specific options are ROS parameters on the node that creates the transport
publisher or subscriber. The namespace is derived from the metadata topic by
removing the leading slash and replacing slashes with dots.

For `/camera/image_raw`, configure publisher options under
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
