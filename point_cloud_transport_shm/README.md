# point_cloud_transport_shm

`point_cloud_transport_shm` provides a `point_cloud_transport` plugin named
`shm` for `sensor_msgs/msg/PointCloud2` streams. It lets existing
`point_cloud_transport` users select the shared-memory path while keeping the
normal point-cloud transport API.

The plugin publishes and subscribes to hidden metadata topics using the same
convention as the direct C++ and Python APIs:

```text
<base_topic>/_shm
```

## Usage

Select the `shm` transport with `TransportHints`:

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

## Parameters

SHM-specific options are ROS parameters on the node that creates the transport
publisher or subscriber. The namespace is derived from the metadata topic by
removing the leading slash and replacing slashes with dots.

For `/points`, configure publisher options under `points._shm`:

```yaml
cloud_node:
  ros__parameters:
    points:
      _shm:
        slot_count: 8
        slot_size_bytes: 4194304
        allow_resize: true
        warn_on_oversized_frame: true
```

Subscriber-side rate limiting uses the same namespace on the subscribing node:

```yaml
consumer_node:
  ros__parameters:
    points:
      _shm:
        rate_limit_hz: 15.0
```
