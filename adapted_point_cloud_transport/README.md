# adapted_point_cloud_transport

`adapted_point_cloud_transport` combines the normal `point_cloud_transport`
topic layout with a same-process type-adapted path for REP-2007 adapters whose
ROS message type is `sensor_msgs::msg::PointCloud2`.

Use it when C++ nodes in the same process should exchange an application-specific
point-cloud representation without converting through
`sensor_msgs/msg/PointCloud2`, while external consumers and ROS tools should
still see the normal point-cloud transport topics.

Public consumers keep using the base point-cloud topic and normal transport
topics. Same-process adapted consumers use a private implementation topic:

```text
<base_topic>/_adapted/<publisher_token>
```

The private topic uses `rclcpp::Publisher<AdapterT>` and
`rclcpp::Subscription<AdapterT>` with intra-process communication enabled. When
only local adapted subscribers exist, publishing does not convert the custom
type into `sensor_msgs::msg::PointCloud2`. When public transport subscribers
also exist, the publisher converts for the public path first, then moves the
custom message into the local adapted publisher.

Publisher calls are asynchronous by default. `publish()` stores the latest
pending message and wakes a node timer, so type-adapter conversion and transport
publishing run in the executor instead of blocking the caller. Pass
`async_publish = false` to the publisher constructor or factory to publish
synchronously.

Subscribers initially fall back to the public transport path if no matching
adapted publisher is registered in the current `rclcpp::Context`. The
context-local registry notifies subscribers through guard conditions when a
publisher appears or disappears, so a subscriber can switch to the local adapted
path or back to public transport without polling.

## Publisher

The adapter type must use `sensor_msgs::msg::PointCloud2` as its ROS message
type:

```cpp
using AdaptedCloud = rclcpp::adapt_type<MyCloudView>::as<sensor_msgs::msg::PointCloud2>;

auto pub = adapted_point_cloud_transport::create_publisher<AdaptedCloud>(
  node,
  "/points");

auto cloud = std::make_unique<MyCloudView>(make_cloud());
pub.publish(std::move(cloud));
```

## Subscriber

Create a shared-message subscription when the callback should borrow the adapted
object on the local path:

```cpp
using AdaptedCloud = rclcpp::adapt_type<MyCloudView>::as<sensor_msgs::msg::PointCloud2>;

auto sub = adapted_point_cloud_transport::create_subscription<AdaptedCloud>(
  node,
  "/points",
  [](const std::shared_ptr<const MyCloudView> & cloud) {
    consume(*cloud);
  },
  "raw");
```

Use `create_unique_subscription()` when the callback should receive owned
custom messages on the local adapted path:

```cpp
auto sub = adapted_point_cloud_transport::create_unique_subscription<AdaptedCloud>(
  node,
  "/points",
  [](std::unique_ptr<MyCloudView> cloud) {
    consume(*cloud);
  },
  "raw");
```

Local adapted topics are reserved implementation details. They are process-local
optimization hooks, not security boundaries.

## Notes

- The public topic remains compatible with standard `point_cloud_transport`
  subscribers.
- Same-process adapted delivery requires the publisher and subscriber to share
  the same `rclcpp::Context`.
- If no matching local adapted publisher is available, subscribers fall back to
  the public transport path.
- The private `/_adapted/<publisher_token>` topic name is implementation detail;
  do not hard-code it in launch files or user-facing configuration.
