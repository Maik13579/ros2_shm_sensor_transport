# adapted_image_transport

`adapted_image_transport` combines the normal `image_transport` topic layout
with a same-process type-adapted path for REP-2007 adapters whose ROS message
type is `sensor_msgs::msg::Image`.

Use it when C++ nodes in the same process should exchange an application-specific
image representation without converting through `sensor_msgs/msg/Image`, while
external consumers and ROS tools should still see the normal image transport
topics.

Public consumers keep using the base image topic and normal transport topics.
Same-process adapted consumers use a private implementation topic:

```text
<base_topic>/_adapted/<publisher_token>
```

The private topic uses `rclcpp::Publisher<AdapterT>` and
`rclcpp::Subscription<AdapterT>` with intra-process communication enabled. When
only local adapted subscribers exist, publishing does not convert the custom
type into `sensor_msgs::msg::Image`. When public transport subscribers also
exist, the publisher converts for the public path first, then moves the custom
message into the local adapted publisher.

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

The adapter type must use `sensor_msgs::msg::Image` as its ROS message type:

```cpp
using AdaptedImage = rclcpp::adapt_type<MyImageView>::as<sensor_msgs::msg::Image>;

auto pub = adapted_image_transport::create_publisher<AdaptedImage>(
  node,
  "/camera/image_raw");

auto frame = std::make_unique<MyImageView>(make_frame());
pub.publish(std::move(frame));
```

## Subscriber

Create a shared-message subscription when the callback should borrow the adapted
object on the local path:

```cpp
using AdaptedImage = rclcpp::adapt_type<MyImageView>::as<sensor_msgs::msg::Image>;

auto sub = adapted_image_transport::create_subscription<AdaptedImage>(
  node,
  "/camera/image_raw",
  [](const std::shared_ptr<const MyImageView> & frame) {
    consume(*frame);
  },
  "raw");
```

Use `create_unique_subscription()` when the callback should receive owned
custom messages on the local adapted path:

```cpp
auto sub = adapted_image_transport::create_unique_subscription<AdaptedImage>(
  node,
  "/camera/image_raw",
  [](std::unique_ptr<MyImageView> frame) {
    consume(*frame);
  },
  "raw");
```

Local adapted topics are reserved implementation details. They are process-local
optimization hooks, not security boundaries.

## Notes

- The public topic remains compatible with standard `image_transport`
  subscribers.
- Same-process adapted delivery requires the publisher and subscriber to share
  the same `rclcpp::Context`.
- If no matching local adapted publisher is available, subscribers fall back to
  the public transport path.
- The private `/_adapted/<publisher_token>` topic name is implementation detail;
  do not hard-code it in launch files or user-facing configuration.
