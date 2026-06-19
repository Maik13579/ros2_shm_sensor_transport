// Copyright 2026 Maik Knof
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <unistd.h>

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "shm_sensor_transport/shm_compressed_image_relay_component.hpp"
#include "shm_sensor_transport/shm_handle.hpp"
#include "shm_sensor_transport/shm_name.hpp"
#include "shm_sensor_transport/shm_publisher.hpp"
#include "shm_sensor_transport/shm_subscriber.hpp"

namespace
{

class RclcppFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    ::setenv("ROS_LOG_DIR", "/tmp/shm_sensor_transport_test_logs", 1);
    ::setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", 1);
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

template<typename MetadataMessageT>
bool spin_until_metadata(
  rclcpp::Node & publisher_node,
  rclcpp::Node & subscriber_node,
  typename MetadataMessageT::SharedPtr & meta)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node.get_node_base_interface());
  executor.add_node(subscriber_node.get_node_base_interface());

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!meta && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(10));
  }
  return static_cast<bool>(meta);
}

template<typename PredicateT>
bool spin_until(
  rclcpp::Node & publisher_node,
  rclcpp::Node & subscriber_node,
  PredicateT predicate)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node.get_node_base_interface());
  executor.add_node(subscriber_node.get_node_base_interface());

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!predicate() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(10));
  }
  return predicate();
}

std::string test_name(const std::string & suffix)
{
  return "/ros2_shm_test_publisher_" + std::to_string(::getpid()) + suffix;
}

}  // namespace

TEST_F(RclcppFixture, ImagePublisherPublishesReadableMetadata)
{
  auto publisher_node = std::make_shared<rclcpp::Node>("test_shm_image_publisher");
  auto subscriber_node = std::make_shared<rclcpp::Node>("test_shm_image_metadata_subscriber");

  shm_sensor_transport::ShmPublisherOptions options;
  options.shm_name = test_name("_image");
  options.slot_count = 2;
  options.slot_size_bytes = 16;
  options.qos = rclcpp::QoS(1).reliable();
  shm_sensor_transport::ShmImagePublisher publisher(
    publisher_node.get(), "/camera/image_raw", options);

  shm_sensor_transport_interfaces::msg::ShmImage::SharedPtr meta;
  const auto subscription =
    subscriber_node->create_subscription<shm_sensor_transport_interfaces::msg::ShmImage>(
    publisher.metadata_topic(), options.qos,
    [&meta](shm_sensor_transport_interfaces::msg::ShmImage::SharedPtr msg) {
      meta = std::move(msg);
    });
  (void)subscription;

  sensor_msgs::msg::Image image;
  image.header.frame_id = "camera";
  image.height = 1;
  image.width = 4;
  image.encoding = "mono8";
  image.step = 4;
  image.data = {1, 2, 3, 4};

  ASSERT_TRUE(publisher.publish(image));
  ASSERT_TRUE(spin_until_metadata<shm_sensor_transport_interfaces::msg::ShmImage>(
    *publisher_node, *subscriber_node, meta));
  EXPECT_EQ(meta->height, 1U);
  EXPECT_EQ(meta->width, 4U);
  EXPECT_EQ(meta->encoding, "mono8");

  shm_sensor_transport::ShmHandle handle;
  EXPECT_EQ(handle.copy_payload(*meta), (std::vector<std::uint8_t>{1, 2, 3, 4}));
}

TEST_F(RclcppFixture, CompressedImagePublisherPublishesReadableMetadata)
{
  auto publisher_node = std::make_shared<rclcpp::Node>("test_shm_compressed_image_publisher");
  auto subscriber_node =
    std::make_shared<rclcpp::Node>("test_shm_compressed_image_metadata_subscriber");

  shm_sensor_transport::ShmPublisherOptions options;
  options.shm_name = test_name("_compressed_image");
  options.slot_count = 2;
  options.slot_size_bytes = 32;
  options.qos = rclcpp::QoS(1).reliable();
  shm_sensor_transport::ShmCompressedImagePublisher publisher(
    publisher_node.get(), "/camera/image_raw/compressed", options);

  shm_sensor_transport_interfaces::msg::ShmCompressedImage::SharedPtr meta;
  const auto subscription =
    subscriber_node->create_subscription<
    shm_sensor_transport_interfaces::msg::ShmCompressedImage>(
    publisher.metadata_topic(), options.qos,
    [&meta](shm_sensor_transport_interfaces::msg::ShmCompressedImage::SharedPtr msg) {
      meta = std::move(msg);
    });
  (void)subscription;

  sensor_msgs::msg::CompressedImage image;
  image.header.frame_id = "camera";
  image.format = "jpeg";
  image.data = {0xff, 0xd8, 1, 2, 0xff, 0xd9};

  ASSERT_TRUE(publisher.publish(image));
  ASSERT_TRUE(spin_until_metadata<shm_sensor_transport_interfaces::msg::ShmCompressedImage>(
    *publisher_node, *subscriber_node, meta));
  EXPECT_EQ(meta->format, "jpeg");

  shm_sensor_transport::ShmHandle handle;
  EXPECT_EQ(handle.copy_payload(*meta), (std::vector<std::uint8_t>{0xff, 0xd8, 1, 2, 0xff, 0xd9}));
}

TEST_F(RclcppFixture, PointCloud2PublisherPublishesReadableMetadata)
{
  auto publisher_node = std::make_shared<rclcpp::Node>("test_shm_cloud_publisher");
  auto subscriber_node = std::make_shared<rclcpp::Node>("test_shm_cloud_metadata_subscriber");

  shm_sensor_transport::ShmPublisherOptions options;
  options.shm_name = test_name("_cloud");
  options.slot_count = 2;
  options.slot_size_bytes = 32;
  options.qos = rclcpp::QoS(1).reliable();
  shm_sensor_transport::ShmPointCloud2Publisher publisher(
    publisher_node.get(), "/points", options);

  shm_sensor_transport_interfaces::msg::ShmPointCloud2::SharedPtr meta;
  const auto subscription =
    subscriber_node->create_subscription<shm_sensor_transport_interfaces::msg::ShmPointCloud2>(
    publisher.metadata_topic(), options.qos,
    [&meta](shm_sensor_transport_interfaces::msg::ShmPointCloud2::SharedPtr msg) {
      meta = std::move(msg);
    });
  (void)subscription;

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "lidar";
  cloud.height = 1;
  cloud.width = 1;
  cloud.point_step = 4;
  cloud.row_step = 4;
  cloud.is_dense = true;
  cloud.data = {5, 6, 7, 8};

  ASSERT_TRUE(publisher.publish(cloud));
  ASSERT_TRUE(spin_until_metadata<shm_sensor_transport_interfaces::msg::ShmPointCloud2>(
    *publisher_node, *subscriber_node, meta));
  EXPECT_EQ(meta->height, 1U);
  EXPECT_EQ(meta->width, 1U);
  EXPECT_EQ(meta->point_step, 4U);

  shm_sensor_transport::ShmHandle handle;
  EXPECT_EQ(handle.copy_payload(*meta), (std::vector<std::uint8_t>{5, 6, 7, 8}));
}

TEST_F(RclcppFixture, ImagePublisherFeedsShmSubscriberCallback)
{
  auto publisher_node = std::make_shared<rclcpp::Node>("test_shm_image_direct_publisher");
  auto subscriber_node = std::make_shared<rclcpp::Node>("test_shm_image_direct_subscriber");

  shm_sensor_transport::ShmPublisherOptions publisher_options;
  publisher_options.shm_name = test_name("_direct_image");
  publisher_options.slot_count = 2;
  publisher_options.slot_size_bytes = 16;
  publisher_options.qos = rclcpp::QoS(1).reliable();
  shm_sensor_transport::ShmImagePublisher publisher(
    publisher_node.get(), "/direct/image_raw", publisher_options);

  sensor_msgs::msg::Image::UniquePtr received;
  shm_sensor_transport_interfaces::msg::ShmImage received_meta;
  shm_sensor_transport::ShmSubscriberOptions subscriber_options;
  subscriber_options.qos = publisher_options.qos;
  shm_sensor_transport::ShmImageSubscriber subscriber(
    subscriber_node.get(),
    "/direct/image_raw",
    [&received, &received_meta](
      sensor_msgs::msg::Image::UniquePtr msg,
      const shm_sensor_transport_interfaces::msg::ShmImage & meta)
    {
      received = std::move(msg);
      received_meta = meta;
    },
    subscriber_options);

  sensor_msgs::msg::Image image;
  image.header.frame_id = "camera";
  image.height = 1;
  image.width = 4;
  image.encoding = "mono8";
  image.step = 4;
  image.data = {10, 11, 12, 13};

  ASSERT_TRUE(publisher.publish(image));
  ASSERT_TRUE(spin_until(*publisher_node, *subscriber_node, [&received]() {
      return static_cast<bool>(received);
  }));
  EXPECT_EQ(received->header.frame_id, "camera");
  EXPECT_EQ(received->data, (std::vector<std::uint8_t>{10, 11, 12, 13}));
  EXPECT_EQ(received_meta.payload_size, 4U);
}

TEST_F(RclcppFixture, CompressedImagePublisherFeedsShmSubscriberCallback)
{
  auto publisher_node = std::make_shared<rclcpp::Node>("test_shm_compressed_image_direct_pub");
  auto subscriber_node = std::make_shared<rclcpp::Node>("test_shm_compressed_image_direct_sub");

  shm_sensor_transport::ShmPublisherOptions publisher_options;
  publisher_options.shm_name = test_name("_direct_compressed_image");
  publisher_options.slot_count = 2;
  publisher_options.slot_size_bytes = 32;
  publisher_options.qos = rclcpp::QoS(1).reliable();
  shm_sensor_transport::ShmCompressedImagePublisher publisher(
    publisher_node.get(), "/direct/image_raw/compressed", publisher_options);

  sensor_msgs::msg::CompressedImage::UniquePtr received;
  shm_sensor_transport_interfaces::msg::ShmCompressedImage received_meta;
  shm_sensor_transport::ShmSubscriberOptions subscriber_options;
  subscriber_options.qos = publisher_options.qos;
  shm_sensor_transport::ShmCompressedImageSubscriber subscriber(
    subscriber_node.get(),
    "/direct/image_raw/compressed",
    [&received, &received_meta](
      sensor_msgs::msg::CompressedImage::UniquePtr msg,
      const shm_sensor_transport_interfaces::msg::ShmCompressedImage & meta)
    {
      received = std::move(msg);
      received_meta = meta;
    },
    subscriber_options);

  sensor_msgs::msg::CompressedImage image;
  image.header.frame_id = "camera";
  image.format = "png";
  image.data = {137, 80, 78, 71, 1, 2};

  ASSERT_TRUE(publisher.publish(image));
  ASSERT_TRUE(spin_until(*publisher_node, *subscriber_node, [&received]() {
      return static_cast<bool>(received);
  }));
  EXPECT_EQ(received->header.frame_id, "camera");
  EXPECT_EQ(received->format, "png");
  EXPECT_EQ(received->data, (std::vector<std::uint8_t>{137, 80, 78, 71, 1, 2}));
  EXPECT_EQ(received_meta.payload_size, 6U);
}

TEST_F(RclcppFixture, PointCloud2PublisherFeedsShmSubscriberCallback)
{
  auto publisher_node = std::make_shared<rclcpp::Node>("test_shm_cloud_direct_publisher");
  auto subscriber_node = std::make_shared<rclcpp::Node>("test_shm_cloud_direct_subscriber");

  shm_sensor_transport::ShmPublisherOptions publisher_options;
  publisher_options.shm_name = test_name("_direct_cloud");
  publisher_options.slot_count = 2;
  publisher_options.slot_size_bytes = 16;
  publisher_options.qos = rclcpp::QoS(1).reliable();
  shm_sensor_transport::ShmPointCloud2Publisher publisher(
    publisher_node.get(), "/direct/points", publisher_options);

  sensor_msgs::msg::PointCloud2::UniquePtr received;
  shm_sensor_transport_interfaces::msg::ShmPointCloud2 received_meta;
  shm_sensor_transport::ShmSubscriberOptions subscriber_options;
  subscriber_options.qos = publisher_options.qos;
  shm_sensor_transport::ShmPointCloud2Subscriber subscriber(
    subscriber_node.get(),
    "/direct/points",
    [&received, &received_meta](
      sensor_msgs::msg::PointCloud2::UniquePtr msg,
      const shm_sensor_transport_interfaces::msg::ShmPointCloud2 & meta)
    {
      received = std::move(msg);
      received_meta = meta;
    },
    subscriber_options);

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "lidar";
  cloud.height = 1;
  cloud.width = 1;
  cloud.point_step = 4;
  cloud.row_step = 4;
  cloud.is_dense = true;
  cloud.data = {20, 21, 22, 23};

  ASSERT_TRUE(publisher.publish(cloud));
  ASSERT_TRUE(spin_until(*publisher_node, *subscriber_node, [&received]() {
      return static_cast<bool>(received);
  }));
  EXPECT_EQ(received->header.frame_id, "lidar");
  EXPECT_EQ(received->data, (std::vector<std::uint8_t>{20, 21, 22, 23}));
  EXPECT_EQ(received_meta.payload_size, 4U);
}

TEST_F(RclcppFixture, RejectsOversizedPayloadWhenResizeDisabled)
{
  auto node = std::make_shared<rclcpp::Node>("test_shm_oversized_publisher");

  shm_sensor_transport::ShmPublisherOptions options;
  options.shm_name = test_name("_oversized");
  options.slot_count = 1;
  options.slot_size_bytes = 4;
  options.allow_resize = false;
  options.warn_on_oversized_frame = false;
  shm_sensor_transport::ShmImagePublisher publisher(node.get(), "/oversized/image", options);

  sensor_msgs::msg::Image image;
  image.height = 1;
  image.width = 4;
  image.encoding = "mono8";
  image.step = 4;
  image.data = {1, 2, 3, 4};
  ASSERT_TRUE(publisher.publish(image));

  image.width = 8;
  image.step = 8;
  image.data = {1, 2, 3, 4, 5, 6, 7, 8};
  EXPECT_FALSE(publisher.publish(image));
  EXPECT_EQ(publisher.last_error(), shm_sensor_transport::ShmPublishError::Oversized);
  EXPECT_EQ(publisher.slot_size(), 4U);
}

TEST_F(RclcppFixture, ResizesPayloadSlotWhenAllowed)
{
  auto node = std::make_shared<rclcpp::Node>("test_shm_resize_publisher");

  shm_sensor_transport::ShmPublisherOptions options;
  options.shm_name = test_name("_resize");
  options.slot_count = 1;
  options.slot_size_bytes = 0;
  options.allow_resize = true;
  shm_sensor_transport::ShmImagePublisher publisher(node.get(), "/resize/image", options);

  sensor_msgs::msg::Image image;
  image.height = 1;
  image.width = 4;
  image.encoding = "mono8";
  image.step = 4;
  image.data = {1, 2, 3, 4};
  ASSERT_TRUE(publisher.publish(image));
  EXPECT_EQ(publisher.slot_size(), 4U);

  image.width = 8;
  image.step = 8;
  image.data = {1, 2, 3, 4, 5, 6, 7, 8};
  ASSERT_TRUE(publisher.publish(image));
  EXPECT_EQ(publisher.slot_size(), 8U);
}

TEST_F(RclcppFixture, RejectsInvalidSlotCount)
{
  auto node = std::make_shared<rclcpp::Node>("test_shm_invalid_slot_count_publisher");

  shm_sensor_transport::ShmPublisherOptions options;
  options.slot_count = 0;
  EXPECT_THROW(
    shm_sensor_transport::ShmImagePublisher(node.get(), "/invalid/image", options),
    std::invalid_argument);
}

TEST_F(RclcppFixture, UnlinksSharedMemoryOnDestruction)
{
  const auto shm_name = test_name("_unlink");
  const auto shm_path = shm_sensor_transport::shared_memory_path(shm_name);
  {
    auto node = std::make_shared<rclcpp::Node>("test_shm_unlink_publisher");

    shm_sensor_transport::ShmPublisherOptions options;
    options.shm_name = shm_name;
    options.slot_count = 1;
    options.slot_size_bytes = 4;
    shm_sensor_transport::ShmImagePublisher publisher(node.get(), "/unlink/image", options);

    sensor_msgs::msg::Image image;
    image.height = 1;
    image.width = 4;
    image.encoding = "mono8";
    image.step = 4;
    image.data = {1, 2, 3, 4};
    ASSERT_TRUE(publisher.publish(image));
    ASSERT_EQ(::access(shm_path.c_str(), F_OK), 0);
  }
  EXPECT_NE(::access(shm_path.c_str(), F_OK), 0);
}

TEST_F(RclcppFixture, CompressedImageRelayComponentConstructs)
{
  EXPECT_NO_THROW({
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("common.input_topic", "/camera/image_raw/compressed"),
      rclcpp::Parameter("common.publish_status", false),
    });
    auto component =
    std::make_shared<shm_sensor_transport::ShmCompressedImageRelayComponent>(options);
    EXPECT_EQ(component->get_name(), std::string("shm_compressed_image_relay"));
  });
}
