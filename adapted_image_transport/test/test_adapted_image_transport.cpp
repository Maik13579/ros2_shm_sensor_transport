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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "adapted_image_transport/adapted_image_transport.hpp"

namespace
{

struct ImageFrame
{
  uint32_t width = 0;
  uint32_t height = 0;
  std::string frame_id;
};

std::atomic<int> image_to_ros_count{0};
std::atomic<int> image_to_custom_count{0};

sensor_msgs::msg::Image make_ros_image(uint32_t width)
{
  sensor_msgs::msg::Image msg;
  msg.header.frame_id = "camera";
  msg.width = width;
  msg.height = 2;
  msg.encoding = "mono8";
  msg.step = width;
  msg.data.resize(width * msg.height);
  return msg;
}

ImageFrame make_frame(uint32_t width)
{
  return ImageFrame{width, 2, "camera"};
}

void reset_counts()
{
  image_to_ros_count = 0;
  image_to_custom_count = 0;
}

template<typename Predicate>
bool spin_until(
  rclcpp::executors::SingleThreadedExecutor & executor,
  Predicate predicate,
  std::chrono::milliseconds timeout = std::chrono::milliseconds(1500))
{
  const auto start = std::chrono::steady_clock::now();
  while (!predicate() && std::chrono::steady_clock::now() - start < timeout) {
    executor.spin_some(std::chrono::milliseconds(10));
  }
  return predicate();
}

}  // namespace

template<>
struct rclcpp::TypeAdapter<ImageFrame, sensor_msgs::msg::Image>
{
  using is_specialized = std::true_type;
  using custom_type = ImageFrame;
  using ros_message_type = sensor_msgs::msg::Image;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    ++image_to_ros_count;
    destination = make_ros_image(source.width);
    destination.header.frame_id = source.frame_id;
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    ++image_to_custom_count;
    destination.width = source.width;
    destination.height = source.height;
    destination.frame_id = source.header.frame_id;
  }
};

using AdaptedImage = rclcpp::TypeAdapter<ImageFrame, sensor_msgs::msg::Image>;

class AdaptedImageTransportTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    reset_counts();
  }
};

TEST_F(AdaptedImageTransportTest, SubscriberBeforePublisherSwitchesToLocal)
{
  auto node = std::make_shared<rclcpp::Node>("adapted_image_sub_first");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  int received = 0;
  auto sub = adapted_image_transport::create_subscription<AdaptedImage>(
    node, "/adapted_image_sub_first/image", [&](const std::shared_ptr<const ImageFrame> & msg) {
      ++received;
      EXPECT_EQ(11u, msg->width);
    });
  EXPECT_FALSE(sub.usingLocalTransport());

  auto pub = adapted_image_transport::create_publisher<AdaptedImage>(
    node, "/adapted_image_sub_first/image");
  ASSERT_TRUE(spin_until(executor, [&] {return sub.usingLocalTransport();}));

  pub.publish(make_frame(11));
  EXPECT_TRUE(spin_until(executor, [&] {return received == 1;}));
  EXPECT_EQ(0, image_to_ros_count.load());
}

TEST_F(AdaptedImageTransportTest, PublisherBeforeSubscriberUsesLocalImmediately)
{
  auto node = std::make_shared<rclcpp::Node>("adapted_image_pub_first");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto pub = adapted_image_transport::create_publisher<AdaptedImage>(
    node, "/adapted_image_pub_first/image");
  int received = 0;
  auto sub = adapted_image_transport::create_subscription<AdaptedImage>(
    node, "/adapted_image_pub_first/image", [&](const std::shared_ptr<const ImageFrame> & msg) {
      ++received;
      EXPECT_EQ(12u, msg->width);
    });

  EXPECT_TRUE(sub.usingLocalTransport());
  pub.publish(make_frame(12));
  EXPECT_TRUE(spin_until(executor, [&] {return received == 1;}));
  EXPECT_EQ(0, image_to_ros_count.load());
}

TEST_F(AdaptedImageTransportTest, UniquePtrLocalDeliveryPreservesAddress)
{
  auto node = std::make_shared<rclcpp::Node>("adapted_image_unique_ptr_address");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  const ImageFrame * received_address = nullptr;
  auto pub = adapted_image_transport::create_publisher<AdaptedImage>(
    node, "/adapted_image_unique_ptr_address/image");
  auto sub = adapted_image_transport::create_unique_subscription<AdaptedImage>(
    node, "/adapted_image_unique_ptr_address/image",
    adapted_image_transport::Subscriber<AdaptedImage>::UniqueCallback(
      [&](std::unique_ptr<ImageFrame> msg) {
        received_address = msg.get();
        EXPECT_EQ(15u, msg->width);
    }));
  ASSERT_TRUE(sub.usingLocalTransport());

  auto frame = std::make_unique<ImageFrame>(make_frame(15));
  const ImageFrame * published_address = frame.get();
  pub.publish(std::move(frame));

  EXPECT_TRUE(spin_until(executor, [&] {return received_address != nullptr;}));
  EXPECT_EQ(published_address, received_address);
  EXPECT_EQ(0, image_to_ros_count.load());
}

TEST_F(AdaptedImageTransportTest, UniqueSubscriberReceivesPublicFallback)
{
  auto node = std::make_shared<rclcpp::Node>("adapted_image_unique_public_fallback");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  int received = 0;
  auto sub = adapted_image_transport::create_unique_subscription<AdaptedImage>(
    node, "/adapted_image_unique_public_fallback/image",
    adapted_image_transport::Subscriber<AdaptedImage>::UniqueCallback(
      [&](std::unique_ptr<ImageFrame> msg) {
        ++received;
        EXPECT_EQ(16u, msg->width);
    }));
  EXPECT_FALSE(sub.usingLocalTransport());

  auto raw_pub = node->create_publisher<sensor_msgs::msg::Image>(
    "/adapted_image_unique_public_fallback/image", 10);
  ASSERT_TRUE(spin_until(executor, [&] {return raw_pub->get_subscription_count() > 0;}));

  raw_pub->publish(make_ros_image(16));

  EXPECT_TRUE(spin_until(executor, [&] {return received == 1;}));
  EXPECT_EQ(1, image_to_custom_count.load());
}

TEST_F(AdaptedImageTransportTest, UniqueSubscriberBeforePublisherSwitchesToLocal)
{
  auto node = std::make_shared<rclcpp::Node>("adapted_image_unique_sub_first");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  const ImageFrame * received_address = nullptr;
  auto sub = adapted_image_transport::create_unique_subscription<AdaptedImage>(
    node, "/adapted_image_unique_sub_first/image",
    adapted_image_transport::Subscriber<AdaptedImage>::UniqueCallback(
      [&](std::unique_ptr<ImageFrame> msg) {
        received_address = msg.get();
        EXPECT_EQ(17u, msg->width);
    }));
  EXPECT_FALSE(sub.usingLocalTransport());

  auto pub = adapted_image_transport::create_publisher<AdaptedImage>(
    node, "/adapted_image_unique_sub_first/image");
  ASSERT_TRUE(spin_until(executor, [&] {return sub.usingLocalTransport();}));

  auto frame = std::make_unique<ImageFrame>(make_frame(17));
  const ImageFrame * published_address = frame.get();
  pub.publish(std::move(frame));

  EXPECT_TRUE(spin_until(executor, [&] {return received_address != nullptr;}));
  EXPECT_EQ(published_address, received_address);
  EXPECT_EQ(0, image_to_ros_count.load());
}

TEST_F(AdaptedImageTransportTest, PublicSubscriberReceivesConvertedMessage)
{
  auto node = std::make_shared<rclcpp::Node>("adapted_image_public_only");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto pub = adapted_image_transport::create_publisher<AdaptedImage>(
    node, "/adapted_image_public_only/image");
  int public_received = 0;
  auto public_sub = node->create_subscription<sensor_msgs::msg::Image>(
    "/adapted_image_public_only/image", 10, [&](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
      ++public_received;
      EXPECT_EQ(13u, msg->width);
    });

  ASSERT_TRUE(spin_until(executor, [&] {return pub.getNumPublicSubscribers() > 0;}));
  pub.publish(make_frame(13));
  EXPECT_EQ(0, image_to_ros_count.load());
  EXPECT_TRUE(spin_until(executor, [&] {return public_received == 1;}));
  EXPECT_EQ(1, image_to_ros_count.load());
}

TEST_F(AdaptedImageTransportTest, LocalAndPublicSubscribersBothReceive)
{
  auto node = std::make_shared<rclcpp::Node>("adapted_image_mixed");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  int local_received = 0;
  auto local_sub = adapted_image_transport::create_subscription<AdaptedImage>(
    node, "/adapted_image_mixed/image", [&](const std::shared_ptr<const ImageFrame> & msg) {
      ++local_received;
      EXPECT_EQ(14u, msg->width);
    });
  auto pub = adapted_image_transport::create_publisher<AdaptedImage>(
    node, "/adapted_image_mixed/image");
  ASSERT_TRUE(spin_until(executor, [&] {return local_sub.usingLocalTransport();}));

  int public_received = 0;
  auto public_sub = node->create_subscription<sensor_msgs::msg::Image>(
    "/adapted_image_mixed/image", 10, [&](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
      ++public_received;
      EXPECT_EQ(14u, msg->width);
    });
  ASSERT_TRUE(spin_until(executor, [&] {return pub.getNumPublicSubscribers() > 0;}));

  pub.publish(std::make_unique<ImageFrame>(make_frame(14)));
  EXPECT_EQ(0, image_to_ros_count.load());
  EXPECT_TRUE(spin_until(executor, [&] {return local_received == 1 && public_received == 1;}));
  EXPECT_EQ(1, image_to_ros_count.load());
}

TEST_F(AdaptedImageTransportTest, ContextsDoNotShareRegistry)
{
  auto context_a = std::make_shared<rclcpp::Context>();
  auto context_b = std::make_shared<rclcpp::Context>();
  context_a->init(0, nullptr);
  context_b->init(0, nullptr);

  auto node_a = std::make_shared<rclcpp::Node>(
    "adapted_image_context_a", rclcpp::NodeOptions().context(context_a));
  auto node_b = std::make_shared<rclcpp::Node>(
    "adapted_image_context_b", rclcpp::NodeOptions().context(context_b));

  auto pub = adapted_image_transport::create_publisher<AdaptedImage>(
    node_a, "/adapted_image_context/image");
  auto sub = adapted_image_transport::create_subscription<AdaptedImage>(
    node_b, "/adapted_image_context/image", [](const std::shared_ptr<const ImageFrame> &) {});

  EXPECT_FALSE(sub.usingLocalTransport());

  pub.shutdown();
  sub.shutdown();
  context_a->shutdown("test done");
  context_b->shutdown("test done");
}

TEST_F(AdaptedImageTransportTest, PublisherUnregisterFallsBackToPublicTransport)
{
  auto node = std::make_shared<rclcpp::Node>("adapted_image_unregister");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto pub = std::make_unique<adapted_image_transport::Publisher<AdaptedImage>>(
    node, "/adapted_image_unregister/image");
  auto sub = adapted_image_transport::create_subscription<AdaptedImage>(
    node, "/adapted_image_unregister/image", [](const std::shared_ptr<const ImageFrame> &) {});
  ASSERT_TRUE(sub.usingLocalTransport());

  pub->shutdown();
  pub.reset();

  EXPECT_TRUE(spin_until(executor, [&] {return !sub.usingLocalTransport();}));
}

TEST_F(AdaptedImageTransportTest, InvalidRosTypeIsRejectedByTrait)
{
  using Invalid = rclcpp::TypeAdapter<ImageFrame, sensor_msgs::msg::PointCloud2>;
  static_assert(!adapted_image_transport::detail::is_valid_adapter_v<Invalid>);
  SUCCEED();
}
