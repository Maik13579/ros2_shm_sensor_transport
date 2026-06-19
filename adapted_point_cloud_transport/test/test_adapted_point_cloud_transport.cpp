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

#include "adapted_point_cloud_transport/adapted_point_cloud_transport.hpp"

namespace
{

struct CloudFrame
{
  uint32_t width = 0;
  uint32_t height = 0;
  std::string frame_id;
};

std::atomic<int> cloud_to_ros_count{0};
std::atomic<int> cloud_to_custom_count{0};

sensor_msgs::msg::PointCloud2 make_ros_cloud(uint32_t width)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = "map";
  msg.width = width;
  msg.height = 1;
  msg.point_step = 4;
  msg.row_step = msg.point_step * width;
  msg.data.resize(msg.row_step);
  return msg;
}

CloudFrame make_cloud(uint32_t width)
{
  return CloudFrame{width, 1, "map"};
}

void reset_counts()
{
  cloud_to_ros_count = 0;
  cloud_to_custom_count = 0;
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
struct rclcpp::TypeAdapter<CloudFrame, sensor_msgs::msg::PointCloud2>
{
  using is_specialized = std::true_type;
  using custom_type = CloudFrame;
  using ros_message_type = sensor_msgs::msg::PointCloud2;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    ++cloud_to_ros_count;
    destination = make_ros_cloud(source.width);
    destination.header.frame_id = source.frame_id;
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    ++cloud_to_custom_count;
    destination.width = source.width;
    destination.height = source.height;
    destination.frame_id = source.header.frame_id;
  }
};

using AdaptedCloud = rclcpp::TypeAdapter<CloudFrame, sensor_msgs::msg::PointCloud2>;

class AdaptedPointCloudTransportTest : public ::testing::Test
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

TEST_F(AdaptedPointCloudTransportTest, SubscriberBeforePublisherSwitchesToLocal)
{
  auto node = std::make_shared<rclcpp::Node>("adapted_cloud_sub_first");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  int received = 0;
  auto sub = adapted_point_cloud_transport::create_subscription<AdaptedCloud>(
    node, "/adapted_cloud_sub_first/points", [&](const std::shared_ptr<const CloudFrame> & msg) {
      ++received;
      EXPECT_EQ(21u, msg->width);
    });
  EXPECT_FALSE(sub.usingLocalTransport());

  auto pub = adapted_point_cloud_transport::create_publisher<AdaptedCloud>(
    node, "/adapted_cloud_sub_first/points");
  ASSERT_TRUE(spin_until(executor, [&] {return sub.usingLocalTransport();}));

  pub.publish(make_cloud(21));
  EXPECT_TRUE(spin_until(executor, [&] {return received == 1;}));
  EXPECT_EQ(0, cloud_to_ros_count.load());
}

TEST_F(AdaptedPointCloudTransportTest, PublisherBeforeSubscriberUsesLocalImmediately)
{
  auto node = std::make_shared<rclcpp::Node>("adapted_cloud_pub_first");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto pub = adapted_point_cloud_transport::create_publisher<AdaptedCloud>(
    node, "/adapted_cloud_pub_first/points");
  int received = 0;
  auto sub = adapted_point_cloud_transport::create_subscription<AdaptedCloud>(
    node, "/adapted_cloud_pub_first/points", [&](const std::shared_ptr<const CloudFrame> & msg) {
      ++received;
      EXPECT_EQ(22u, msg->width);
    });

  EXPECT_TRUE(sub.usingLocalTransport());
  pub.publish(make_cloud(22));
  EXPECT_TRUE(spin_until(executor, [&] {return received == 1;}));
  EXPECT_EQ(0, cloud_to_ros_count.load());
}

TEST_F(AdaptedPointCloudTransportTest, UniquePtrLocalDeliveryPreservesAddress)
{
  auto node = std::make_shared<rclcpp::Node>("adapted_cloud_unique_ptr_address");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  const CloudFrame * received_address = nullptr;
  auto pub = adapted_point_cloud_transport::create_publisher<AdaptedCloud>(
    node, "/adapted_cloud_unique_ptr_address/points");
  auto sub = adapted_point_cloud_transport::create_unique_subscription<AdaptedCloud>(
    node, "/adapted_cloud_unique_ptr_address/points",
    adapted_point_cloud_transport::Subscriber<AdaptedCloud>::UniqueCallback(
      [&](std::unique_ptr<CloudFrame> msg) {
        received_address = msg.get();
        EXPECT_EQ(25u, msg->width);
    }));
  ASSERT_TRUE(sub.usingLocalTransport());

  auto cloud = std::make_unique<CloudFrame>(make_cloud(25));
  const CloudFrame * published_address = cloud.get();
  pub.publish(std::move(cloud));

  EXPECT_TRUE(spin_until(executor, [&] {return received_address != nullptr;}));
  EXPECT_EQ(published_address, received_address);
  EXPECT_EQ(0, cloud_to_ros_count.load());
}

TEST_F(AdaptedPointCloudTransportTest, UniqueSubscriberReceivesPublicFallback)
{
  auto node = std::make_shared<rclcpp::Node>("adapted_cloud_unique_public_fallback");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  int received = 0;
  auto sub = adapted_point_cloud_transport::create_unique_subscription<AdaptedCloud>(
    node, "/adapted_cloud_unique_public_fallback/points",
    adapted_point_cloud_transport::Subscriber<AdaptedCloud>::UniqueCallback(
      [&](std::unique_ptr<CloudFrame> msg) {
        ++received;
        EXPECT_EQ(26u, msg->width);
    }));
  EXPECT_FALSE(sub.usingLocalTransport());

  auto raw_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/adapted_cloud_unique_public_fallback/points", 10);
  ASSERT_TRUE(spin_until(executor, [&] {return raw_pub->get_subscription_count() > 0;}));

  raw_pub->publish(make_ros_cloud(26));

  EXPECT_TRUE(spin_until(executor, [&] {return received == 1;}));
  EXPECT_EQ(1, cloud_to_custom_count.load());
}

TEST_F(AdaptedPointCloudTransportTest, UniqueSubscriberBeforePublisherSwitchesToLocal)
{
  auto node = std::make_shared<rclcpp::Node>("adapted_cloud_unique_sub_first");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  const CloudFrame * received_address = nullptr;
  auto sub = adapted_point_cloud_transport::create_unique_subscription<AdaptedCloud>(
    node, "/adapted_cloud_unique_sub_first/points",
    adapted_point_cloud_transport::Subscriber<AdaptedCloud>::UniqueCallback(
      [&](std::unique_ptr<CloudFrame> msg) {
        received_address = msg.get();
        EXPECT_EQ(27u, msg->width);
    }));
  EXPECT_FALSE(sub.usingLocalTransport());

  auto pub = adapted_point_cloud_transport::create_publisher<AdaptedCloud>(
    node, "/adapted_cloud_unique_sub_first/points");
  ASSERT_TRUE(spin_until(executor, [&] {return sub.usingLocalTransport();}));

  auto cloud = std::make_unique<CloudFrame>(make_cloud(27));
  const CloudFrame * published_address = cloud.get();
  pub.publish(std::move(cloud));

  EXPECT_TRUE(spin_until(executor, [&] {return received_address != nullptr;}));
  EXPECT_EQ(published_address, received_address);
  EXPECT_EQ(0, cloud_to_ros_count.load());
}

TEST_F(AdaptedPointCloudTransportTest, PublicSubscriberReceivesConvertedMessage)
{
  auto node = std::make_shared<rclcpp::Node>("adapted_cloud_public_only");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto pub = adapted_point_cloud_transport::create_publisher<AdaptedCloud>(
    node, "/adapted_cloud_public_only/points");
  int public_received = 0;
  auto public_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/adapted_cloud_public_only/points", 10,
    [&](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      ++public_received;
      EXPECT_EQ(23u, msg->width);
    });

  ASSERT_TRUE(spin_until(executor, [&] {return pub.getNumPublicSubscribers() > 0;}));
  pub.publish(make_cloud(23));
  EXPECT_EQ(0, cloud_to_ros_count.load());
  EXPECT_TRUE(spin_until(executor, [&] {return public_received == 1;}));
  EXPECT_EQ(1, cloud_to_ros_count.load());
}

TEST_F(AdaptedPointCloudTransportTest, LocalAndPublicSubscribersBothReceive)
{
  auto node = std::make_shared<rclcpp::Node>("adapted_cloud_mixed");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  int local_received = 0;
  auto local_sub = adapted_point_cloud_transport::create_subscription<AdaptedCloud>(
    node, "/adapted_cloud_mixed/points", [&](const std::shared_ptr<const CloudFrame> & msg) {
      ++local_received;
      EXPECT_EQ(24u, msg->width);
    });
  auto pub = adapted_point_cloud_transport::create_publisher<AdaptedCloud>(
    node, "/adapted_cloud_mixed/points");
  ASSERT_TRUE(spin_until(executor, [&] {return local_sub.usingLocalTransport();}));

  int public_received = 0;
  auto public_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/adapted_cloud_mixed/points", 10,
    [&](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      ++public_received;
      EXPECT_EQ(24u, msg->width);
    });
  ASSERT_TRUE(spin_until(executor, [&] {return pub.getNumPublicSubscribers() > 0;}));

  pub.publish(std::make_unique<CloudFrame>(make_cloud(24)));
  EXPECT_EQ(0, cloud_to_ros_count.load());
  EXPECT_TRUE(spin_until(executor, [&] {return local_received == 1 && public_received == 1;}));
  EXPECT_EQ(1, cloud_to_ros_count.load());
}

TEST_F(AdaptedPointCloudTransportTest, ContextsDoNotShareRegistry)
{
  auto context_a = std::make_shared<rclcpp::Context>();
  auto context_b = std::make_shared<rclcpp::Context>();
  context_a->init(0, nullptr);
  context_b->init(0, nullptr);

  auto node_a = std::make_shared<rclcpp::Node>(
    "adapted_cloud_context_a", rclcpp::NodeOptions().context(context_a));
  auto node_b = std::make_shared<rclcpp::Node>(
    "adapted_cloud_context_b", rclcpp::NodeOptions().context(context_b));

  auto pub = adapted_point_cloud_transport::create_publisher<AdaptedCloud>(
    node_a, "/adapted_cloud_context/points");
  auto sub = adapted_point_cloud_transport::create_subscription<AdaptedCloud>(
    node_b, "/adapted_cloud_context/points", [](const std::shared_ptr<const CloudFrame> &) {});

  EXPECT_FALSE(sub.usingLocalTransport());

  pub.shutdown();
  sub.shutdown();
  context_a->shutdown("test done");
  context_b->shutdown("test done");
}

TEST_F(AdaptedPointCloudTransportTest, PublisherUnregisterFallsBackToPublicTransport)
{
  auto node = std::make_shared<rclcpp::Node>("adapted_cloud_unregister");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto pub = std::make_unique<adapted_point_cloud_transport::Publisher<AdaptedCloud>>(
    node, "/adapted_cloud_unregister/points");
  auto sub = adapted_point_cloud_transport::create_subscription<AdaptedCloud>(
    node, "/adapted_cloud_unregister/points", [](const std::shared_ptr<const CloudFrame> &) {});
  ASSERT_TRUE(sub.usingLocalTransport());

  pub->shutdown();
  pub.reset();

  EXPECT_TRUE(spin_until(executor, [&] {return !sub.usingLocalTransport();}));
}

TEST_F(AdaptedPointCloudTransportTest, InvalidRosTypeIsRejectedByTrait)
{
  using Invalid = rclcpp::TypeAdapter<CloudFrame, sensor_msgs::msg::Image>;
  static_assert(!adapted_point_cloud_transport::detail::is_valid_adapter_v<Invalid>);
  SUCCEED();
}
