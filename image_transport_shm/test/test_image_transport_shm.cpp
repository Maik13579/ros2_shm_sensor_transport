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

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <image_transport/image_transport.hpp>
#include <image_transport/publisher_plugin.hpp>
#include <image_transport/subscriber_plugin.hpp>
#include <image_transport/transport_hints.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <shm_sensor_transport_interfaces/msg/shm_image.hpp>

namespace
{

class ImageTransportShmFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    ::setenv("ROS_LOG_DIR", "/tmp/image_transport_shm_test_logs", 1);
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

bool spin_until(
  rclcpp::Node & publisher_node,
  rclcpp::Node & subscriber_node,
  const std::function<bool()> & predicate)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node.get_node_base_interface());
  executor.add_node(subscriber_node.get_node_base_interface());

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!predicate() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(10));
  }
  return predicate();
}

sensor_msgs::msg::Image make_image()
{
  sensor_msgs::msg::Image msg;
  msg.header.frame_id = "camera";
  msg.height = 2;
  msg.width = 3;
  msg.encoding = "mono8";
  msg.is_bigendian = 0;
  msg.step = 3;
  msg.data = {1, 2, 3, 4, 5, 6};
  return msg;
}

std::string parameter_prefix(const std::string & topic)
{
  auto prefix = topic;
  if (!prefix.empty() && prefix.front() == '/') {
    prefix.erase(prefix.begin());
  }
  std::replace(prefix.begin(), prefix.end(), '/', '.');
  return prefix + "._shm";
}

}  // namespace

TEST_F(ImageTransportShmFixture, PluginlibLoadsPublisherAndSubscriber)
{
  pluginlib::ClassLoader<image_transport::PublisherPlugin> pub_loader(
    "image_transport", "image_transport::PublisherPlugin");
  pluginlib::ClassLoader<image_transport::SubscriberPlugin> sub_loader(
    "image_transport", "image_transport::SubscriberPlugin");

  const auto pub = pub_loader.createSharedInstance("image_transport/shm_pub");
  const auto sub = sub_loader.createSharedInstance("image_transport/shm_sub");

  EXPECT_EQ(pub->getTransportName(), "shm");
  EXPECT_EQ(sub->getTransportName(), "shm");
}

TEST_F(ImageTransportShmFixture, DeclaredPluginClassesIncludeShm)
{
  pluginlib::ClassLoader<image_transport::PublisherPlugin> pub_loader(
    "image_transport", "image_transport::PublisherPlugin");
  pluginlib::ClassLoader<image_transport::SubscriberPlugin> sub_loader(
    "image_transport", "image_transport::SubscriberPlugin");
  const auto pub_classes = pub_loader.getDeclaredClasses();
  const auto sub_classes = sub_loader.getDeclaredClasses();

  EXPECT_NE(std::find(pub_classes.begin(), pub_classes.end(), "image_transport/shm_pub"),
    pub_classes.end());
  EXPECT_NE(std::find(sub_classes.begin(), sub_classes.end(), "image_transport/shm_sub"),
    sub_classes.end());
}

TEST_F(ImageTransportShmFixture, PublishesAndSubscribesThroughImageTransport)
{
  const auto suffix = std::to_string(::getpid());
  const auto pub_node = std::make_shared<rclcpp::Node>("test_image_transport_shm_pub_" + suffix);
  const auto sub_node = std::make_shared<rclcpp::Node>("test_image_transport_shm_sub_" + suffix);
  const std::string topic = "/image_transport_shm_test_" + suffix + "/image";
  const auto prefix = parameter_prefix(topic);

  pub_node->declare_parameter<std::int64_t>(prefix + ".slot_size_bytes", 32);
  pub_node->declare_parameter<int>(prefix + ".slot_count", 3);

  sensor_msgs::msg::Image::ConstSharedPtr received;
  shm_sensor_transport_interfaces::msg::ShmImage::ConstSharedPtr meta;
  const auto qos = rclcpp::QoS(1).reliable();
  const auto meta_sub =
    sub_node->create_subscription<shm_sensor_transport_interfaces::msg::ShmImage>(
    topic + "/_shm", qos,
    [&meta](const shm_sensor_transport_interfaces::msg::ShmImage::ConstSharedPtr msg) {
      meta = msg;
    });
  (void)meta_sub;

  image_transport::ImageTransport pub_transport(pub_node);
  image_transport::ImageTransport sub_transport(sub_node);
  auto pub = pub_transport.advertise(topic, qos.get_rmw_qos_profile());
  auto hints = image_transport::TransportHints(sub_node.get(), "shm");
  auto sub = sub_transport.subscribe(
    topic,
    1,
    [&received](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
      received = msg;
    },
    nullptr,
    &hints);

  EXPECT_EQ(sub.getTransport(), "shm");

  const auto image = make_image();
  const auto connected = spin_until(
    *pub_node, *sub_node,
    [&pub, &sub]() {return pub.getNumSubscribers() > 0U && sub.getNumPublishers() > 0U;});
  ASSERT_TRUE(connected);

  for (int i = 0; i < 3 && (!received || !meta); ++i) {
    pub.publish(image);
    spin_until(
      *pub_node, *sub_node,
      [&received, &meta]() {return static_cast<bool>(received) && static_cast<bool>(meta);});
  }

  ASSERT_TRUE(received);
  ASSERT_TRUE(meta);
  EXPECT_EQ(received->header.frame_id, image.header.frame_id);
  EXPECT_EQ(received->height, image.height);
  EXPECT_EQ(received->width, image.width);
  EXPECT_EQ(received->encoding, image.encoding);
  EXPECT_EQ(received->step, image.step);
  EXPECT_EQ(received->data, image.data);
  EXPECT_EQ(meta->slot_size, 32U);
}
