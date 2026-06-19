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
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <point_cloud_transport/publisher_plugin.hpp>
#include <point_cloud_transport/subscriber_plugin.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <shm_sensor_transport_interfaces/msg/shm_point_cloud2.hpp>

namespace
{

class PointCloudTransportShmFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    ::setenv("ROS_LOG_DIR", "/tmp/point_cloud_transport_shm_test_logs", 1);
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

sensor_msgs::msg::PointCloud2 make_cloud()
{
  sensor_msgs::msg::PointField field;
  field.name = "x";
  field.offset = 0;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;

  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = "lidar";
  msg.height = 1;
  msg.width = 2;
  msg.fields = {field};
  msg.is_bigendian = false;
  msg.point_step = 4;
  msg.row_step = 8;
  msg.is_dense = true;
  msg.data = {1, 2, 3, 4, 5, 6, 7, 8};
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

TEST_F(PointCloudTransportShmFixture, PluginlibLoadsPublisherAndSubscriber)
{
  pluginlib::ClassLoader<point_cloud_transport::PublisherPlugin> pub_loader(
    "point_cloud_transport", "point_cloud_transport::PublisherPlugin");
  pluginlib::ClassLoader<point_cloud_transport::SubscriberPlugin> sub_loader(
    "point_cloud_transport", "point_cloud_transport::SubscriberPlugin");

  const auto pub = pub_loader.createSharedInstance("point_cloud_transport/shm_pub");
  const auto sub = sub_loader.createSharedInstance("point_cloud_transport/shm_sub");

  EXPECT_EQ(pub->getTransportName(), "shm");
  EXPECT_EQ(sub->getTransportName(), "shm");
  EXPECT_EQ(pub->getDataType(), "shm_sensor_transport_interfaces/msg/ShmPointCloud2");
  EXPECT_EQ(sub->getDataType(), "shm_sensor_transport_interfaces/msg/ShmPointCloud2");
}

TEST_F(PointCloudTransportShmFixture, DeclaredPluginClassesIncludeShm)
{
  pluginlib::ClassLoader<point_cloud_transport::PublisherPlugin> pub_loader(
    "point_cloud_transport", "point_cloud_transport::PublisherPlugin");
  pluginlib::ClassLoader<point_cloud_transport::SubscriberPlugin> sub_loader(
    "point_cloud_transport", "point_cloud_transport::SubscriberPlugin");
  const auto pub_classes = pub_loader.getDeclaredClasses();
  const auto sub_classes = sub_loader.getDeclaredClasses();

  EXPECT_NE(
    std::find(pub_classes.begin(), pub_classes.end(), "point_cloud_transport/shm_pub"),
    pub_classes.end());
  EXPECT_NE(
    std::find(sub_classes.begin(), sub_classes.end(), "point_cloud_transport/shm_sub"),
    sub_classes.end());
}

TEST_F(PointCloudTransportShmFixture, PublishesAndSubscribesThroughPointCloudTransport)
{
  const auto suffix = std::to_string(::getpid());
  const auto pub_node =
    std::make_shared<rclcpp::Node>("test_point_cloud_transport_shm_pub_" + suffix);
  const auto sub_node =
    std::make_shared<rclcpp::Node>("test_point_cloud_transport_shm_sub_" + suffix);
  const std::string topic = "/point_cloud_transport_shm_test_" + suffix + "/points";
  const auto prefix = parameter_prefix(topic);

  pub_node->declare_parameter<std::int64_t>(prefix + ".slot_size_bytes", 32);
  pub_node->declare_parameter<int>(prefix + ".slot_count", 3);

  sensor_msgs::msg::PointCloud2::ConstSharedPtr received;
  shm_sensor_transport_interfaces::msg::ShmPointCloud2::ConstSharedPtr meta;
  const auto qos = rclcpp::QoS(1).reliable();
  const auto meta_sub =
    sub_node->create_subscription<shm_sensor_transport_interfaces::msg::ShmPointCloud2>(
    topic + "/_shm", qos,
    [&meta](const shm_sensor_transport_interfaces::msg::ShmPointCloud2::ConstSharedPtr msg) {
      meta = msg;
    });
  (void)meta_sub;

  point_cloud_transport::PointCloudTransport pub_transport(pub_node);
  point_cloud_transport::PointCloudTransport sub_transport(sub_node);
  auto pub = pub_transport.advertise(topic, qos.get_rmw_qos_profile());
  auto hints = point_cloud_transport::TransportHints("shm");
  auto sub = sub_transport.subscribe(
    topic,
    qos.get_rmw_qos_profile(),
    [&received](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
      received = msg;
    },
    nullptr,
    &hints);

  EXPECT_EQ(sub.getTransport(), "shm");

  const auto cloud = make_cloud();
  const auto connected = spin_until(
    *pub_node, *sub_node,
    [&pub, &sub]() {return pub.getNumSubscribers() > 0U && sub.getNumPublishers() > 0U;});
  ASSERT_TRUE(connected);

  for (int i = 0; i < 3 && (!received || !meta); ++i) {
    pub.publish(cloud);
    spin_until(
      *pub_node, *sub_node,
      [&received, &meta]() {return static_cast<bool>(received) && static_cast<bool>(meta);});
  }

  ASSERT_TRUE(received);
  ASSERT_TRUE(meta);
  EXPECT_EQ(received->header.frame_id, cloud.header.frame_id);
  EXPECT_EQ(received->height, cloud.height);
  EXPECT_EQ(received->width, cloud.width);
  EXPECT_EQ(received->fields.size(), cloud.fields.size());
  ASSERT_FALSE(received->fields.empty());
  EXPECT_EQ(received->fields.front().name, cloud.fields.front().name);
  EXPECT_EQ(received->point_step, cloud.point_step);
  EXPECT_EQ(received->row_step, cloud.row_step);
  EXPECT_EQ(received->is_dense, cloud.is_dense);
  EXPECT_EQ(received->data, cloud.data);
  EXPECT_EQ(meta->slot_size, 32U);
}
