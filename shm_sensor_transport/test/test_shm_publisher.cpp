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
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "shm_sensor_transport/shm_handle.hpp"
#include "shm_sensor_transport/shm_publisher.hpp"

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
