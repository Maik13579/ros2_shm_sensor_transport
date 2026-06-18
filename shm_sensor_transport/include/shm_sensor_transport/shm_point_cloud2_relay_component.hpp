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

#pragma once

#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <shm_sensor_transport_interfaces/msg/shm_point_cloud2.hpp>
#include <shm_sensor_transport_interfaces/msg/shm_transport_status.hpp>

#include "shm_sensor_transport/parameter_utils.hpp"
#include "shm_sensor_transport/shm_ring_buffer.hpp"

namespace shm_sensor_transport
{

/// @brief Composable relay from sensor_msgs::msg::PointCloud2 to shared-memory metadata.
class ShmPointCloud2RelayComponent : public rclcpp::Node
{
public:
  /// @brief Construct the relay node, declare parameters, and create ROS entities.
  explicit ShmPointCloud2RelayComponent(const rclcpp::NodeOptions & options);

  /// @brief Unlink the owned shared-memory object when the relay node is destroyed.
  ~ShmPointCloud2RelayComponent() override;

private:
  /// @brief Copy one PointCloud2 payload into shared memory and publish metadata.
  void cloud_callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  /// @brief Lazily create or resize the ring buffer for the payload size.
  bool ensure_buffer(std::uint64_t payload_size);

  /// @brief Publish a transport status sample when status publishing is enabled.
  void publish_status(const std::string & message);

  RelayParameters params_;
  ShmRingBuffer ring_;
  std::uint64_t published_frames_{0};
  std::uint64_t dropped_frames_{0};
  std::uint64_t oversized_frames_{0};
  std::optional<rclcpp::Time> last_published_time_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<shm_sensor_transport_interfaces::msg::ShmPointCloud2>::SharedPtr
    metadata_publisher_;
  rclcpp::Publisher<shm_sensor_transport_interfaces::msg::ShmTransportStatus>::SharedPtr
    status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace shm_sensor_transport
