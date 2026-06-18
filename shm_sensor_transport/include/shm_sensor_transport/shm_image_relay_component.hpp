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
#include <sensor_msgs/msg/image.hpp>
#include <shm_sensor_transport_interfaces/msg/shm_image.hpp>
#include <shm_sensor_transport_interfaces/msg/shm_transport_status.hpp>

#include "shm_sensor_transport/parameter_utils.hpp"
#include "shm_sensor_transport/shm_publisher.hpp"

namespace shm_sensor_transport
{

/// @brief Composable relay from sensor_msgs::msg::Image to shared-memory metadata.
class ShmImageRelayComponent : public rclcpp::Node
{
public:
  /// @brief Construct the relay node, declare parameters, and create ROS entities.
  explicit ShmImageRelayComponent(const rclcpp::NodeOptions & options);

  /// @brief Unlink the owned shared-memory object when the relay node is destroyed.
  ~ShmImageRelayComponent() override;

private:
  /// @brief Copy one Image payload into shared memory and publish metadata.
  void image_callback(sensor_msgs::msg::Image::ConstSharedPtr msg);

  /// @brief Publish a transport status sample when status publishing is enabled.
  void publish_status(const std::string & message);

  RelayParameters params_;
  std::unique_ptr<ShmImagePublisher> shm_publisher_;
  std::uint64_t published_frames_{0};
  std::uint64_t dropped_frames_{0};
  std::uint64_t oversized_frames_{0};
  std::optional<rclcpp::Time> last_published_time_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<shm_sensor_transport_interfaces::msg::ShmTransportStatus>::SharedPtr
    status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace shm_sensor_transport
