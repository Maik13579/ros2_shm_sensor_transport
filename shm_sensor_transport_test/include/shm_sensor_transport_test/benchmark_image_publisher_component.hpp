// Copyright 2026 Maik Knof
//
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

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace shm_sensor_transport_test
{

/// @brief Composable C++ image publisher that emits sequence-checkable mono8 payloads.
class BenchmarkImagePublisherComponent : public rclcpp::Node
{
public:
  /// @brief Construct the publisher node and declare benchmark parameters.
  explicit BenchmarkImagePublisherComponent(const rclcpp::NodeOptions & options);

private:
  /// @brief Publish the next benchmark image frame until the configured count is reached.
  void publish_frame();

  /// @brief Fill the message payload with a benchmark pattern derived from the frame sequence.
  void fill_payload(sensor_msgs::msg::Image & msg, int sequence) const;

  std::string topic_;
  rclcpp::Time start_time_;
  int frame_count_{100};
  int payload_size_{0};
  int published_frames_{0};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace shm_sensor_transport_test
