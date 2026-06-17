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

#include "shm_sensor_transport_test/benchmark_image_publisher_component.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>

namespace shm_sensor_transport_test
{

BenchmarkImagePublisherComponent::BenchmarkImagePublisherComponent(
  const rclcpp::NodeOptions & options)
: Node("benchmark_image_publisher", options)
{
  topic_ = declare_parameter<std::string>("topic", "/benchmark/image_raw");
  frame_count_ = declare_parameter<int>("frames", 100);
  payload_size_ = declare_parameter<int>("payload_size", 1920 * 1080 * 3);
  const auto rate_hz = std::max(1.0, declare_parameter<double>("rate_hz", 30.0));
  const auto start_delay_sec = std::max(0.0, declare_parameter<double>("start_delay_sec", 0.0));

  payload_size_ = std::max(1, payload_size_);
  start_time_ = now() + rclcpp::Duration::from_seconds(start_delay_sec);

  publisher_ = create_publisher<sensor_msgs::msg::Image>(topic_, rclcpp::SensorDataQoS());
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate_hz)),
    [this]() {publish_frame();});
}

void BenchmarkImagePublisherComponent::publish_frame()
{
  if (now() < start_time_) {
    return;
  }
  if (published_frames_ >= frame_count_) {
    return;
  }

  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  fill_payload(*msg, published_frames_);
  msg->header.stamp = now();
  msg->header.frame_id = std::to_string(published_frames_);
  msg->height = 1U;
  msg->width = static_cast<std::uint32_t>(payload_size_);
  msg->encoding = "mono8";
  msg->is_bigendian = 0U;
  msg->step = static_cast<std::uint32_t>(payload_size_);
  publisher_->publish(std::move(msg));
  ++published_frames_;
}

void BenchmarkImagePublisherComponent::fill_payload(
  sensor_msgs::msg::Image & msg, const int sequence) const
{
  msg.data.resize(static_cast<std::size_t>(payload_size_));
  for (std::size_t index = 0; index < msg.data.size(); ++index) {
    msg.data[index] = static_cast<std::uint8_t>((sequence + static_cast<int>(index)) % 251);
  }
}

}  // namespace shm_sensor_transport_test
