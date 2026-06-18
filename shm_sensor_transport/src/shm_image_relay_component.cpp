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

#include "shm_sensor_transport/shm_image_relay_component.hpp"

#include <chrono>

#include "shm_sensor_transport/qos_utils.hpp"

namespace shm_sensor_transport
{

ShmImageRelayComponent::ShmImageRelayComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("shm_image_relay", options)
{
  params_ = declare_relay_parameters(*this, "image_raw");
  const auto qos = make_metadata_qos(
    params_.use_sensor_data_qos, params_.reliable, params_.depth);

  ShmPublisherOptions publisher_options;
  publisher_options.shm_name = params_.shm_name;
  publisher_options.slot_count = params_.slot_count;
  publisher_options.slot_size_bytes = params_.slot_size_bytes;
  publisher_options.allow_resize = params_.allow_resize;
  publisher_options.qos = qos;
  publisher_options.warn_on_oversized_frame = params_.warn_on_oversized_frame;
  shm_publisher_ = std::make_unique<ShmImagePublisher>(this, params_.input_topic,
      publisher_options);

  if (params_.publish_status && !params_.status_topic.empty()) {
    status_publisher_ =
      create_publisher<shm_sensor_transport_interfaces::msg::ShmTransportStatus>(
        params_.status_topic, rclcpp::QoS(1).reliable());
    if (params_.status_rate > 0.0) {
      const auto period = std::chrono::duration<double>(1.0 / params_.status_rate);
      status_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() {
          publish_status(shm_publisher_->is_open() ? "ok" : "waiting for first frame");
        });
    }
  }

  subscription_ = create_subscription<sensor_msgs::msg::Image>(
    params_.input_topic,
    qos,
    [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {image_callback(msg);});
}

ShmImageRelayComponent::~ShmImageRelayComponent() = default;

void ShmImageRelayComponent::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  const auto callback_time = now();
  if (params_.rate_limit_hz > 0.0 && last_published_time_.has_value()) {
    const auto min_period = rclcpp::Duration::from_seconds(1.0 / params_.rate_limit_hz);
    if ((callback_time - last_published_time_.value()) < min_period) {
      return;
    }
  }

  if (!shm_publisher_->publish(*msg)) {
    ++dropped_frames_;
    if (shm_publisher_->last_error() == ShmPublishError::Oversized ||
      shm_publisher_->last_error() == ShmPublishError::WriteFailed)
    {
      ++oversized_frames_;
    }
    return;
  }

  last_published_time_ = callback_time;
  ++published_frames_;
}

void ShmImageRelayComponent::publish_status(const std::string & message)
{
  if (!status_publisher_) {
    return;
  }
  shm_sensor_transport_interfaces::msg::ShmTransportStatus status;
  status.header.stamp = now();
  status.shm_name = shm_publisher_->shm_name();
  status.input_topic = params_.input_topic;
  status.meta_topic = params_.meta_topic;
  status.slot_count = shm_publisher_->slot_count();
  status.slot_size = shm_publisher_->slot_size();
  status.total_size = shm_publisher_->total_size();
  status.published_frames = published_frames_;
  status.dropped_frames = dropped_frames_;
  status.oversized_frames = oversized_frames_;
  status.initialized = shm_publisher_->is_open();
  status.message = message;
  status_publisher_->publish(status);
}

}  // namespace shm_sensor_transport
