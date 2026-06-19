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

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "shm_sensor_transport/qos_utils.hpp"
#include "shm_sensor_transport/shm_handle.hpp"
#include "shm_sensor_transport/shm_message_traits.hpp"

namespace shm_sensor_transport
{

/// @brief Optional settings for C++ shared-memory subscribers.
struct ShmSubscriberOptions
{
  rclcpp::QoS qos{make_metadata_qos(true, false, 1)};
  double rate_limit_hz{0.0};
};

/// @brief Subscribe to shared-memory metadata and invoke callbacks with normal sensor messages.
template<typename SensorMessageT>
class ShmSubscriber
{
public:
  using Traits = ShmMessageTraits<SensorMessageT>;
  using MetadataMessage = typename Traits::MetadataMessage;
  using Callback = std::function<void(std::unique_ptr<SensorMessageT>, const MetadataMessage &)>;

  ShmSubscriber(
    rclcpp::Node * node,
    const std::string & topic,
    Callback callback,
    ShmSubscriberOptions options = {})
  : node_(node),
    callback_(std::move(callback)),
    metadata_topic_(resolve_metadata_topic(topic)),
    rate_limit_hz_(std::max(0.0, options.rate_limit_hz))
  {
    if (node_ == nullptr) {
      throw std::invalid_argument("node must not be null");
    }
    if (!callback_) {
      throw std::invalid_argument("callback must not be empty");
    }

    subscription_ = node_->create_subscription<MetadataMessage>(
      metadata_topic_,
      options.qos,
      [this](typename MetadataMessage::ConstSharedPtr meta) {metadata_callback(*meta);});
  }

  /// @brief Return the underlying ROS metadata subscription.
  typename rclcpp::Subscription<MetadataMessage>::SharedPtr subscription() const
  {
    return subscription_;
  }

  /// @brief Return the hidden metadata topic this subscriber listens to.
  const std::string & metadata_topic() const noexcept
  {
    return metadata_topic_;
  }

  /// @brief Close the cached shared-memory mapping.
  void close()
  {
    handle_.close();
  }

private:
  void metadata_callback(const MetadataMessage & meta)
  {
    const auto now = std::chrono::steady_clock::now();
    if (!rate_limit_allows_callback(now)) {
      return;
    }

    try {
      auto payload = handle_.copy_payload(meta);
      auto msg = Traits::decode(std::move(payload), meta);
      callback_(std::move(msg), meta);
      last_callback_time_ = now;
    } catch (const ShmFrameInvalid & error) {
      RCLCPP_DEBUG(
        node_->get_logger(), "Dropped invalid shared-memory frame: %s", error.what());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        node_->get_logger(), "Shared-memory subscriber callback failed: %s", error.what());
    }
  }

  bool rate_limit_allows_callback(const std::chrono::steady_clock::time_point now) const
  {
    if (rate_limit_hz_ <= 0.0 || !last_callback_time_.has_value()) {
      return true;
    }
    const auto min_period = std::chrono::duration<double>(1.0 / rate_limit_hz_);
    return now - last_callback_time_.value() >= min_period;
  }

  rclcpp::Node * node_;
  Callback callback_;
  std::string metadata_topic_;
  double rate_limit_hz_{0.0};
  ShmHandle handle_;
  std::optional<std::chrono::steady_clock::time_point> last_callback_time_;
  typename rclcpp::Subscription<MetadataMessage>::SharedPtr subscription_;
};

using ShmCompressedImageSubscriber = ShmSubscriber<sensor_msgs::msg::CompressedImage>;
using ShmImageSubscriber = ShmSubscriber<sensor_msgs::msg::Image>;
using ShmPointCloud2Subscriber = ShmSubscriber<sensor_msgs::msg::PointCloud2>;

}  // namespace shm_sensor_transport
