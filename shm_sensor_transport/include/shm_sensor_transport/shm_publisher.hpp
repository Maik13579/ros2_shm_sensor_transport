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
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "shm_sensor_transport/qos_utils.hpp"
#include "shm_sensor_transport/shm_message_traits.hpp"
#include "shm_sensor_transport/shm_name.hpp"
#include "shm_sensor_transport/shm_ring_buffer.hpp"

namespace shm_sensor_transport
{

/// @brief Optional settings for C++ shared-memory publishers.
struct ShmPublisherOptions
{
  std::string shm_name;
  int slot_count{8};
  std::uint64_t slot_size_bytes{0};
  bool allow_resize{false};
  rclcpp::QoS qos{make_metadata_qos(true, false, 1)};
  bool warn_on_oversized_frame{true};
};

enum class ShmPublishError
{
  None,
  Oversized,
  CreateFailed,
  WriteFailed,
};

/// @brief Publish normal ROS sensor messages through a POSIX shared-memory ring.
template<typename SensorMessageT>
class ShmPublisher
{
public:
  using Traits = ShmMessageTraits<SensorMessageT>;
  using MetadataMessage = typename Traits::MetadataMessage;

  ShmPublisher(
    rclcpp::Node * node,
    const std::string & topic,
    ShmPublisherOptions options = {})
  : node_(node),
    topic_(topic),
    metadata_topic_(resolve_metadata_topic(topic)),
    options_(std::move(options))
  {
    if (node_ == nullptr) {
      throw std::invalid_argument("node must not be null");
    }
    if (options_.slot_count <= 0) {
      throw std::invalid_argument("slot_count must be positive");
    }
    metadata_publisher_ = node_->create_publisher<MetadataMessage>(metadata_topic_, options_.qos);
  }

  ~ShmPublisher()
  {
    ring_.unlink();
    ring_.close();
  }

  ShmPublisher(const ShmPublisher &) = delete;
  ShmPublisher & operator=(const ShmPublisher &) = delete;

  /// @brief Write one sensor payload into shared memory and publish its metadata.
  bool publish(const SensorMessageT & msg)
  {
    last_error_ = ShmPublishError::None;
    last_error_message_.clear();

    const auto & payload = Traits::payload(msg);
    const auto payload_size = static_cast<std::uint64_t>(payload.size());
    if (!ensure_buffer(payload_size)) {
      return false;
    }

    const auto write = ring_.write(payload.data(), payload.size());
    if (!write.success) {
      last_error_ = ShmPublishError::WriteFailed;
      last_error_message_ = write.message;
      if (options_.warn_on_oversized_frame) {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 5000, "%s", write.message.c_str());
      }
      return false;
    }

    metadata_publisher_->publish(Traits::encode(msg, ring_, write));
    return true;
  }

  typename rclcpp::Publisher<MetadataMessage>::SharedPtr metadata_publisher() const
  {
    return metadata_publisher_;
  }

  const std::string & metadata_topic() const noexcept
  {
    return metadata_topic_;
  }

  const std::string & shm_name() const noexcept
  {
    return ring_.name();
  }

  bool is_open() const noexcept
  {
    return ring_.is_open();
  }

  std::uint32_t slot_count() const noexcept
  {
    return ring_.slot_count();
  }

  std::uint64_t slot_size() const noexcept
  {
    return ring_.slot_size();
  }

  std::uint64_t total_size() const noexcept
  {
    return ring_.total_size();
  }

  ShmPublishError last_error() const noexcept
  {
    return last_error_;
  }

private:
  bool ensure_buffer(const std::uint64_t payload_size)
  {
    if (ring_.is_open() && payload_size <= ring_.slot_size()) {
      return true;
    }
    if (ring_.is_open() && !options_.allow_resize) {
      last_error_ = ShmPublishError::Oversized;
      last_error_message_ = "payload is larger than the configured slot size";
      if (options_.warn_on_oversized_frame) {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 5000,
          "Payload is %zu bytes, larger than shared-memory slot size %zu",
          static_cast<std::size_t>(payload_size), static_cast<std::size_t>(ring_.slot_size()));
      }
      return false;
    }

    const auto slot_size = options_.slot_size_bytes == 0U ? payload_size : options_.slot_size_bytes;
    if (slot_size == 0U || payload_size > slot_size) {
      last_error_ = ShmPublishError::Oversized;
      last_error_message_ = "payload is larger than the configured slot size";
      return false;
    }

    const auto shm_name = options_.shm_name.empty() ?
      make_shared_memory_name(topic_) :
      normalize_shared_memory_name(options_.shm_name);
    try {
      ring_.create(shm_name, static_cast<std::uint32_t>(options_.slot_count), slot_size);
    } catch (const std::exception & error) {
      last_error_ = ShmPublishError::CreateFailed;
      last_error_message_ = error.what();
      RCLCPP_ERROR(node_->get_logger(), "Failed to create shared memory: %s", error.what());
      return false;
    }

    RCLCPP_INFO(
      node_->get_logger(), "Created shared-memory ring %s with %d slots of %zu bytes",
      ring_.name().c_str(), options_.slot_count, static_cast<std::size_t>(slot_size));
    return true;
  }

  rclcpp::Node * node_;
  std::string topic_;
  std::string metadata_topic_;
  ShmPublisherOptions options_;
  ShmRingBuffer ring_;
  ShmPublishError last_error_{ShmPublishError::None};
  std::string last_error_message_;
  typename rclcpp::Publisher<MetadataMessage>::SharedPtr metadata_publisher_;
};

using ShmImagePublisher = ShmPublisher<sensor_msgs::msg::Image>;
using ShmPointCloud2Publisher = ShmPublisher<sensor_msgs::msg::PointCloud2>;

}  // namespace shm_sensor_transport
