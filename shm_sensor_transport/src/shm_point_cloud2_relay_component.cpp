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

#include "shm_sensor_transport/shm_point_cloud2_relay_component.hpp"

#include <chrono>
#include <stdexcept>

#include "shm_sensor_transport/qos_utils.hpp"
#include "shm_sensor_transport/shm_name.hpp"

namespace shm_sensor_transport
{

ShmPointCloud2RelayComponent::ShmPointCloud2RelayComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("shm_point_cloud2_relay", options)
{
  params_ = declare_relay_parameters(*this, "points");
  const auto qos = make_metadata_qos(
    params_.use_sensor_data_qos, params_.reliable, params_.depth);

  metadata_publisher_ =
    create_publisher<shm_sensor_transport_interfaces::msg::ShmPointCloud2>(params_.meta_topic, qos);
  if (params_.publish_status && !params_.status_topic.empty()) {
    status_publisher_ =
      create_publisher<shm_sensor_transport_interfaces::msg::ShmTransportStatus>(
        params_.status_topic, rclcpp::QoS(1).reliable());
    if (params_.status_rate > 0.0) {
      const auto period = std::chrono::duration<double>(1.0 / params_.status_rate);
      status_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() {publish_status(ring_.is_open() ? "ok" : "waiting for first frame");});
    }
  }

  subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    params_.input_topic,
    qos,
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {cloud_callback(msg);});
}

ShmPointCloud2RelayComponent::~ShmPointCloud2RelayComponent()
{
  // The relay owns the shared-memory object; readers only map it while metadata is active.
  ring_.unlink();
  ring_.close();
}

void ShmPointCloud2RelayComponent::cloud_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  const auto payload_size = static_cast<std::uint64_t>(msg->data.size());
  if (!ensure_buffer(payload_size)) {
    ++dropped_frames_;
    return;
  }

  const auto write = ring_.write(msg->data.data(), msg->data.size());
  if (!write.success) {
    ++dropped_frames_;
    ++oversized_frames_;
    if (params_.warn_on_oversized_frame) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", write.message.c_str());
    }
    return;
  }

  shm_sensor_transport_interfaces::msg::ShmPointCloud2 meta;
  meta.header = msg->header;
  meta.shm_name = ring_.name();
  meta.slot_index = write.slot_index;
  meta.sequence = write.sequence;
  meta.slot_offset = write.payload_offset;
  meta.slot_size = write.slot_size;
  meta.payload_offset = write.payload_offset;
  meta.payload_size = write.payload_size;
  meta.height = msg->height;
  meta.width = msg->width;
  meta.fields = msg->fields;
  meta.is_bigendian = msg->is_bigendian != 0U;
  meta.point_step = msg->point_step;
  meta.row_step = msg->row_step;
  meta.is_dense = msg->is_dense;
  metadata_publisher_->publish(meta);
  ++published_frames_;
}

bool ShmPointCloud2RelayComponent::ensure_buffer(const std::uint64_t payload_size)
{
  if (ring_.is_open() && payload_size <= ring_.slot_size()) {
    return true;
  }
  if (ring_.is_open() && !params_.allow_resize) {
    ++oversized_frames_;
    if (params_.warn_on_oversized_frame) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "PointCloud2 payload is %zu bytes, larger than shared-memory slot size %zu",
        static_cast<std::size_t>(payload_size), static_cast<std::size_t>(ring_.slot_size()));
    }
    return false;
  }

  const auto slot_size = params_.slot_size_bytes == 0U ? payload_size : params_.slot_size_bytes;
  if (slot_size == 0U || payload_size > slot_size) {
    ++oversized_frames_;
    return false;
  }

  const auto shm_name = params_.shm_name.empty() ?
    make_shared_memory_name(params_.input_topic) :
    normalize_shared_memory_name(params_.shm_name);
  try {
    ring_.create(shm_name, static_cast<std::uint32_t>(params_.slot_count), slot_size);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Failed to create shared memory: %s", error.what());
    return false;
  }
  RCLCPP_INFO(
    get_logger(), "Created shared-memory ring %s with %d slots of %zu bytes",
    ring_.name().c_str(), params_.slot_count, static_cast<std::size_t>(slot_size));
  return true;
}

void ShmPointCloud2RelayComponent::publish_status(const std::string & message)
{
  if (!status_publisher_) {
    return;
  }
  shm_sensor_transport_interfaces::msg::ShmTransportStatus status;
  status.header.stamp = now();
  status.shm_name = ring_.name();
  status.input_topic = params_.input_topic;
  status.meta_topic = params_.meta_topic;
  status.slot_count = ring_.slot_count();
  status.slot_size = ring_.slot_size();
  status.total_size = ring_.total_size();
  status.published_frames = published_frames_;
  status.dropped_frames = dropped_frames_;
  status.oversized_frames = oversized_frames_;
  status.initialized = ring_.is_open();
  status.message = message;
  status_publisher_->publish(status);
}

}  // namespace shm_sensor_transport
