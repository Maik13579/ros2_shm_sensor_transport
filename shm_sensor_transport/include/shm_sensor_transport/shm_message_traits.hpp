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

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <shm_sensor_transport_interfaces/msg/shm_compressed_image.hpp>
#include <shm_sensor_transport_interfaces/msg/shm_image.hpp>
#include <shm_sensor_transport_interfaces/msg/shm_point_cloud2.hpp>

#include "shm_sensor_transport/shm_ring_buffer.hpp"

namespace shm_sensor_transport
{

/// @brief Return the hidden shared-memory metadata topic for a normal sensor topic.
std::string resolve_metadata_topic(const std::string & topic);

/// @brief Metadata encode/decode traits for supported normal sensor message types.
template<typename SensorMessageT>
struct ShmMessageTraits;

template<>
struct ShmMessageTraits<sensor_msgs::msg::CompressedImage>
{
  using MetadataMessage = shm_sensor_transport_interfaces::msg::ShmCompressedImage;

  static const std::vector<std::uint8_t> & payload(
    const sensor_msgs::msg::CompressedImage & msg)
  {
    return msg.data;
  }

  static MetadataMessage encode(
    const sensor_msgs::msg::CompressedImage & msg,
    const ShmRingBuffer & ring,
    const WriteResult & write)
  {
    MetadataMessage meta;
    meta.header = msg.header;
    meta.shm_name = ring.name();
    meta.slot_index = write.slot_index;
    meta.sequence = write.sequence;
    meta.slot_offset = write.payload_offset;
    meta.slot_size = write.slot_size;
    meta.payload_offset = write.payload_offset;
    meta.payload_size = write.payload_size;
    meta.format = msg.format;
    return meta;
  }

  static std::unique_ptr<sensor_msgs::msg::CompressedImage> decode(
    std::vector<std::uint8_t> payload,
    const MetadataMessage & meta)
  {
    auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
    msg->header = meta.header;
    msg->format = meta.format;
    msg->data = std::move(payload);
    return msg;
  }
};

template<>
struct ShmMessageTraits<sensor_msgs::msg::Image>
{
  using MetadataMessage = shm_sensor_transport_interfaces::msg::ShmImage;

  static const std::vector<std::uint8_t> & payload(const sensor_msgs::msg::Image & msg)
  {
    return msg.data;
  }

  static MetadataMessage encode(
    const sensor_msgs::msg::Image & msg,
    const ShmRingBuffer & ring,
    const WriteResult & write)
  {
    MetadataMessage meta;
    meta.header = msg.header;
    meta.shm_name = ring.name();
    meta.slot_index = write.slot_index;
    meta.sequence = write.sequence;
    meta.slot_offset = write.payload_offset;
    meta.slot_size = write.slot_size;
    meta.payload_offset = write.payload_offset;
    meta.payload_size = write.payload_size;
    meta.height = msg.height;
    meta.width = msg.width;
    meta.encoding = msg.encoding;
    meta.step = msg.step;
    meta.is_bigendian = msg.is_bigendian != 0U;
    return meta;
  }

  static std::unique_ptr<sensor_msgs::msg::Image> decode(
    std::vector<std::uint8_t> payload,
    const MetadataMessage & meta)
  {
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header = meta.header;
    msg->height = meta.height;
    msg->width = meta.width;
    msg->encoding = meta.encoding;
    msg->is_bigendian = meta.is_bigendian ? 1U : 0U;
    msg->step = meta.step;
    msg->data = std::move(payload);
    return msg;
  }
};

template<>
struct ShmMessageTraits<sensor_msgs::msg::PointCloud2>
{
  using MetadataMessage = shm_sensor_transport_interfaces::msg::ShmPointCloud2;

  static const std::vector<std::uint8_t> & payload(const sensor_msgs::msg::PointCloud2 & msg)
  {
    return msg.data;
  }

  static MetadataMessage encode(
    const sensor_msgs::msg::PointCloud2 & msg,
    const ShmRingBuffer & ring,
    const WriteResult & write)
  {
    MetadataMessage meta;
    meta.header = msg.header;
    meta.shm_name = ring.name();
    meta.slot_index = write.slot_index;
    meta.sequence = write.sequence;
    meta.slot_offset = write.payload_offset;
    meta.slot_size = write.slot_size;
    meta.payload_offset = write.payload_offset;
    meta.payload_size = write.payload_size;
    meta.height = msg.height;
    meta.width = msg.width;
    meta.fields = msg.fields;
    meta.is_bigendian = msg.is_bigendian != 0U;
    meta.point_step = msg.point_step;
    meta.row_step = msg.row_step;
    meta.is_dense = msg.is_dense;
    return meta;
  }

  static std::unique_ptr<sensor_msgs::msg::PointCloud2> decode(
    std::vector<std::uint8_t> payload,
    const MetadataMessage & meta)
  {
    auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    msg->header = meta.header;
    msg->height = meta.height;
    msg->width = meta.width;
    msg->fields = meta.fields;
    msg->is_bigendian = meta.is_bigendian;
    msg->point_step = meta.point_step;
    msg->row_step = meta.row_step;
    msg->is_dense = meta.is_dense;
    msg->data = std::move(payload);
    return msg;
  }
};

}  // namespace shm_sensor_transport
