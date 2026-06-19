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

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include <sensor_msgs/msg/compressed_image.hpp>

#include "shm_sensor_transport/shm_subscriber.hpp"

TEST(ShmSubscriber, ResolvesMetadataTopic)
{
  EXPECT_EQ(
    shm_sensor_transport::resolve_metadata_topic("/camera/image_raw"),
    "/camera/image_raw/_shm");
  EXPECT_EQ(
    shm_sensor_transport::resolve_metadata_topic("/camera/image_raw/_shm"),
    "/camera/image_raw/_shm");
  EXPECT_EQ(
    shm_sensor_transport::resolve_metadata_topic("/camera/image_raw/"),
    "/camera/image_raw/_shm");
  EXPECT_THROW(shm_sensor_transport::resolve_metadata_topic("/"), std::invalid_argument);
}

TEST(ShmSubscriber, DecodesImageMetadataToRosMessage)
{
  shm_sensor_transport_interfaces::msg::ShmImage meta;
  meta.header.frame_id = "camera";
  meta.height = 2;
  meta.width = 3;
  meta.encoding = "mono8";
  meta.step = 3;
  meta.is_bigendian = false;

  std::vector<std::uint8_t> payload{1, 2, 3, 4, 5, 6};
  const auto msg = shm_sensor_transport::ShmMessageTraits<sensor_msgs::msg::Image>::decode(
    std::move(payload), meta);

  EXPECT_EQ(msg->header.frame_id, "camera");
  EXPECT_EQ(msg->height, 2U);
  EXPECT_EQ(msg->width, 3U);
  EXPECT_EQ(msg->encoding, "mono8");
  EXPECT_EQ(msg->step, 3U);
  EXPECT_EQ(msg->data, (std::vector<std::uint8_t>{1, 2, 3, 4, 5, 6}));
}

TEST(ShmSubscriber, DecodesCompressedImageMetadataToRosMessage)
{
  shm_sensor_transport_interfaces::msg::ShmCompressedImage meta;
  meta.header.frame_id = "camera";
  meta.format = "jpeg";

  std::vector<std::uint8_t> payload{0xff, 0xd8, 1, 2, 0xff, 0xd9};
  const auto msg =
    shm_sensor_transport::ShmMessageTraits<sensor_msgs::msg::CompressedImage>::decode(
    std::move(payload), meta);

  EXPECT_EQ(msg->header.frame_id, "camera");
  EXPECT_EQ(msg->format, "jpeg");
  EXPECT_EQ(msg->data, (std::vector<std::uint8_t>{0xff, 0xd8, 1, 2, 0xff, 0xd9}));
}

TEST(ShmSubscriber, DecodesPointCloud2MetadataToRosMessage)
{
  shm_sensor_transport_interfaces::msg::ShmPointCloud2 meta;
  meta.header.frame_id = "lidar";
  meta.height = 1;
  meta.width = 2;
  meta.point_step = 16;
  meta.row_step = 32;
  meta.is_dense = true;
  sensor_msgs::msg::PointField field;
  field.name = "x";
  field.offset = 0;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  meta.fields.push_back(field);

  std::vector<std::uint8_t> payload{0, 1, 2, 3};
  const auto msg = shm_sensor_transport::ShmMessageTraits<sensor_msgs::msg::PointCloud2>::decode(
    std::move(payload), meta);

  EXPECT_EQ(msg->header.frame_id, "lidar");
  EXPECT_EQ(msg->height, 1U);
  EXPECT_EQ(msg->width, 2U);
  EXPECT_EQ(msg->point_step, 16U);
  EXPECT_EQ(msg->row_step, 32U);
  ASSERT_EQ(msg->fields.size(), 1U);
  EXPECT_EQ(msg->fields.front().name, "x");
  EXPECT_EQ(msg->data, (std::vector<std::uint8_t>{0, 1, 2, 3}));
}
