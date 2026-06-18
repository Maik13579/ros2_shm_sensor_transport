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

#include <unistd.h>

#include <gtest/gtest.h>

#include <array>
#include <string>

#include <shm_sensor_transport_interfaces/msg/shm_image.hpp>

#include "shm_sensor_transport/shm_handle.hpp"
#include "shm_sensor_transport/shm_ring_buffer.hpp"

namespace
{

std::string test_name(const std::string & suffix)
{
  return "/ros2_shm_test_handle_" + std::to_string(::getpid()) + suffix;
}

shm_sensor_transport_interfaces::msg::ShmImage make_meta(
  const shm_sensor_transport::ShmRingBuffer & ring,
  const shm_sensor_transport::WriteResult & write)
{
  shm_sensor_transport_interfaces::msg::ShmImage meta;
  meta.shm_name = ring.name();
  meta.slot_index = write.slot_index;
  meta.sequence = write.sequence;
  meta.slot_offset = write.payload_offset;
  meta.slot_size = write.slot_size;
  meta.payload_offset = write.payload_offset;
  meta.payload_size = write.payload_size;
  return meta;
}

}  // namespace

TEST(ShmHandle, CopiesPayloadFromMetadata)
{
  shm_sensor_transport::ShmRingBuffer ring;
  ring.create(test_name("_copy"), 2, 16);

  const std::array<std::uint8_t, 4> payload{1, 2, 3, 4};
  const auto write = ring.write(payload.data(), payload.size());
  ASSERT_TRUE(write.success);

  shm_sensor_transport::ShmHandle handle;
  const auto copied = handle.copy_payload(make_meta(ring, write));

  EXPECT_EQ(copied, std::vector<std::uint8_t>(payload.begin(), payload.end()));
  EXPECT_EQ(handle.name(), ring.name());
  ring.unlink();
}

TEST(ShmHandle, RejectsStaleMetadata)
{
  shm_sensor_transport::ShmRingBuffer ring;
  ring.create(test_name("_stale"), 1, 16);

  const std::array<std::uint8_t, 4> payload{1, 2, 3, 4};
  const auto write = ring.write(payload.data(), payload.size());
  ASSERT_TRUE(write.success);

  auto meta = make_meta(ring, write);
  meta.sequence += 2U;

  shm_sensor_transport::ShmHandle handle;
  EXPECT_THROW(handle.copy_payload(meta), shm_sensor_transport::ShmFrameInvalid);
  ring.unlink();
}
