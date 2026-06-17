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

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include <array>
#include <cstring>
#include <string>

#include "shm_sensor_transport/shm_layout.hpp"
#include "shm_sensor_transport/shm_name.hpp"
#include "shm_sensor_transport/shm_ring_buffer.hpp"

namespace
{

std::string test_name()
{
  return "/ros2_shm_test_ring_" + std::to_string(::getpid());
}

}  // namespace

TEST(ShmRingBuffer, WritesPayloadAndSequence)
{
  const auto name = test_name();
  const auto normalized_name = shm_sensor_transport::normalize_shared_memory_name(name);
  shm_sensor_transport::ShmRingBuffer ring;
  ring.create(name, 2, 16);

  const std::array<std::uint8_t, 4> payload{1, 2, 3, 4};
  const auto result = ring.write(payload.data(), payload.size());
  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.slot_index, 0U);
  EXPECT_EQ(result.sequence % 2U, 0U);
  EXPECT_EQ(result.payload_size, payload.size());

  const int fd = ::shm_open(normalized_name.c_str(), O_RDONLY, 0);
  ASSERT_GE(fd, 0);
  const auto * mapping = ::mmap(nullptr, ring.total_size(), PROT_READ, MAP_SHARED, fd, 0);
  ASSERT_NE(mapping, MAP_FAILED);

  const auto * header = static_cast<const shm_sensor_transport::SharedMemoryHeader *>(mapping);
  EXPECT_EQ(header->magic, shm_sensor_transport::kShmMagic);
  EXPECT_EQ(header->slot_count, 2U);
  EXPECT_EQ(header->slot_size, 16U);

  const auto * bytes = static_cast<const std::uint8_t *>(mapping);
  const auto * slot_header = reinterpret_cast<const shm_sensor_transport::SlotHeader *>(
    bytes + sizeof(shm_sensor_transport::SharedMemoryHeader));
  EXPECT_EQ(slot_header->sequence, result.sequence);
  EXPECT_EQ(slot_header->payload_size, payload.size());
  EXPECT_EQ(std::memcmp(bytes + result.payload_offset, payload.data(), payload.size()), 0);

  ::munmap(const_cast<void *>(mapping), ring.total_size());
  ::close(fd);
  ring.unlink();
}

TEST(ShmRingBuffer, RejectsOversizedPayload)
{
  const auto name = test_name() + "_oversized";
  shm_sensor_transport::ShmRingBuffer ring;
  ring.create(name, 1, 2);

  const std::array<std::uint8_t, 3> payload{1, 2, 3};
  const auto result = ring.write(payload.data(), payload.size());
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("larger"), std::string::npos);
  ring.unlink();
}
