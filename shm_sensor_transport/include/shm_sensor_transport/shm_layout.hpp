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

namespace shm_sensor_transport
{

/// @brief Magic value used to identify shm_sensor_transport memory regions.
constexpr std::uint64_t kShmMagic = 0x53544d48534f5232ULL;

/// @brief Version of the shared-memory binary layout.
constexpr std::uint32_t kShmLayoutVersion = 1U;

/// @brief Fixed header stored at the start of every shared-memory region.
struct SharedMemoryHeader
{
  std::uint64_t magic;
  std::uint32_t version;
  std::uint32_t header_size;
  std::uint32_t slot_count;
  std::uint32_t reserved;
  std::uint64_t slot_size;
  std::uint64_t payload_base_offset;
  std::uint64_t generation;
};

/// @brief Per-slot metadata used for lock-free writer/reader validation.
struct SlotHeader
{
  std::uint64_t sequence;
  std::uint64_t payload_size;
  std::uint64_t reserved0;
  std::uint64_t reserved1;
};

static_assert(sizeof(SharedMemoryHeader) == 48U);
static_assert(sizeof(SlotHeader) == 32U);

}  // namespace shm_sensor_transport
