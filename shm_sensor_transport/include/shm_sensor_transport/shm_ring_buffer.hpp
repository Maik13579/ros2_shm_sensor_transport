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

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "shm_sensor_transport/shm_layout.hpp"

namespace shm_sensor_transport
{

/// @brief Result metadata for one payload written into a shared-memory slot.
struct WriteResult
{
  bool success{false};
  std::uint32_t slot_index{0};
  std::uint64_t sequence{0};
  std::uint64_t payload_offset{0};
  std::uint64_t slot_size{0};
  std::uint64_t payload_size{0};
  std::string message;
};

/// @brief Fixed-size POSIX shared-memory ring buffer for raw sensor payload bytes.
class ShmRingBuffer
{
public:
  /// @brief Construct an unopened ring buffer handle.
  ShmRingBuffer() = default;

  /// @brief Close the mapping and file descriptor without unlinking the object.
  ~ShmRingBuffer();

  ShmRingBuffer(const ShmRingBuffer &) = delete;
  ShmRingBuffer & operator=(const ShmRingBuffer &) = delete;

  /// @brief Move ownership of an open shared-memory mapping.
  ShmRingBuffer(ShmRingBuffer && other) noexcept;

  /// @brief Move ownership of an open shared-memory mapping.
  ShmRingBuffer & operator=(ShmRingBuffer && other) noexcept;

  /// @brief Create a new shared-memory object with fixed-size payload slots.
  void create(const std::string & shm_name, std::uint32_t slot_count, std::uint64_t slot_size);

  /// @brief Close this process' mapping and file descriptor.
  void close();

  /// @brief Unlink the named POSIX shared-memory object.
  void unlink();

  /// @brief Write payload bytes into the next slot and publish even sequence metadata.
  WriteResult write(const std::uint8_t * data, std::size_t size);

  /// @brief Return true when this handle owns a valid memory mapping.
  bool is_open() const noexcept;

  /// @brief Return the normalized POSIX shared-memory object name.
  const std::string & name() const noexcept;

  /// @brief Return the number of payload slots in the ring.
  std::uint32_t slot_count() const noexcept;

  /// @brief Return the fixed payload capacity of each slot in bytes.
  std::uint64_t slot_size() const noexcept;

  /// @brief Return the total mapped shared-memory size in bytes.
  std::uint64_t total_size() const noexcept;

private:
  SlotHeader * slot_header(std::uint32_t slot_index) const;
  std::uint8_t * payload(std::uint32_t slot_index) const;
  void move_from(ShmRingBuffer && other) noexcept;

  std::string shm_name_;
  int fd_{-1};
  void * mapping_{nullptr};
  std::uint64_t total_size_{0};
  std::uint32_t slot_count_{0};
  std::uint64_t slot_size_{0};
  std::uint32_t next_slot_{0};
  std::uint64_t next_sequence_{2};
};

}  // namespace shm_sensor_transport
