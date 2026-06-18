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
#include <stdexcept>
#include <string>
#include <vector>

#include "shm_sensor_transport/shm_layout.hpp"

namespace shm_sensor_transport
{

/// @brief Raised when metadata points at a payload that cannot be copied safely.
class ShmFrameInvalid : public std::runtime_error
{
public:
  explicit ShmFrameInvalid(const std::string & message);
};

/// @brief Cached read-only mmap handle for one shared-memory sensor transport region.
class ShmHandle
{
public:
  /// @brief Construct an unopened shared-memory handle.
  ShmHandle() = default;

  /// @brief Close the mapping and file descriptor.
  ~ShmHandle();

  ShmHandle(const ShmHandle &) = delete;
  ShmHandle & operator=(const ShmHandle &) = delete;

  /// @brief Move ownership of an open shared-memory mapping.
  ShmHandle(ShmHandle && other) noexcept;

  /// @brief Move ownership of an open shared-memory mapping.
  ShmHandle & operator=(ShmHandle && other) noexcept;

  /// @brief Open or reuse the named POSIX shared-memory object.
  void open(const std::string & shm_name);

  /// @brief Close this process' mapping and file descriptor.
  void close();

  /// @brief Copy payload bytes described by a ShmImage or ShmPointCloud2 metadata message.
  template<typename MetadataT>
  std::vector<std::uint8_t> copy_payload(const MetadataT & meta)
  {
    return copy_payload(
      meta.shm_name, meta.slot_index, meta.sequence, meta.payload_offset, meta.payload_size);
  }

  /// @brief Return the normalized POSIX shared-memory object name.
  const std::string & name() const noexcept;

  /// @brief Return true when this handle owns a valid memory mapping.
  bool is_open() const noexcept;

private:
  std::vector<std::uint8_t> copy_payload(
    const std::string & shm_name,
    std::uint32_t slot_index,
    std::uint64_t sequence,
    std::uint64_t payload_offset,
    std::uint64_t payload_size);
  const SlotHeader & slot_header(std::uint32_t slot_index) const;
  void validate_layout() const;
  void move_from(ShmHandle && other) noexcept;

  std::string shm_name_;
  int fd_{-1};
  void * mapping_{nullptr};
  std::uint64_t total_size_{0};
  SharedMemoryHeader layout_{};
};

}  // namespace shm_sensor_transport
