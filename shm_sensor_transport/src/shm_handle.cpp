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

#include "shm_sensor_transport/shm_handle.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <system_error>
#include <utility>

#include "shm_sensor_transport/shm_name.hpp"

namespace shm_sensor_transport
{

namespace
{

void throw_errno(const std::string & action)
{
  throw std::system_error(errno, std::generic_category(), action);
}

std::uint64_t payload_base_offset(const std::uint32_t slot_count)
{
  return sizeof(SharedMemoryHeader) + (static_cast<std::uint64_t>(slot_count) * sizeof(SlotHeader));
}

}  // namespace

ShmFrameInvalid::ShmFrameInvalid(const std::string & message)
: std::runtime_error(message)
{
}

ShmHandle::~ShmHandle()
{
  close();
}

ShmHandle::ShmHandle(ShmHandle && other) noexcept
{
  move_from(std::move(other));
}

ShmHandle & ShmHandle::operator=(ShmHandle && other) noexcept
{
  if (this != &other) {
    close();
    move_from(std::move(other));
  }
  return *this;
}

void ShmHandle::open(const std::string & shm_name)
{
  const auto normalized_name = normalize_shared_memory_name(shm_name);
  if (shm_name_ == normalized_name && mapping_ != nullptr) {
    return;
  }

  close();
  fd_ = ::shm_open(normalized_name.c_str(), O_RDONLY, 0);
  if (fd_ < 0) {
    throw_errno("shm_open " + normalized_name);
  }

  struct stat info {};
  if (::fstat(fd_, &info) != 0) {
    const auto saved_errno = errno;
    close();
    errno = saved_errno;
    throw_errno("fstat " + normalized_name);
  }
  if (info.st_size < static_cast<off_t>(sizeof(SharedMemoryHeader))) {
    close();
    throw ShmFrameInvalid("shared-memory object is smaller than the layout header");
  }

  total_size_ = static_cast<std::uint64_t>(info.st_size);
  mapping_ = ::mmap(nullptr, total_size_, PROT_READ, MAP_SHARED, fd_, 0);
  if (mapping_ == MAP_FAILED) {
    mapping_ = nullptr;
    const auto saved_errno = errno;
    close();
    errno = saved_errno;
    throw_errno("mmap " + normalized_name);
  }

  shm_name_ = normalized_name;
  layout_ = *static_cast<const SharedMemoryHeader *>(mapping_);
  validate_layout();
}

void ShmHandle::close()
{
  if (mapping_ != nullptr) {
    ::munmap(mapping_, total_size_);
    mapping_ = nullptr;
  }
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  shm_name_.clear();
  total_size_ = 0;
  layout_ = {};
}

const std::string & ShmHandle::name() const noexcept
{
  return shm_name_;
}

bool ShmHandle::is_open() const noexcept
{
  return mapping_ != nullptr;
}

std::vector<std::uint8_t> ShmHandle::copy_payload(
  const std::string & shm_name,
  const std::uint32_t slot_index,
  const std::uint64_t sequence,
  const std::uint64_t payload_offset,
  const std::uint64_t payload_size)
{
  open(shm_name);

  if (slot_index >= layout_.slot_count) {
    throw ShmFrameInvalid("slot index exceeds shared-memory layout");
  }
  if (payload_size > layout_.slot_size) {
    throw ShmFrameInvalid("payload size exceeds slot size");
  }
  if (payload_offset > total_size_ || payload_size > total_size_ - payload_offset) {
    throw ShmFrameInvalid("payload extends past shared-memory mapping");
  }

  const auto & slot = slot_header(slot_index);
  const auto before_sequence = __atomic_load_n(&slot.sequence, __ATOMIC_ACQUIRE);
  const auto before_payload_size = __atomic_load_n(&slot.payload_size, __ATOMIC_ACQUIRE);
  if (before_sequence != sequence || (before_sequence % 2U) != 0U) {
    throw ShmFrameInvalid("slot is being written or metadata is stale");
  }

  const auto * bytes = static_cast<const std::uint8_t *>(mapping_);
  const auto * payload_begin = bytes + payload_offset;
  std::vector<std::uint8_t> payload(payload_begin, payload_begin + payload_size);

  const auto after_sequence = __atomic_load_n(&slot.sequence, __ATOMIC_ACQUIRE);
  const auto after_payload_size = __atomic_load_n(&slot.payload_size, __ATOMIC_ACQUIRE);
  if (before_sequence != after_sequence || (after_sequence % 2U) != 0U) {
    throw ShmFrameInvalid("slot changed while payload was copied");
  }
  if (before_payload_size != after_payload_size || after_payload_size != payload_size) {
    throw ShmFrameInvalid("slot payload size changed while payload was copied");
  }
  return payload;
}

const SlotHeader & ShmHandle::slot_header(const std::uint32_t slot_index) const
{
  const auto * bytes = static_cast<const std::uint8_t *>(mapping_);
  return *reinterpret_cast<const SlotHeader *>(
    bytes + sizeof(SharedMemoryHeader) + (slot_index * sizeof(SlotHeader)));
}

void ShmHandle::validate_layout() const
{
  if (layout_.magic != kShmMagic) {
    throw ShmFrameInvalid("shared-memory object has an unexpected magic value");
  }
  if (layout_.version != kShmLayoutVersion) {
    throw ShmFrameInvalid("shared-memory object has an unsupported layout version");
  }
  if (layout_.header_size != sizeof(SharedMemoryHeader)) {
    throw ShmFrameInvalid("shared-memory object has an unexpected header size");
  }
  if (layout_.payload_base_offset != payload_base_offset(layout_.slot_count)) {
    throw ShmFrameInvalid("shared-memory object has an unexpected payload offset");
  }
  if (layout_.payload_base_offset > total_size_) {
    throw ShmFrameInvalid("shared-memory payload base extends past mapping");
  }
}

void ShmHandle::move_from(ShmHandle && other) noexcept
{
  shm_name_ = std::move(other.shm_name_);
  fd_ = other.fd_;
  mapping_ = other.mapping_;
  total_size_ = other.total_size_;
  layout_ = other.layout_;

  other.fd_ = -1;
  other.mapping_ = nullptr;
  other.total_size_ = 0;
  other.layout_ = {};
}

}  // namespace shm_sensor_transport
