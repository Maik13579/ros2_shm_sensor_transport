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

#include "shm_sensor_transport/shm_ring_buffer.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>
#include <system_error>
#include <utility>

#include "shm_sensor_transport/shm_name.hpp"

namespace shm_sensor_transport
{

namespace
{

std::uint64_t payload_base_offset(const std::uint32_t slot_count)
{
  return sizeof(SharedMemoryHeader) + (static_cast<std::uint64_t>(slot_count) * sizeof(SlotHeader));
}

void throw_errno(const std::string & action)
{
  throw std::system_error(errno, std::generic_category(), action);
}

}  // namespace

ShmRingBuffer::~ShmRingBuffer()
{
  close();
}

ShmRingBuffer::ShmRingBuffer(ShmRingBuffer && other) noexcept
{
  move_from(std::move(other));
}

ShmRingBuffer & ShmRingBuffer::operator=(ShmRingBuffer && other) noexcept
{
  if (this != &other) {
    close();
    move_from(std::move(other));
  }
  return *this;
}

void ShmRingBuffer::create(
  const std::string & shm_name,
  const std::uint32_t slot_count,
  const std::uint64_t slot_size)
{
  if (slot_count == 0U || slot_size == 0U) {
    throw std::invalid_argument("slot_count and slot_size must be non-zero");
  }

  close();
  shm_name_ = normalize_shared_memory_name(shm_name);
  slot_count_ = slot_count;
  slot_size_ = slot_size;
  next_slot_ = 0U;
  next_sequence_ = 2U;
  total_size_ = payload_base_offset(slot_count_) +
    (static_cast<std::uint64_t>(slot_count_) * slot_size_);

  ::shm_unlink(shm_name_.c_str());
  fd_ = ::shm_open(shm_name_.c_str(), O_CREAT | O_EXCL | O_RDWR, 0600);
  if (fd_ < 0) {
    throw_errno("shm_open " + shm_name_);
  }
  if (::ftruncate(fd_, static_cast<off_t>(total_size_)) != 0) {
    const auto saved_errno = errno;
    close();
    errno = saved_errno;
    throw_errno("ftruncate " + shm_name_);
  }
  mapping_ = ::mmap(nullptr, total_size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
  if (mapping_ == MAP_FAILED) {
    mapping_ = nullptr;
    const auto saved_errno = errno;
    close();
    errno = saved_errno;
    throw_errno("mmap " + shm_name_);
  }

  std::memset(mapping_, 0, static_cast<std::size_t>(total_size_));
  auto * header = static_cast<SharedMemoryHeader *>(mapping_);
  header->magic = kShmMagic;
  header->version = kShmLayoutVersion;
  header->header_size = sizeof(SharedMemoryHeader);
  header->slot_count = slot_count_;
  header->slot_size = slot_size_;
  header->payload_base_offset = payload_base_offset(slot_count_);
  header->generation = next_sequence_;
}

void ShmRingBuffer::close()
{
  if (mapping_ != nullptr) {
    ::munmap(mapping_, total_size_);
    mapping_ = nullptr;
  }
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void ShmRingBuffer::unlink()
{
  if (!shm_name_.empty()) {
    ::shm_unlink(shm_name_.c_str());
  }
}

WriteResult ShmRingBuffer::write(const std::uint8_t * data, const std::size_t size)
{
  WriteResult result;
  if (!is_open()) {
    result.message = "shared memory is not open";
    return result;
  }
  if (size > slot_size_) {
    result.message = "payload is larger than the configured slot size";
    return result;
  }

  const auto slot = next_slot_;
  auto * header = slot_header(slot);
  const auto even_sequence = next_sequence_;
  const auto odd_sequence = even_sequence | 1ULL;

  // Readers only accept matching even sequence values before and after copying.
  // Publishing an odd value marks the slot as actively being written.
  __atomic_store_n(&header->sequence, odd_sequence, __ATOMIC_RELEASE);
  std::memcpy(payload(slot), data, size);
  header->payload_size = static_cast<std::uint64_t>(size);
  __atomic_store_n(&header->sequence, even_sequence, __ATOMIC_RELEASE);

  result.success = true;
  result.slot_index = slot;
  result.sequence = even_sequence;
  result.payload_offset = payload_base_offset(slot_count_) +
    (static_cast<std::uint64_t>(slot) * slot_size_);
  result.slot_size = slot_size_;
  result.payload_size = static_cast<std::uint64_t>(size);

  next_slot_ = (next_slot_ + 1U) % slot_count_;
  next_sequence_ += 2U;
  return result;
}

bool ShmRingBuffer::is_open() const noexcept
{
  return mapping_ != nullptr;
}

const std::string & ShmRingBuffer::name() const noexcept
{
  return shm_name_;
}

std::uint32_t ShmRingBuffer::slot_count() const noexcept
{
  return slot_count_;
}

std::uint64_t ShmRingBuffer::slot_size() const noexcept
{
  return slot_size_;
}

std::uint64_t ShmRingBuffer::total_size() const noexcept
{
  return total_size_;
}

SlotHeader * ShmRingBuffer::slot_header(const std::uint32_t slot_index) const
{
  auto * bytes = static_cast<std::uint8_t *>(mapping_);
  return reinterpret_cast<SlotHeader *>(
    bytes + sizeof(SharedMemoryHeader) + (slot_index * sizeof(SlotHeader)));
}

std::uint8_t * ShmRingBuffer::payload(const std::uint32_t slot_index) const
{
  auto * bytes = static_cast<std::uint8_t *>(mapping_);
  return bytes + payload_base_offset(slot_count_) +
         (static_cast<std::uint64_t>(slot_index) * slot_size_);
}

void ShmRingBuffer::move_from(ShmRingBuffer && other) noexcept
{
  shm_name_ = std::move(other.shm_name_);
  fd_ = other.fd_;
  mapping_ = other.mapping_;
  total_size_ = other.total_size_;
  slot_count_ = other.slot_count_;
  slot_size_ = other.slot_size_;
  next_slot_ = other.next_slot_;
  next_sequence_ = other.next_sequence_;

  other.fd_ = -1;
  other.mapping_ = nullptr;
  other.total_size_ = 0;
  other.slot_count_ = 0;
  other.slot_size_ = 0;
  other.next_slot_ = 0;
  other.next_sequence_ = 2;
}

}  // namespace shm_sensor_transport
