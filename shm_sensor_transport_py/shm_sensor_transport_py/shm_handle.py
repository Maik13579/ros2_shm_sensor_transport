# Copyright 2026 Maik Knof
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import mmap
import os
import struct
from typing import Optional

from shm_sensor_transport_py.metadata import (
    SHM_HEADER_SIZE,
    SHM_LAYOUT_VERSION,
    SHM_MAGIC,
    SLOT_HEADER_SIZE,
    SharedMemoryLayout,
    SlotSnapshot,
)


class ShmFrameInvalid(RuntimeError):
    """Raised when metadata points at a payload that cannot be copied safely."""


def shared_memory_path(shm_name: str) -> str:
    name = shm_name if shm_name.startswith("/") else f"/{shm_name}"
    return f"/dev/shm/{name[1:]}"


class ShmHandle:
    """Cached mmap handle for one shared-memory sensor transport region."""

    _HEADER = struct.Struct("<QIIIIQQQ")
    _SLOT = struct.Struct("<QQQQ")

    def __init__(self) -> None:
        self._name: Optional[str] = None
        self._fd: Optional[int] = None
        self._mmap: Optional[mmap.mmap] = None
        self._layout: Optional[SharedMemoryLayout] = None

    @property
    def name(self) -> Optional[str]:
        return self._name

    @property
    def layout(self) -> Optional[SharedMemoryLayout]:
        return self._layout

    def close(self) -> None:
        if self._mmap is not None:
            self._mmap.close()
            self._mmap = None
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None
        self._name = None
        self._layout = None

    def open(self, shm_name: str) -> None:
        normalized = shm_name if shm_name.startswith("/") else f"/{shm_name}"
        if self._name == normalized and self._mmap is not None:
            return

        self.close()
        path = shared_memory_path(normalized)
        fd = os.open(path, os.O_RDONLY)
        size = os.fstat(fd).st_size
        mapped = mmap.mmap(fd, size, access=mmap.ACCESS_READ)
        layout = self._read_layout(mapped)

        self._name = normalized
        self._fd = fd
        self._mmap = mapped
        self._layout = layout

    def copy_payload(self, meta) -> bytes:
        self.open(meta.shm_name)
        assert self._mmap is not None
        assert self._layout is not None

        slot_index = int(meta.slot_index)
        payload_size = int(meta.payload_size)
        payload_offset = int(meta.payload_offset)
        if slot_index >= self._layout.slot_count:
            raise ShmFrameInvalid(f"slot index {slot_index} exceeds shared-memory layout")
        if payload_size > self._layout.slot_size:
            raise ShmFrameInvalid("payload size exceeds slot size")
        if payload_offset + payload_size > len(self._mmap):
            raise ShmFrameInvalid("payload extends past shared-memory mapping")

        slot_offset = SHM_HEADER_SIZE + (slot_index * SLOT_HEADER_SIZE)
        # The writer flips the slot sequence odd while writing and even when complete.
        # Accepting only identical even values prevents torn payload copies.
        before = self._read_slot(slot_offset)
        if before.sequence != int(meta.sequence) or before.sequence % 2 != 0:
            raise ShmFrameInvalid("slot is being written or metadata is stale")

        data = bytes(self._mmap[payload_offset : payload_offset + payload_size])

        after = self._read_slot(slot_offset)
        if before.sequence != after.sequence or after.sequence % 2 != 0:
            raise ShmFrameInvalid("slot changed while payload was copied")
        if after.payload_size != payload_size:
            raise ShmFrameInvalid("slot payload size changed while payload was copied")
        return data

    def _read_layout(self, mapped: mmap.mmap) -> SharedMemoryLayout:
        if len(mapped) < SHM_HEADER_SIZE:
            raise ShmFrameInvalid("shared-memory object is smaller than the layout header")
        # Keep this struct format in sync with shm_sensor_transport::SharedMemoryHeader.
        values = self._HEADER.unpack_from(mapped, 0)
        layout = SharedMemoryLayout(
            magic=values[0],
            version=values[1],
            header_size=values[2],
            slot_count=values[3],
            slot_size=values[5],
            payload_base_offset=values[6],
            generation=values[7],
        )
        if layout.magic != SHM_MAGIC:
            raise ShmFrameInvalid("shared-memory object has an unexpected magic value")
        if layout.version != SHM_LAYOUT_VERSION:
            raise ShmFrameInvalid("shared-memory object has an unsupported layout version")
        if layout.header_size != SHM_HEADER_SIZE:
            raise ShmFrameInvalid("shared-memory object has an unexpected header size")
        return layout

    def _read_slot(self, offset: int) -> SlotSnapshot:
        assert self._mmap is not None
        values = self._SLOT.unpack_from(self._mmap, offset)
        return SlotSnapshot(sequence=values[0], payload_size=values[1])
