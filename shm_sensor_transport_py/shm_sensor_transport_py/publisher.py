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

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from shm_sensor_transport_interfaces.msg import ShmImage, ShmPointCloud2

from shm_sensor_transport_py.metadata import (
    SHM_HEADER_SIZE,
    SHM_LAYOUT_VERSION,
    SHM_MAGIC,
    SLOT_HEADER_SIZE,
)
from shm_sensor_transport_py.shm_handle import shared_memory_path
from shm_sensor_transport_py.subscriber import resolve_metadata_topic


def _normalize_shared_memory_name(name: str) -> str:
    raw = name or "/ros2_shm_topic"
    normalized = "/" if not raw.startswith("/") else ""
    for index, char in enumerate(raw):
        if index == 0 and char == "/":
            normalized += "/"
        elif char.isalnum():
            normalized += char
        else:
            normalized += "_"
    while len(normalized) > 1 and normalized.endswith("_"):
        normalized = normalized[:-1]
    return normalized


def _make_shared_memory_name(topic: str) -> str:
    stem = "".join(char if char.isalnum() else "_" for char in topic).lstrip("_")
    if not stem:
        stem = "topic"
    return _normalize_shared_memory_name(f"/ros2_shm_{stem}_{abs(hash(topic)):x}")


class _RingBuffer:
    _HEADER = struct.Struct("<QIIIIQQQ")
    _SLOT = struct.Struct("<QQQQ")

    def __init__(self) -> None:
        self.name: str = ""
        self.slot_count: int = 0
        self.slot_size: int = 0
        self.total_size: int = 0
        self._fd: Optional[int] = None
        self._mmap: Optional[mmap.mmap] = None
        self._next_slot = 0
        self._next_sequence = 2

    @property
    def is_open(self) -> bool:
        return self._mmap is not None

    def create(self, shm_name: str, slot_count: int, slot_size: int) -> None:
        if slot_count <= 0 or slot_size <= 0:
            raise ValueError("slot_count and slot_size must be non-zero")

        self.close()
        self.name = _normalize_shared_memory_name(shm_name)
        self.slot_count = int(slot_count)
        self.slot_size = int(slot_size)
        self.total_size = SHM_HEADER_SIZE + (self.slot_count * SLOT_HEADER_SIZE) + (
            self.slot_count * self.slot_size
        )
        self._next_slot = 0
        self._next_sequence = 2

        path = shared_memory_path(self.name)
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass
        self._fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_RDWR, 0o600)
        os.ftruncate(self._fd, self.total_size)
        self._mmap = mmap.mmap(self._fd, self.total_size, access=mmap.ACCESS_WRITE)
        self._mmap[:] = b"\x00" * self.total_size
        self._HEADER.pack_into(
            self._mmap,
            0,
            SHM_MAGIC,
            SHM_LAYOUT_VERSION,
            SHM_HEADER_SIZE,
            self.slot_count,
            0,
            self.slot_size,
            self._payload_base_offset,
            self._next_sequence,
        )

    def close(self) -> None:
        if self._mmap is not None:
            self._mmap.close()
            self._mmap = None
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None

    def unlink(self) -> None:
        if not self.name:
            return
        try:
            os.unlink(shared_memory_path(self.name))
        except FileNotFoundError:
            pass

    def write(self, payload: bytes) -> tuple[int, int, int, int, int]:
        if self._mmap is None:
            raise RuntimeError("shared memory is not open")
        if len(payload) > self.slot_size:
            raise ValueError("payload is larger than the configured slot size")

        slot = self._next_slot
        sequence = self._next_sequence
        slot_offset = SHM_HEADER_SIZE + (slot * SLOT_HEADER_SIZE)
        payload_offset = self._payload_base_offset + (slot * self.slot_size)

        self._SLOT.pack_into(self._mmap, slot_offset, sequence | 1, len(payload), 0, 0)
        self._mmap[payload_offset : payload_offset + len(payload)] = payload
        self._SLOT.pack_into(self._mmap, slot_offset, sequence, len(payload), 0, 0)

        self._next_slot = (self._next_slot + 1) % self.slot_count
        self._next_sequence += 2
        return slot, sequence, payload_offset, self.slot_size, len(payload)

    @property
    def _payload_base_offset(self) -> int:
        return SHM_HEADER_SIZE + (self.slot_count * SLOT_HEADER_SIZE)


class ShmPublisher:
    """Publish Image or PointCloud2 messages through a shared-memory ring."""

    def __init__(
        self,
        node,
        topic: str,
        msg_type=None,
        slot_count: int = 8,
        slot_size_bytes: int = 0,
        allow_resize: bool = False,
        qos_profile: QoSProfile = None,
        shm_name: str = "",
    ) -> None:
        self._node = node
        self._topic = topic
        self._msg_type = msg_type
        self._slot_count = max(1, int(slot_count))
        self._slot_size_bytes = max(0, int(slot_size_bytes))
        self._allow_resize = bool(allow_resize)
        self._shm_name = shm_name
        self._metadata_topic = resolve_metadata_topic(topic)
        self._qos = qos_profile or QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._ring = _RingBuffer()
        self._publisher = None
        self._last_metadata = None

    @property
    def metadata_topic(self) -> str:
        return self._metadata_topic

    @property
    def publisher(self):
        return self._publisher

    @property
    def shm_name(self) -> str:
        return self._ring.name

    @property
    def last_metadata(self):
        return self._last_metadata

    def close(self) -> None:
        self._ring.unlink()
        self._ring.close()

    def publish(self, msg) -> bool:
        msg_type = self._resolve_msg_type(msg)
        metadata_type = ShmImage if msg_type is Image else ShmPointCloud2
        if self._publisher is None:
            self._publisher = self._node.create_publisher(
                metadata_type, self._metadata_topic, self._qos
            )

        payload = bytes(msg.data)
        self._ensure_buffer(len(payload))
        slot_index, sequence, payload_offset, slot_size, payload_size = self._ring.write(payload)
        meta = self._make_metadata(
            msg, slot_index, sequence, payload_offset, slot_size, payload_size
        )
        self._publisher.publish(meta)
        self._last_metadata = meta
        return True

    def _resolve_msg_type(self, msg):
        msg_type = self._msg_type or type(msg)
        if msg_type not in (Image, PointCloud2):
            raise TypeError("ShmPublisher only supports sensor_msgs.msg.Image and PointCloud2")
        if not isinstance(msg, msg_type):
            raise TypeError(f"expected {msg_type.__name__}, got {type(msg).__name__}")
        return msg_type

    def _ensure_buffer(self, payload_size: int) -> None:
        if self._ring.is_open and payload_size <= self._ring.slot_size:
            return
        if self._ring.is_open and not self._allow_resize:
            raise ValueError("payload is larger than the configured slot size")

        slot_size = payload_size if self._slot_size_bytes == 0 else self._slot_size_bytes
        if slot_size == 0 or payload_size > slot_size:
            raise ValueError("payload is larger than the configured slot size")
        shm_name = self._shm_name or _make_shared_memory_name(self._topic)
        self._ring.create(shm_name, self._slot_count, slot_size)

    def _make_metadata(
        self,
        msg,
        slot_index: int,
        sequence: int,
        payload_offset: int,
        slot_size: int,
        payload_size: int,
    ):
        if isinstance(msg, Image):
            meta = ShmImage()
            meta.encoding = msg.encoding
            meta.step = msg.step
        else:
            meta = ShmPointCloud2()
            meta.fields = msg.fields
            meta.point_step = msg.point_step
            meta.row_step = msg.row_step
            meta.is_dense = msg.is_dense

        meta.header = msg.header
        meta.shm_name = self._ring.name
        meta.slot_index = slot_index
        meta.sequence = sequence
        meta.slot_offset = payload_offset
        meta.slot_size = slot_size
        meta.payload_offset = payload_offset
        meta.payload_size = payload_size
        meta.height = msg.height
        meta.width = msg.width
        meta.is_bigendian = bool(msg.is_bigendian)
        return meta
