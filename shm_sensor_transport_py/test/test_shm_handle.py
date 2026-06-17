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

from shm_sensor_transport_py.metadata import SHM_HEADER_SIZE, SHM_MAGIC, SHM_LAYOUT_VERSION
from shm_sensor_transport_py.shm_handle import ShmHandle


class Meta:
    shm_name = ""
    slot_index = 0
    sequence = 2
    payload_offset = SHM_HEADER_SIZE + 32
    payload_size = 4


def test_shm_handle_copies_valid_payload(tmp_path):
    shm_name = f"/ros2_shm_pytest_{os.getpid()}"
    path = f"/dev/shm/{shm_name[1:]}"
    size = SHM_HEADER_SIZE + 32 + 16
    fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_RDWR, 0o600)
    try:
        os.ftruncate(fd, size)
        with mmap.mmap(fd, size, access=mmap.ACCESS_WRITE) as mapped:
            mapped[:SHM_HEADER_SIZE] = struct.pack(
                "<QIIIIQQQ",
                SHM_MAGIC,
                SHM_LAYOUT_VERSION,
                SHM_HEADER_SIZE,
                1,
                0,
                16,
                SHM_HEADER_SIZE + 32,
                2,
            )
            mapped[SHM_HEADER_SIZE : SHM_HEADER_SIZE + 32] = struct.pack("<QQQQ", 2, 4, 0, 0)
            mapped[Meta.payload_offset : Meta.payload_offset + 4] = b"abcd"
            mapped.flush()

        meta = Meta()
        meta.shm_name = shm_name
        handle = ShmHandle()
        try:
            assert handle.copy_payload(meta) == b"abcd"
        finally:
            handle.close()
    finally:
        os.close(fd)
        os.unlink(path)
