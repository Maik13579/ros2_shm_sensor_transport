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

from dataclasses import dataclass


SHM_MAGIC = 0x53544D48534F5232
SHM_LAYOUT_VERSION = 1
SHM_HEADER_SIZE = 48
SLOT_HEADER_SIZE = 32


@dataclass(frozen=True)
class SharedMemoryLayout:
    magic: int
    version: int
    header_size: int
    slot_count: int
    slot_size: int
    payload_base_offset: int
    generation: int


@dataclass(frozen=True)
class SlotSnapshot:
    sequence: int
    payload_size: int
