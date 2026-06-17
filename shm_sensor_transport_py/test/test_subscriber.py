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

import pytest

from shm_sensor_transport_py.subscriber import resolve_metadata_topic


def test_resolve_metadata_topic_appends_hidden_suffix():
    assert resolve_metadata_topic("/camera/image_raw") == "/camera/image_raw/_shm"


def test_resolve_metadata_topic_keeps_existing_hidden_suffix():
    assert resolve_metadata_topic("/camera/image_raw/_shm") == "/camera/image_raw/_shm"


def test_resolve_metadata_topic_strips_trailing_slashes():
    assert resolve_metadata_topic("/camera/image_raw/") == "/camera/image_raw/_shm"


def test_resolve_metadata_topic_rejects_empty_topic():
    with pytest.raises(ValueError, match="topic must not be empty"):
        resolve_metadata_topic("/")
