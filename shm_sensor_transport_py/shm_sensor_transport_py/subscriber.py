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

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from shm_sensor_transport_interfaces.msg import ShmImage

from shm_sensor_transport_py.shm_handle import ShmFrameInvalid, ShmHandle


def resolve_metadata_topic(topic: str) -> str:
    """Return the hidden shared-memory metadata topic for a normal sensor topic."""
    normalized = topic.rstrip("/")
    if not normalized:
        raise ValueError("topic must not be empty")
    if normalized.endswith("/_shm"):
        return normalized
    return f"{normalized}/_shm"


class ShmSubscriber:
    """Subscribe to metadata and invoke callbacks with copied shared-memory payloads."""

    def __init__(
        self,
        *,
        node,
        topic: str,
        loader,
        callback,
        msg_type=None,
        qos_profile: QoSProfile = None,
    ) -> None:
        self._node = node
        self._loader = loader
        self._callback = callback
        self._handle = ShmHandle()
        self._msg_type = msg_type or getattr(loader, "msg_type", ShmImage)
        self._metadata_topic = resolve_metadata_topic(topic)
        self._qos = qos_profile or QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscription = node.create_subscription(
            self._msg_type,
            self._metadata_topic,
            self._metadata_callback,
            self._qos,
        )

    @property
    def subscription(self):
        return self._subscription

    @property
    def metadata_topic(self) -> str:
        return self._metadata_topic

    def close(self) -> None:
        self._handle.close()

    def _metadata_callback(self, meta) -> None:
        try:
            payload = self._handle.copy_payload(meta)
            decoded = self._loader.from_bytes(payload, meta)
            self._callback(decoded, meta)
        except ShmFrameInvalid as error:
            self._node.get_logger().debug(f"Dropped invalid shared-memory frame: {error}")
        except Exception as error:  # noqa: BLE001 - callback failures should not kill the subscription
            self._node.get_logger().exception(f"Shared-memory subscriber callback failed: {error}")
