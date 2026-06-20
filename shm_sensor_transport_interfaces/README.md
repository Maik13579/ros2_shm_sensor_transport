# shm_sensor_transport_interfaces

`shm_sensor_transport_interfaces` defines the metadata messages used by the C++
and Python shared-memory transport implementations.

The messages do not carry full sensor payloads. They identify the shared-memory
object, ring-buffer slot, payload size, sequence information, and sensor layout
needed by subscribers to copy and reconstruct the payload.

## Messages

| Message | Used for |
| --- | --- |
| `ShmImage` | Metadata for `sensor_msgs/msg/Image` payloads. |
| `ShmCompressedImage` | Metadata for `sensor_msgs/msg/CompressedImage` payloads. |
| `ShmPointCloud2` | Metadata for `sensor_msgs/msg/PointCloud2` payloads. |
| `ShmTransportStatus` | Optional transport status reporting from publishers or relays. |

## Topic Convention

Metadata topics are hidden topics derived from the normal sensor topic:

```text
/camera/image_raw/_shm              ShmImage
/camera/image_raw/compressed/_shm   ShmCompressedImage
/points/_shm                        ShmPointCloud2
```
