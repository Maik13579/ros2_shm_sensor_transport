# shm_sensor_transport

`shm_sensor_transport` is a ROS 2 transport path for high-bandwidth, intra-host
sensor streams. It is intended for image and point-cloud pipelines where the
standard ROS 2 topic remains available, but Python consumers can avoid the cost
of deserializing large `sensor_msgs/Image` or `sensor_msgs/PointCloud2` messages
on every callback.

## Why

Many robotics perception systems are split between C++ sensor drivers and Python
processing code. ROS 2 keeps that split productive, but very large sensor
messages can become expensive when they cross into Python through normal rclpy
subscription paths. The data is already local to the machine, yet it still has to
move through DDS serialization, Python message construction, and downstream
conversion before the user callback can work with the pixels or points.

This package keeps normal ROS 2 compatibility and adds a local fast path:

- The original sensor topic is still published as `sensor_msgs/Image` or
  `sensor_msgs/PointCloud2`.
- A C++ relay subscribes to that topic and writes only the raw payload bytes into
  a fixed-size shared-memory ring buffer.
- The relay publishes a small ROS 2 metadata message that identifies the shared
  memory object, slot, sequence, and sensor layout.
- Python subscribers receive the metadata, copy the selected payload bytes from
  shared memory, validate that the slot was not overwritten during the copy, and
  pass loader output to user code.

The Python side still copies the payload before invoking callbacks. That copy is
intentional: it gives user code normal object lifetimes while allowing the C++
writer to keep reusing ring-buffer slots.

## Architecture

The transport has three ROS 2 packages:

- `shm_sensor_transport_interfaces`: message and service definitions shared by
  the C++ and Python packages.
- `shm_sensor_transport`: C++ relay components and shared-memory ring-buffer
  implementation.
- `shm_sensor_transport_py`: Python subscriber API, shared-memory handles, and
  loader plugins.

Typical data flow:

```text
Sensor driver process
  └── publishes /camera/image_raw as sensor_msgs/Image

C++ relay component
  ├── subscribes to /camera/image_raw
  ├── allocates /dev/shm/ros2_shm_camera_image_raw_<hash>
  ├── writes msg.data into the next ring-buffer slot
  └── publishes /camera/image_raw/shm as ShmImage metadata

Python process
  └── ShmSubscriber
        ├── subscribes to /camera/image_raw/shm
        ├── opens and caches the shared-memory object
        ├── copies the slot payload into Python-owned memory
        ├── validates slot sequence counters
        └── calls the user callback with loader output
```

Point clouds follow the same pattern using `sensor_msgs/PointCloud2` input and
`ShmPointCloud2` metadata.

## Shared Memory Model

Each relay owns one shared-memory object. The object contains:

```text
SharedMemoryHeader
SlotHeader[slot_count]
PayloadSlot[slot_count]
```

Every payload slot has the same configured size. If `slot_size_bytes` is zero,
the relay infers the slot size from the first received message. A slot sequence
counter is odd while the writer is updating the slot and even once the payload is
complete. Readers accept a copy only when the sequence value before and after the
copy is identical and even.

This gives latest-frame behavior suitable for high-rate sensor streams. It does
not provide reliable history for frames whose slots have already been reused.

## Compatibility

The shared-memory stream is an additional local transport, not a replacement for
normal ROS 2 communication:

```text
/camera/image_raw       sensor_msgs/Image
/camera/image_raw/shm   shm_sensor_transport_interfaces/ShmImage

/points                 sensor_msgs/PointCloud2
/points/shm             shm_sensor_transport_interfaces/ShmPointCloud2
```

Existing ROS 2 tools and remote subscribers can continue to use the original
topics. Local Python perception code can opt into the shared-memory metadata
topic when it benefits from avoiding Python-side deserialization of the full
sensor message.

## Limits

- Only intra-host communication is supported.
- The relay still receives the original ROS 2 sensor message.
- Python subscribers copy payload bytes before invoking user callbacks.
- Overwritten ring-buffer slots are dropped, not recovered.
- Maximum efficiency requires direct sensor-driver integration with the shared
  memory writer rather than a relay subscribed to an existing topic.
