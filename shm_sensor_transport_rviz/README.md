# shm_sensor_transport_rviz

`shm_sensor_transport_rviz` contains RViz display plugins for inspecting streams
carried by the shared-memory transport.

The displays subscribe to the shared-memory metadata stream, copy the payload
from the referenced shared-memory slot, and hand reconstructed sensor messages to
the corresponding RViz rendering path.

## Topic Convention

Use the normal sensor topic when configuring displays. The transport code maps
the topic to the hidden metadata topic:

```text
/camera/image_raw       -> /camera/image_raw/_shm
/points                 -> /points/_shm
```

Compressed image streams use the compressed base topic:

```text
/camera/image_raw/compressed -> /camera/image_raw/compressed/_shm
```

## Notes

- RViz must run on the same host as the shared-memory publisher.
- Visualization still copies payload bytes out of shared memory before rendering.
- For remote visualization, publish or relay a normal ROS sensor topic instead.
