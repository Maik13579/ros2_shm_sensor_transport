# Benchmarks

This benchmark compares two ways of delivering the same deterministic
`sensor_msgs/Image` stream to Python:

- Normal phase: a C++ image publisher component publishes `sensor_msgs/Image`;
  a Python node subscribes directly to the image topic.
- Shared-memory phase: the same C++ publisher component and the
  `ShmImageRelayComponent` are loaded into one component container with
  intra-process communication enabled; a Python `ShmSubscriber` subscribes to
  the normal image topic and reads the payload through the hidden `/_shm`
  metadata topic.

The phases run one after the other. The benchmark parent launches the C++
component container and, for `--subscribers N`, starts `N` separate Python
subscriber worker processes. Subscriber scaling results therefore measure
inter-process delivery rather than multiple subscriptions in a single Python
executor. Both Python paths use ROS-message loaders and return
`sensor_msgs.msg.Image`, so the result compares the transport path instead of
comparing different user-facing message types.

Each image payload is deterministic: byte `i` in frame `n` is `(n + i) % 251`.
The subscriber checks sentinel bytes in every received frame and reports valid
frames separately from received frames. Best-effort sensor QoS is used, so
received frame count can be lower than the requested publish count under load.

Latency is measured from the timestamp written by the C++ publisher to the time
the Python subscriber receives the reconstructed image. CPU time and RSS are
sampled around each phase. The shared-memory subscriber keeps the POSIX
shared-memory object open after the first metadata sample for a given `shm_name`
and reuses the cached file descriptor and `mmap` until the shared-memory name
changes or the subscriber is closed.

## 1 MiB Image Payload, 120 Hz Target Rate

Rerun command:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source /root/ros2_ws/install/setup.bash
ROS_LOG_DIR=/tmp/shm_sensor_transport_benchmark_logs \
ros2 run shm_sensor_transport_test benchmark_image_transport \
  --frames 120 \
  --payload-size 1048576 \
  --rate-hz 120 \
  --timeout 20
```

| Metric | Normal Python Image subscriber | Python ShmSubscriber |
| --- | ---: | ---: |
| Subscribers | 1 | 1 |
| Expected delivered frames | 120 | 120 |
| Received frames | 119 | 119 |
| Valid frames | 119 / 119 | 119 / 119 |
| Mean latency | 4.180 ms | 0.944 ms |
| Median latency | 4.136 ms | 0.785 ms |
| Min latency | 2.851 ms | 0.544 ms |
| Max latency | 5.380 ms | 15.617 ms |
| Receive rate | 120.7 fps | 122.0 fps |
| Receive throughput | 120.7 MiB/s | 122.0 MiB/s |
| Wall throughput | 26.9 MiB/s | 26.8 MiB/s |
| CPU time / wall time | 1.492 s / 4.421 s | 1.011 s / 4.438 s |
| CPU percent | 33.7% | 22.8% |
| Max RSS, benchmark process | 84,720 KiB | 91,852 KiB |
| Max RSS, child processes | 79,076 KiB | 84,720 KiB |

Both phases use best-effort sensor QoS. The final run received 119 of 120
published frames in each phase; all received frames passed deterministic payload
validation.

## 1 MiB Image Payload, 120 Hz Target Rate, 2 Subscriber Processes

Rerun command:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source /root/ros2_ws/install/setup.bash
ROS_LOG_DIR=/tmp/shm_sensor_transport_benchmark_logs \
ros2 run shm_sensor_transport_test benchmark_image_transport \
  --frames 120 \
  --payload-size 1048576 \
  --rate-hz 120 \
  --timeout 20 \
  --subscribers 2
```

| Metric | Normal Python Image subscriber | Python ShmSubscriber |
| --- | ---: | ---: |
| Subscribers | 2 | 2 |
| Expected delivered frames | 240 | 240 |
| Received frames | 240 | 240 |
| Valid frames | 240 / 240 | 240 / 240 |
| Mean latency | 13.060 ms | 1.714 ms |
| Median latency | 12.759 ms | 1.981 ms |
| Min latency | 11.231 ms | 0.505 ms |
| Max latency | 18.528 ms | 13.290 ms |
| Receive rate | 158.7 fps | 245.1 fps |
| Receive throughput | 158.7 MiB/s | 245.1 MiB/s |
| Wall throughput | 53.1 MiB/s | 68.4 MiB/s |
| CPU time / wall time | 5.262 s / 4.518 s | 2.499 s / 3.510 s |
| CPU percent | 116.5% | 71.2% |
| Max RSS, benchmark process | 63,268 KiB | 63,380 KiB |
| Max RSS, child processes | 71,564 KiB | 75,424 KiB |

Result: both paths delivered all requested frames. The shared-memory path reduced
mean latency from `13.060 ms` to `1.714 ms` and aggregate CPU time from
`5.262 s` to `2.499 s`.

## 1 MiB Image Payload, 120 Hz Target Rate, 3 Subscriber Processes

Rerun command:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source /root/ros2_ws/install/setup.bash
ROS_LOG_DIR=/tmp/shm_sensor_transport_benchmark_logs \
ros2 run shm_sensor_transport_test benchmark_image_transport \
  --frames 120 \
  --payload-size 1048576 \
  --rate-hz 120 \
  --timeout 20 \
  --subscribers 3
```

| Metric | Normal Python Image subscriber | Python ShmSubscriber |
| --- | ---: | ---: |
| Subscribers | 3 | 3 |
| Expected delivered frames | 360 | 360 |
| Received frames | 360 | 360 |
| Valid frames | 360 / 360 | 360 / 360 |
| Mean latency | 14.443 ms | 1.803 ms |
| Median latency | 13.936 ms | 2.001 ms |
| Min latency | 11.225 ms | 0.563 ms |
| Max latency | 20.841 ms | 13.088 ms |
| Receive rate | 210.6 fps | 367.9 fps |
| Receive throughput | 210.6 MiB/s | 367.9 MiB/s |
| Wall throughput | 79.5 MiB/s | 102.1 MiB/s |
| CPU time / wall time | 8.125 s / 4.528 s | 4.270 s / 3.525 s |
| CPU percent | 179.5% | 121.1% |
| Max RSS, benchmark process | 62,784 KiB | 63,168 KiB |
| Max RSS, child processes | 72,132 KiB | 76,496 KiB |

Result: both paths delivered all requested frames. The shared-memory path reduced
mean latency from `14.443 ms` to `1.803 ms` and aggregate CPU time from
`8.125 s` to `4.270 s`. CPU still increases with each additional shared-memory
subscriber because every Python process copies the payload into process-local
memory.

## 1 MiB Image Payload, 60 Hz Target Rate

Rerun command:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source /root/ros2_ws/install/setup.bash
ROS_LOG_DIR=/tmp/shm_sensor_transport_benchmark_logs \
ros2 run shm_sensor_transport_test benchmark_image_transport \
  --frames 120 \
  --payload-size 1048576 \
  --rate-hz 60 \
  --timeout 20
```

| Metric | Normal Python Image subscriber | Python ShmSubscriber |
| --- | ---: | ---: |
| Received frames | 119 | 120 |
| Valid frames | 119 / 119 | 120 / 120 |
| Mean latency | 5.336 ms | 1.110 ms |
| Median latency | 3.650 ms | 0.839 ms |
| Min latency | 2.870 ms | 0.575 ms |
| Max latency | 13.475 ms | 25.024 ms |
| Receive rate | 60.7 fps | 61.4 fps |
| Receive throughput | 60.7 MiB/s | 61.4 MiB/s |
| Wall throughput | 5.8 MiB/s | 27.1 MiB/s |
| CPU time / wall time | 2.242 s / 20.417 s | 0.873 s / 4.426 s |
| CPU percent | 11.0% | 19.7% |
| Max RSS, benchmark process | 85,068 KiB | 92,324 KiB |
| Max RSS, child processes | 79,224 KiB | 85,068 KiB |

The normal phase missed one best-effort sample and therefore waited for the
benchmark timeout in this run. The shared-memory phase completed all requested
frames before the timeout.

## 256 KiB Image Payload, 240 Hz Target Rate

Rerun command:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source /root/ros2_ws/install/setup.bash
ROS_LOG_DIR=/tmp/shm_sensor_transport_benchmark_logs \
ros2 run shm_sensor_transport_test benchmark_image_transport \
  --frames 240 \
  --payload-size 262144 \
  --rate-hz 240 \
  --timeout 20
```

| Metric | Normal Python Image subscriber | Python ShmSubscriber |
| --- | ---: | ---: |
| Received frames | 240 | 240 |
| Valid frames | 240 / 240 | 240 / 240 |
| Mean latency | 0.607 ms | 0.414 ms |
| Median latency | 0.476 ms | 0.379 ms |
| Min latency | 0.282 ms | 0.264 ms |
| Max latency | 8.268 ms | 4.154 ms |
| Receive rate | 243.0 fps | 242.0 fps |
| Receive throughput | 60.7 MiB/s | 60.5 MiB/s |
| Wall throughput | 17.5 MiB/s | 17.5 MiB/s |
| CPU time / wall time | 1.006 s / 3.423 s | 0.735 s / 3.424 s |
| CPU percent | 29.4% | 21.5% |
| Max RSS, benchmark process | 80,440 KiB | 84,152 KiB |
| Max RSS, child processes | 78,720 KiB | 81,220 KiB |

Both phases completed all requested frames. At this smaller payload size, the
absolute latency gap is smaller, but the shared-memory path still used less CPU.

## 4 MiB Image Payload, 30 Hz Target Rate

Rerun command:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source /root/ros2_ws/install/setup.bash
ROS_LOG_DIR=/tmp/shm_sensor_transport_benchmark_logs \
ros2 run shm_sensor_transport_test benchmark_image_transport \
  --frames 60 \
  --payload-size 4194304 \
  --rate-hz 30 \
  --timeout 20
```

| Metric | Normal Python Image subscriber | Python ShmSubscriber |
| --- | ---: | ---: |
| Received frames | 58 | 60 |
| Valid frames | 58 / 58 | 60 / 60 |
| Mean latency | 19.987 ms | 3.953 ms |
| Median latency | 18.800 ms | 2.254 ms |
| Min latency | 15.479 ms | 2.060 ms |
| Max latency | 32.919 ms | 64.225 ms |
| Receive rate | 29.5 fps | 31.5 fps |
| Receive throughput | 118.2 MiB/s | 125.8 MiB/s |
| Wall throughput | 40.7 MiB/s | 53.9 MiB/s |
| CPU time / wall time | 3.486 s / 5.698 s | 1.695 s / 4.454 s |
| CPU percent | 61.2% | 38.1% |
| Max RSS, benchmark process | 96,600 KiB | 122,540 KiB |
| Max RSS, child processes | 98,968 KiB | 110,428 KiB |

The normal phase missed two best-effort samples. The shared-memory phase
completed all requested frames and reduced mean latency and CPU time on this
large-payload run.
