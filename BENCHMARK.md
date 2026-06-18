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
| Received frames | 116 | 120 |
| Valid frames | 116 / 116 | 120 / 120 |
| Mean latency | 3.830 ms | 1.644 ms |
| Median latency | 3.697 ms | 1.862 ms |
| Min latency | 3.550 ms | 0.476 ms |
| Max latency | 6.151 ms | 12.846 ms |
| Max latency after first | 5.118 ms | 7.149 ms |
| Receive rate | 118.3 fps | 122.6 fps |
| Receive throughput | 118.3 MiB/s | 122.6 MiB/s |
| Wall throughput | 25.8 MiB/s | 34.3 MiB/s |
| CPU time / wall time | 1.566 s / 4.503 s | 1.197 s / 3.503 s |
| CPU percent | 34.8% | 34.2% |
| Max RSS, benchmark process | 62,748 KiB | 62,876 KiB |
| Max RSS, child processes | 71,176 KiB | 76,068 KiB |

Both phases use best-effort sensor QoS. The final run received 116 of 120
published frames in the normal phase and 120 of 120 in the shared-memory phase;
all received frames passed deterministic payload validation.

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
| Received frames | 238 | 240 |
| Valid frames | 238 / 238 | 240 / 240 |
| Mean latency | 5.267 ms | 1.620 ms |
| Median latency | 5.243 ms | 1.877 ms |
| Min latency | 4.138 ms | 0.458 ms |
| Max latency | 7.031 ms | 13.012 ms |
| Max latency after first | 6.985 ms | 7.330 ms |
| Receive rate | 240.4 fps | 245.3 fps |
| Receive throughput | 240.4 MiB/s | 245.3 MiB/s |
| Wall throughput | 52.8 MiB/s | 68.6 MiB/s |
| CPU time / wall time | 3.090 s / 4.509 s | 2.063 s / 3.501 s |
| CPU percent | 68.5% | 58.9% |
| Max RSS, benchmark process | 62,548 KiB | 62,548 KiB |
| Max RSS, child processes | 71,384 KiB | 75,912 KiB |

Result: the normal path missed two best-effort samples, while the shared-memory
path delivered all requested frames. The shared-memory path reduced mean latency
from `5.267 ms` to `1.620 ms` and aggregate CPU time from `3.090 s` to
`2.063 s`.

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
| Mean latency | 6.355 ms | 1.681 ms |
| Median latency | 6.366 ms | 1.912 ms |
| Min latency | 4.419 ms | 0.459 ms |
| Max latency | 8.740 ms | 13.488 ms |
| Max latency after first | 8.740 ms | 7.645 ms |
| Receive rate | 364.1 fps | 367.7 fps |
| Receive throughput | 364.1 MiB/s | 367.7 MiB/s |
| Wall throughput | 102.3 MiB/s | 102.3 MiB/s |
| CPU time / wall time | 5.010 s / 3.518 s | 3.437 s / 3.518 s |
| CPU percent | 142.4% | 97.7% |
| Max RSS, benchmark process | 62,748 KiB | 63,004 KiB |
| Max RSS, child processes | 71,560 KiB | 76,016 KiB |

Result: both paths delivered all requested frames. The shared-memory path reduced
mean latency from `6.355 ms` to `1.681 ms` and aggregate CPU time from
`5.010 s` to `3.437 s`. CPU still increases with each additional shared-memory
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
| Subscribers | 1 | 1 |
| Expected delivered frames | 120 | 120 |
| Received frames | 117 | 120 |
| Valid frames | 117 / 117 | 120 / 120 |
| Mean latency | 3.928 ms | 1.658 ms |
| Median latency | 3.751 ms | 2.002 ms |
| Min latency | 3.590 ms | 0.471 ms |
| Max latency | 5.932 ms | 12.381 ms |
| Max latency after first | 5.389 ms | 2.611 ms |
| Receive rate | 59.5 fps | 60.9 fps |
| Receive throughput | 59.5 MiB/s | 60.9 MiB/s |
| Wall throughput | 21.3 MiB/s | 26.7 MiB/s |
| CPU time / wall time | 1.576 s / 5.505 s | 1.181 s / 4.502 s |
| CPU percent | 28.6% | 26.2% |
| Max RSS, benchmark process | 62,452 KiB | 62,708 KiB |
| Max RSS, child processes | 72,576 KiB | 74,948 KiB |

The normal phase missed three best-effort samples. The shared-memory phase
completed all requested frames.

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
| Subscribers | 1 | 1 |
| Expected delivered frames | 240 | 240 |
| Received frames | 240 | 240 |
| Valid frames | 240 / 240 | 240 / 240 |
| Mean latency | 0.557 ms | 0.329 ms |
| Median latency | 0.517 ms | 0.308 ms |
| Min latency | 0.415 ms | 0.252 ms |
| Max latency | 1.814 ms | 3.901 ms |
| Max latency after first | 1.514 ms | 0.474 ms |
| Receive rate | 241.4 fps | 241.9 fps |
| Receive throughput | 60.3 MiB/s | 60.5 MiB/s |
| Wall throughput | 17.2 MiB/s | 17.1 MiB/s |
| CPU time / wall time | 0.978 s / 3.498 s | 0.930 s / 3.504 s |
| CPU percent | 28.0% | 26.5% |
| Max RSS, benchmark process | 62,440 KiB | 62,696 KiB |
| Max RSS, child processes | 67,472 KiB | 67,472 KiB |

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
| Subscribers | 1 | 1 |
| Expected delivered frames | 60 | 60 |
| Received frames | 50 | 60 |
| Valid frames | 50 / 50 | 60 / 60 |
| Mean latency | 15.905 ms | 3.313 ms |
| Median latency | 15.438 ms | 2.093 ms |
| Min latency | 13.395 ms | 2.005 ms |
| Max latency | 22.188 ms | 48.790 ms |
| Max latency after first | 18.829 ms | 24.337 ms |
| Receive rate | 30.2 fps | 31.3 fps |
| Receive throughput | 121.0 MiB/s | 125.2 MiB/s |
| Wall throughput | 30.7 MiB/s | 53.1 MiB/s |
| CPU time / wall time | 2.611 s / 6.519 s | 1.659 s / 4.522 s |
| CPU percent | 40.1% | 36.7% |
| Max RSS, benchmark process | 62,472 KiB | 62,472 KiB |
| Max RSS, child processes | 156,324 KiB | 156,324 KiB |

The normal phase missed ten best-effort samples. The shared-memory phase
completed all requested frames and reduced mean latency and CPU time on this
large-payload run.
