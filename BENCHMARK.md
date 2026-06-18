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

The `after 1s` columns recalculate per-sample receive and latency metrics after
discarding the first second of each phase. Phase-wide metrics such as CPU time,
wall throughput, and peak RSS are sampled for the full phase only, so those
post-startup columns show `n/a`.

## 1 MiB Image Payload, 120 Hz Target Rate

Rerun command:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source /root/ros2_ws/install/setup.bash
ROS_LOG_DIR=/tmp/shm_sensor_transport_benchmark_logs \
ros2 run shm_sensor_transport_test benchmark_image_transport \
  --frames 600 \
  --payload-size 1048576 \
  --rate-hz 120 \
  --timeout 30
```

| Metric | Normal full | SHM full | Normal after 1s | SHM after 1s |
| --- | ---: | ---: | ---: | ---: |
| Subscribers | 1 | 1 | 1 | 1 |
| Expected delivered frames | 600 | 600 | n/a | n/a |
| Received frames | 600 | 600 | 479 | 478 |
| Valid frames | 600 / 600 | 600 / 600 | 479 / 479 | 478 / 478 |
| Mean latency | 3.554 ms | 0.727 ms | 3.497 ms | 0.553 ms |
| Median latency | 3.578 ms | 0.556 ms | 3.517 ms | 0.548 ms |
| Min latency | 2.516 ms | 0.410 ms | 2.516 ms | 0.410 ms |
| Max latency | 6.367 ms | 13.036 ms | 6.003 ms | 0.936 ms |
| Max latency after first | 6.003 ms | 7.962 ms | 6.003 ms | 0.936 ms |
| Receive rate | 120.3 fps | 120.5 fps | 120.3 fps | 120.2 fps |
| Receive throughput | 120.3 MiB/s | 120.5 MiB/s | 120.3 MiB/s | 120.2 MiB/s |
| Wall throughput | 79.9 MiB/s | 80.0 MiB/s | n/a | n/a |
| CPU time / wall time | 5.353 s / 7.505 s | 3.067 s / 7.499 s | n/a | n/a |
| CPU percent | 71.3% | 40.9% | n/a | n/a |
| Max RSS, benchmark process | 62,812 KiB | 63,068 KiB | n/a | n/a |
| Max RSS, child processes | 71,824 KiB | 76,492 KiB | n/a | n/a |

Both phases use best-effort sensor QoS. The final run received all 600
published frames in each phase; all received frames passed deterministic payload
validation.

## 1 MiB Image Payload, 120 Hz Target Rate, 2 Subscriber Processes

Rerun command:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source /root/ros2_ws/install/setup.bash
ROS_LOG_DIR=/tmp/shm_sensor_transport_benchmark_logs \
ros2 run shm_sensor_transport_test benchmark_image_transport \
  --frames 600 \
  --payload-size 1048576 \
  --rate-hz 120 \
  --timeout 30 \
  --subscribers 2
```

| Metric | Normal full | SHM full | Normal after 1s | SHM after 1s |
| --- | ---: | ---: | ---: | ---: |
| Subscribers | 2 | 2 | 2 | 2 |
| Expected delivered frames | 1200 | 1200 | n/a | n/a |
| Received frames | 1196 | 1200 | 955 | 956 |
| Valid frames | 1196 / 1196 | 1200 / 1200 | 955 / 955 | 956 / 956 |
| Mean latency | 5.062 ms | 0.794 ms | 5.022 ms | 0.610 ms |
| Median latency | 5.024 ms | 0.596 ms | 4.964 ms | 0.585 ms |
| Min latency | 3.578 ms | 0.448 ms | 3.578 ms | 0.448 ms |
| Max latency | 7.129 ms | 12.924 ms | 6.922 ms | 1.434 ms |
| Max latency after first | 6.982 ms | 8.166 ms | 6.922 ms | 1.434 ms |
| Receive rate | 239.7 fps | 241.1 fps | 239.8 fps | 240.5 fps |
| Receive throughput | 239.7 MiB/s | 241.1 MiB/s | 239.8 MiB/s | 240.5 MiB/s |
| Wall throughput | 140.5 MiB/s | 159.8 MiB/s | n/a | n/a |
| CPU time / wall time | 10.050 s / 8.514 s | 4.495 s / 7.508 s | n/a | n/a |
| CPU percent | 118.0% | 59.9% | n/a | n/a |
| Max RSS, benchmark process | 62,476 KiB | 62,988 KiB | n/a | n/a |
| Max RSS, child processes | 71,848 KiB | 76,548 KiB | n/a | n/a |

Result: the normal path missed four best-effort samples, while the shared-memory
path delivered all requested frames. After the first second, the shared-memory
path reduced mean latency from `5.022 ms` to `0.610 ms`.

## 1 MiB Image Payload, 120 Hz Target Rate, 3 Subscriber Processes

Rerun command:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source /root/ros2_ws/install/setup.bash
ROS_LOG_DIR=/tmp/shm_sensor_transport_benchmark_logs \
ros2 run shm_sensor_transport_test benchmark_image_transport \
  --frames 600 \
  --payload-size 1048576 \
  --rate-hz 120 \
  --timeout 30 \
  --subscribers 3
```

| Metric | Normal full | SHM full | Normal after 1s | SHM after 1s |
| --- | ---: | ---: | ---: | ---: |
| Subscribers | 3 | 3 | 3 | 3 |
| Expected delivered frames | 1800 | 1800 | n/a | n/a |
| Received frames | 1800 | 1800 | 1437 | 1437 |
| Valid frames | 1800 / 1800 | 1800 / 1800 | 1437 / 1437 | 1437 / 1437 |
| Mean latency | 6.711 ms | 0.934 ms | 6.770 ms | 0.705 ms |
| Median latency | 6.540 ms | 0.732 ms | 6.587 ms | 0.690 ms |
| Min latency | 5.164 ms | 0.475 ms | 5.235 ms | 0.475 ms |
| Max latency | 13.964 ms | 12.786 ms | 13.964 ms | 2.320 ms |
| Max latency after first | 13.964 ms | 6.292 ms | 13.964 ms | 2.320 ms |
| Receive rate | 358.9 fps | 360.9 fps | 358.4 fps | 360.8 fps |
| Receive throughput | 358.9 MiB/s | 360.9 MiB/s | 358.4 MiB/s | 360.8 MiB/s |
| Wall throughput | 239.1 MiB/s | 239.6 MiB/s | n/a | n/a |
| CPU time / wall time | 16.553 s / 7.528 s | 6.577 s / 7.511 s | n/a | n/a |
| CPU percent | 219.9% | 87.6% | n/a | n/a |
| Max RSS, benchmark process | 62,700 KiB | 63,056 KiB | n/a | n/a |
| Max RSS, child processes | 72,088 KiB | 76,504 KiB | n/a | n/a |

Result: both paths delivered all requested frames. The shared-memory path reduced
post-startup mean latency from `6.770 ms` to `0.705 ms`. CPU still increases
with each additional shared-memory subscriber because every Python process
copies the payload into process-local memory.

## 1 MiB Image Payload, 60 Hz Target Rate

Rerun command:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source /root/ros2_ws/install/setup.bash
ROS_LOG_DIR=/tmp/shm_sensor_transport_benchmark_logs \
ros2 run shm_sensor_transport_test benchmark_image_transport \
  --frames 300 \
  --payload-size 1048576 \
  --rate-hz 60 \
  --timeout 30
```

| Metric | Normal full | SHM full | Normal after 1s | SHM after 1s |
| --- | ---: | ---: | ---: | ---: |
| Subscribers | 1 | 1 | 1 | 1 |
| Expected delivered frames | 300 | 300 | n/a | n/a |
| Received frames | 287 | 300 | 230 | 239 |
| Valid frames | 287 / 287 | 300 / 300 | 230 / 230 | 239 / 239 |
| Mean latency | 4.439 ms | 0.964 ms | 4.379 ms | 0.643 ms |
| Median latency | 4.521 ms | 0.627 ms | 4.472 ms | 0.605 ms |
| Min latency | 3.108 ms | 0.504 ms | 3.108 ms | 0.504 ms |
| Max latency | 6.584 ms | 13.239 ms | 5.617 ms | 2.047 ms |
| Max latency after first | 6.584 ms | 2.197 ms | 5.617 ms | 2.047 ms |
| Receive rate | 57.6 fps | 60.4 fps | 58.0 fps | 60.3 fps |
| Receive throughput | 57.6 MiB/s | 60.4 MiB/s | 58.0 MiB/s | 60.3 MiB/s |
| Wall throughput | 33.7 MiB/s | 40.0 MiB/s | n/a | n/a |
| CPU time / wall time | 3.241 s / 8.506 s | 1.892 s / 7.502 s | n/a | n/a |
| CPU percent | 38.1% | 25.2% | n/a | n/a |
| Max RSS, benchmark process | 62,364 KiB | 62,620 KiB | n/a | n/a |
| Max RSS, child processes | 71,852 KiB | 76,528 KiB | n/a | n/a |

The normal phase missed thirteen best-effort samples. The shared-memory phase
completed all requested frames.

## 256 KiB Image Payload, 240 Hz Target Rate

Rerun command:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source /root/ros2_ws/install/setup.bash
ROS_LOG_DIR=/tmp/shm_sensor_transport_benchmark_logs \
ros2 run shm_sensor_transport_test benchmark_image_transport \
  --frames 1200 \
  --payload-size 262144 \
  --rate-hz 240 \
  --timeout 30
```

| Metric | Normal full | SHM full | Normal after 1s | SHM after 1s |
| --- | ---: | ---: | ---: | ---: |
| Subscribers | 1 | 1 | 1 | 1 |
| Expected delivered frames | 1200 | 1200 | n/a | n/a |
| Received frames | 1200 | 1200 | 959 | 959 |
| Valid frames | 1200 / 1200 | 1200 / 1200 | 959 / 959 | 959 / 959 |
| Mean latency | 0.534 ms | 0.289 ms | 0.522 ms | 0.285 ms |
| Median latency | 0.506 ms | 0.282 ms | 0.502 ms | 0.281 ms |
| Min latency | 0.378 ms | 0.181 ms | 0.378 ms | 0.181 ms |
| Max latency | 1.893 ms | 3.918 ms | 0.898 ms | 0.503 ms |
| Max latency after first | 1.806 ms | 0.535 ms | 0.898 ms | 0.503 ms |
| Receive rate | 240.3 fps | 240.4 fps | 240.2 fps | 240.3 fps |
| Receive throughput | 60.1 MiB/s | 60.1 MiB/s | 60.1 MiB/s | 60.1 MiB/s |
| Wall throughput | 40.0 MiB/s | 40.0 MiB/s | n/a | n/a |
| CPU time / wall time | 2.756 s / 7.504 s | 2.260 s / 7.495 s | n/a | n/a |
| CPU percent | 36.7% | 30.2% | n/a | n/a |
| Max RSS, benchmark process | 62,672 KiB | 63,056 KiB | n/a | n/a |
| Max RSS, child processes | 67,512 KiB | 67,816 KiB | n/a | n/a |

Both phases completed all requested frames. At this smaller payload size, the
absolute latency gap is smaller, but the shared-memory path still used less CPU.

## 4 MiB Image Payload, 30 Hz Target Rate

Rerun command:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source /root/ros2_ws/install/setup.bash
ROS_LOG_DIR=/tmp/shm_sensor_transport_benchmark_logs \
ros2 run shm_sensor_transport_test benchmark_image_transport \
  --frames 150 \
  --payload-size 4194304 \
  --rate-hz 30 \
  --timeout 30
```

| Metric | Normal full | SHM full | Normal after 1s | SHM after 1s |
| --- | ---: | ---: | ---: | ---: |
| Subscribers | 1 | 1 | 1 | 1 |
| Expected delivered frames | 150 | 150 | n/a | n/a |
| Received frames | 135 | 150 | 106 | 118 |
| Valid frames | 135 / 135 | 150 / 150 | 106 / 106 | 118 / 118 |
| Mean latency | 15.603 ms | 2.554 ms | 15.525 ms | 2.115 ms |
| Median latency | 15.286 ms | 2.049 ms | 15.267 ms | 2.043 ms |
| Min latency | 11.875 ms | 1.829 ms | 11.875 ms | 1.877 ms |
| Max latency | 21.722 ms | 49.998 ms | 18.178 ms | 5.135 ms |
| Max latency after first | 18.289 ms | 20.438 ms | 18.178 ms | 5.135 ms |
| Receive rate | 29.8 fps | 30.5 fps | 30.2 fps | 30.3 fps |
| Receive throughput | 119.2 MiB/s | 122.0 MiB/s | 121.0 MiB/s | 121.1 MiB/s |
| Wall throughput | 56.7 MiB/s | 79.8 MiB/s | n/a | n/a |
| CPU time / wall time | 5.730 s / 9.519 s | 3.159 s / 7.517 s | n/a | n/a |
| CPU percent | 60.2% | 42.0% | n/a | n/a |
| Max RSS, benchmark process | 62,740 KiB | 63,124 KiB | n/a | n/a |
| Max RSS, child processes | 152,296 KiB | 152,296 KiB | n/a | n/a |

The normal phase missed fifteen best-effort samples. The shared-memory phase
completed all requested frames and reduced mean latency and CPU time on this
large-payload run.
