# shm_sensor_transport_test

`shm_sensor_transport_test` contains integration tests and benchmark utilities
for the shared-memory sensor transport packages.

It provides test fixtures, a C++ benchmark image publisher component,
launch-based integration coverage, and the Python benchmark runner used by the
repository benchmark report.

## Integration Tests

The package includes launch-based tests that exercise a C++ image publisher,
shared-memory relay, and Python subscriber path together.

Run the package tests from the workspace:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon test --base-paths /root/ros2_ws/src --packages-select shm_sensor_transport_test --event-handlers console_direct+
colcon test-result --verbose
```

## Benchmarks

The benchmark runner compares normal Python `sensor_msgs/msg/Image`
subscription with Python shared-memory subscription. Both phases validate
deterministic payload bytes.

Example:

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

Use `--subscribers N` to start multiple Python subscriber worker processes and
measure inter-process subscriber scaling.

Recorded results and rerun commands live in the repository-level
[`BENCHMARK.md`](../BENCHMARK.md).
