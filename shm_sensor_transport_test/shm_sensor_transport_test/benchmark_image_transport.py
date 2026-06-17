# Copyright 2026 Maik Knof
#
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

import argparse
import os
import resource
import statistics
import threading
import time
from dataclasses import dataclass

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image

from shm_sensor_transport_py import ShmSubscriber
from shm_sensor_transport_py.loaders import RosImageLoader


IMAGE_TOPIC = '/benchmark/image_raw'


@dataclass
class ResourceSample:
    """CPU and memory counters captured before or after one benchmark phase."""

    monotonic: float
    user_seconds: float
    system_seconds: float
    self_max_rss_kib: int
    children_max_rss_kib: int


@dataclass
class PhaseMetrics:
    """Computed resource counters for one benchmark phase."""

    wall_seconds: float
    cpu_seconds: float
    cpu_percent: float
    self_max_rss_kib: int
    children_max_rss_kib: int


@dataclass
class PhaseSummary:
    """Display-ready benchmark measurements for one phase."""

    received_frames: int
    valid_frames: int
    mean_latency_ms: float
    median_latency_ms: float
    min_latency_ms: float
    max_latency_ms: float
    receive_rate_fps: float
    receive_throughput_mib_s: float
    wall_throughput_mib_s: float
    cpu_seconds: float
    wall_seconds: float
    cpu_percent: float
    self_max_rss_kib: int
    children_max_rss_kib: int


class BenchmarkSubscriber(Node):
    """Measure one benchmark phase and validate each received payload."""

    def __init__(self, mode: str, frame_count: int, payload_size: int, rate_hz: float) -> None:
        super().__init__(f'shm_sensor_transport_{mode}_benchmark_subscriber')
        self._mode = mode
        self._frame_count = frame_count
        self._payload_size = payload_size
        self._expected_receive_window = (frame_count / max(1.0, rate_hz)) + 1.0
        self.latencies_ms = []
        self.valid_frames = 0
        self.first_receive_time = None
        self.last_receive_time = None
        if mode == 'normal':
            self._subscription = self.create_subscription(
                Image,
                IMAGE_TOPIC,
                self._on_normal_image,
                qos_profile_sensor_data,
            )
        elif mode == 'shm':
            self._subscription = ShmSubscriber(
                node=self,
                topic=IMAGE_TOPIC,
                loader=RosImageLoader(),
                callback=self._on_shm_image,
            )
        else:
            raise ValueError(f'unknown benchmark mode: {mode}')

    def complete(self) -> bool:
        if len(self.latencies_ms) >= self._frame_count:
            return True
        if self.first_receive_time is None:
            return False
        return (time.monotonic() - self.first_receive_time) >= self._expected_receive_window

    def _on_normal_image(self, msg: Image) -> None:
        self._record(msg.header, msg.data)

    def _on_shm_image(self, msg: Image, _meta) -> None:
        self._record(msg.header, msg.data)

    def _record(self, header, payload) -> None:
        try:
            sequence = int(header.frame_id)
        except ValueError:
            return

        now_ns = self.get_clock().now().nanoseconds
        sent_ns = Time.from_msg(header.stamp).nanoseconds
        now_monotonic = time.monotonic()
        if self.first_receive_time is None:
            self.first_receive_time = now_monotonic
        self.last_receive_time = now_monotonic
        self.latencies_ms.append((now_ns - sent_ns) / 1_000_000.0)
        if payload_matches_sequence(payload, sequence, self._payload_size):
            self.valid_frames += 1


def payload_matches_sequence(payload, sequence: int, payload_size: int) -> bool:
    if len(payload) != payload_size:
        return False
    if payload_size == 0:
        return True
    # Full-payload comparison would allocate another large bytes object per frame.
    # These deterministic sentinels keep validation meaningful without dominating the benchmark.
    offsets = {0, payload_size // 4, payload_size // 2, (payload_size * 3) // 4, payload_size - 1}
    return all(payload[offset] == ((sequence + offset) % 251) for offset in offsets)


def make_container(mode: str, frame_count: int, payload_size: int, rate_hz: float):
    nodes = [
        ComposableNode(
            package='shm_sensor_transport_test',
            plugin='shm_sensor_transport_test::BenchmarkImagePublisherComponent',
            name=f'{mode}_benchmark_image_publisher',
            parameters=[{
                'topic': IMAGE_TOPIC,
                'frames': frame_count,
                'payload_size': payload_size,
                'rate_hz': rate_hz,
                'start_delay_sec': 1.5,
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        )
    ]
    if mode == 'shm':
        nodes.append(
            ComposableNode(
                package='shm_sensor_transport',
                plugin='shm_sensor_transport::ShmImageRelayComponent',
                name='benchmark_shm_image_relay',
                parameters=[{
                    'common.input_topic': IMAGE_TOPIC,
                    'common.shm_name': '/ros2_shm_benchmark_image',
                    'common.slot_count': 8,
                    'common.slot_size_bytes': payload_size,
                    'common.publish_status': False,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        )

    return ComposableNodeContainer(
        name=f'{mode}_benchmark_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=nodes,
        output='screen',
    )


def sample_resources() -> ResourceSample:
    self_usage = resource.getrusage(resource.RUSAGE_SELF)
    child_usage = resource.getrusage(resource.RUSAGE_CHILDREN)
    return ResourceSample(
        monotonic=time.monotonic(),
        user_seconds=self_usage.ru_utime + child_usage.ru_utime,
        system_seconds=self_usage.ru_stime + child_usage.ru_stime,
        self_max_rss_kib=self_usage.ru_maxrss,
        children_max_rss_kib=child_usage.ru_maxrss,
    )


def diff_resources(before: ResourceSample, after: ResourceSample) -> PhaseMetrics:
    wall_seconds = max(0.000001, after.monotonic - before.monotonic)
    cpu_seconds = (
        (after.user_seconds - before.user_seconds) +
        (after.system_seconds - before.system_seconds)
    )
    return PhaseMetrics(
        wall_seconds=wall_seconds,
        cpu_seconds=cpu_seconds,
        cpu_percent=(cpu_seconds / wall_seconds) * 100.0,
        self_max_rss_kib=after.self_max_rss_kib,
        children_max_rss_kib=after.children_max_rss_kib,
    )


def run_launch_phase(
  container,
  subscriber: BenchmarkSubscriber,
  timeout: float,
) -> PhaseMetrics:
    launch_service = launch.LaunchService()
    launch_service.include_launch_description(launch.LaunchDescription([container]))

    executor = SingleThreadedExecutor()
    executor.add_node(subscriber)
    done = threading.Event()
    before = sample_resources()

    def spin_until_done() -> None:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline and not subscriber.complete():
            executor.spin_once(timeout_sec=0.02)
        done.set()
        launch_service.shutdown()

    spin_thread = threading.Thread(target=spin_until_done, daemon=True)
    spin_thread.start()
    try:
        launch_service.run()
    finally:
        done.set()
        spin_thread.join(timeout=5.0)
    return diff_resources(before, sample_resources())


def summarize_phase(subscriber: BenchmarkSubscriber, metrics: PhaseMetrics) -> PhaseSummary:
    samples = subscriber.latencies_ms
    if not samples:
        return PhaseSummary(
            received_frames=0,
            valid_frames=0,
            mean_latency_ms=0.0,
            median_latency_ms=0.0,
            min_latency_ms=0.0,
            max_latency_ms=0.0,
            receive_rate_fps=0.0,
            receive_throughput_mib_s=0.0,
            wall_throughput_mib_s=0.0,
            cpu_seconds=metrics.cpu_seconds,
            wall_seconds=metrics.wall_seconds,
            cpu_percent=metrics.cpu_percent,
            self_max_rss_kib=metrics.self_max_rss_kib,
            children_max_rss_kib=metrics.children_max_rss_kib,
        )

    receive_span = 0.0
    if subscriber.first_receive_time is not None and subscriber.last_receive_time is not None:
        receive_span = max(0.000001, subscriber.last_receive_time - subscriber.first_receive_time)
    mib_received = (len(samples) * subscriber._payload_size) / (1024.0 * 1024.0)
    receive_mib_per_s = mib_received / receive_span if receive_span > 0.0 else 0.0
    wall_mib_per_s = mib_received / metrics.wall_seconds
    receive_fps = len(samples) / receive_span if receive_span > 0.0 else 0.0
    return PhaseSummary(
        received_frames=len(samples),
        valid_frames=subscriber.valid_frames,
        mean_latency_ms=statistics.mean(samples),
        median_latency_ms=statistics.median(samples),
        min_latency_ms=min(samples),
        max_latency_ms=max(samples),
        receive_rate_fps=receive_fps,
        receive_throughput_mib_s=receive_mib_per_s,
        wall_throughput_mib_s=wall_mib_per_s,
        cpu_seconds=metrics.cpu_seconds,
        wall_seconds=metrics.wall_seconds,
        cpu_percent=metrics.cpu_percent,
        self_max_rss_kib=metrics.self_max_rss_kib,
        children_max_rss_kib=metrics.children_max_rss_kib,
    )


def print_summary_table(normal: PhaseSummary, shm: PhaseSummary) -> None:
    rows = [
        ('Received frames', f'{normal.received_frames}', f'{shm.received_frames}'),
        (
            'Valid frames',
            f'{normal.valid_frames} / {normal.received_frames}',
            f'{shm.valid_frames} / {shm.received_frames}',
        ),
        ('Mean latency', f'{normal.mean_latency_ms:.3f} ms', f'{shm.mean_latency_ms:.3f} ms'),
        (
            'Median latency',
            f'{normal.median_latency_ms:.3f} ms',
            f'{shm.median_latency_ms:.3f} ms',
        ),
        ('Min latency', f'{normal.min_latency_ms:.3f} ms', f'{shm.min_latency_ms:.3f} ms'),
        ('Max latency', f'{normal.max_latency_ms:.3f} ms', f'{shm.max_latency_ms:.3f} ms'),
        ('Receive rate', f'{normal.receive_rate_fps:.1f} fps', f'{shm.receive_rate_fps:.1f} fps'),
        (
            'Receive throughput',
            f'{normal.receive_throughput_mib_s:.1f} MiB/s',
            f'{shm.receive_throughput_mib_s:.1f} MiB/s',
        ),
        (
            'Wall throughput',
            f'{normal.wall_throughput_mib_s:.1f} MiB/s',
            f'{shm.wall_throughput_mib_s:.1f} MiB/s',
        ),
        (
            'CPU time / wall time',
            f'{normal.cpu_seconds:.3f} s / {normal.wall_seconds:.3f} s',
            f'{shm.cpu_seconds:.3f} s / {shm.wall_seconds:.3f} s',
        ),
        ('CPU percent', f'{normal.cpu_percent:.1f}%', f'{shm.cpu_percent:.1f}%'),
        (
            'Max RSS, benchmark process',
            f'{normal.self_max_rss_kib} KiB',
            f'{shm.self_max_rss_kib} KiB',
        ),
        (
            'Max RSS, child component process',
            f'{normal.children_max_rss_kib} KiB',
            f'{shm.children_max_rss_kib} KiB',
        ),
    ]

    print()
    print('| Metric | Normal Python Image subscriber | Python ShmSubscriber |')
    print('| --- | ---: | ---: |')
    for metric, normal_value, shm_value in rows:
        print(f'| {metric} | {normal_value} | {shm_value} |')
    print()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--frames', type=int, default=120)
    parser.add_argument('--payload-size', type=int, default=1024 * 1024)
    parser.add_argument('--rate-hz', type=float, default=120.0)
    parser.add_argument('--timeout', type=float, default=20.0)
    args = parser.parse_args()

    os.environ.setdefault('ROS_LOG_DIR', '/tmp/shm_sensor_transport_benchmark_logs')
    rclpy.init()
    try:
        normal_sub = BenchmarkSubscriber('normal', args.frames, args.payload_size, args.rate_hz)
        normal_metrics = run_launch_phase(
            make_container('normal', args.frames, args.payload_size, args.rate_hz),
            normal_sub,
            args.timeout,
        )
        normal_summary = summarize_phase(normal_sub, normal_metrics)
        normal_sub.destroy_node()

        shm_sub = BenchmarkSubscriber('shm', args.frames, args.payload_size, args.rate_hz)
        shm_metrics = run_launch_phase(
            make_container('shm', args.frames, args.payload_size, args.rate_hz),
            shm_sub,
            args.timeout,
        )
        shm_summary = summarize_phase(shm_sub, shm_metrics)
        shm_sub.destroy_node()
        print_summary_table(normal_summary, shm_summary)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
