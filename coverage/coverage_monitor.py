#!/usr/bin/env python3
"""清掃ナビゲーションの入力、計画、速度指令とTFを診断情報へ集約する。"""

from __future__ import annotations

import time
from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PolygonStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


# 最大30sの実時間サンプルからHzを求める。経路と速度指令はイベント型である。
TOPICS = {
    '/cleaning_zone': (PolygonStamped, None),
    '/map': (OccupancyGrid, None),
    '/scan': (LaserScan, 2.0),
    '/coverage_server/coverage_plan': (Path, None),
    '/plan': (Path, None),
    '/cmd_vel_nav': (Twist, None),
    '/cmd_vel_smoothed': (Twist, None),
    '/cmd_vel': (Twist, None),
}
THREE_D_TOPICS = {
    '/scan_raw': (LaserScan, 2.0),
    '/odom': (Odometry, 2.0),
    '/imu/data_raw': (Imu, 2.0),
    '/wheel/odometry': (Odometry, 2.0),
    '/imu/data_fusion': (Imu, 2.0),
    '/points': (PointCloud2, 2.0),
    '/points_icp_filtered': (PointCloud2, 2.0),
    '/points_assembled_icp': (PointCloud2, 15.0),
    '/odometry/filtered': (Odometry, 2.0),
}
STAMP_TOPICS = {'/scan', '/scan_raw', '/points', '/points_icp_filtered',
                '/ground_segmentation/obstacle_points',
                '/points_assembled_icp', '/odometry/filtered',
                '/localization_pose', '/odom', '/imu/data_raw',
                '/wheel/odometry', '/imu/data_fusion'}
SENSOR_STAGES = ('scan', 'cloud', 'filtered', 'assembled', 'localization')


class CoverageMonitor(Node):
    """主要トピックの実時間Hz、最終受信からの秒数、TFを発行する。"""

    def __init__(self) -> None:
        super().__init__('coverage_monitor')
        self.declare_parameter('profile', '2d')
        self.declare_parameter('zone_topic', '/cleaning_zone')
        self.declare_parameter('diagnostic_stage', 'scan')
        profile = self.get_parameter('profile').value
        self._sensor_profile = profile == 'sensor'
        self._expect_clock = profile in ('2d', '3d', 'sensor')
        zone_topic = self.get_parameter('zone_topic').value
        self._tf_target = 'map' if profile != 'sensor' else None
        if profile == 'sensor':
            stage = self.get_parameter('diagnostic_stage').value
            if stage not in SENSOR_STAGES:
                raise ValueError(f'Unknown diagnostic_stage: {stage}')
            topics = {'/scan_raw': THREE_D_TOPICS['/scan_raw'],
                      '/scan': TOPICS['/scan']}
            if SENSOR_STAGES.index(stage) >= SENSOR_STAGES.index('cloud'):
                topics.update({
                    '/points': THREE_D_TOPICS['/points'],
                    '/odom': THREE_D_TOPICS['/odom'],
                    '/imu/data_raw': THREE_D_TOPICS['/imu/data_raw'],
                    '/wheel/odometry': THREE_D_TOPICS['/wheel/odometry'],
                    '/imu/data_fusion': THREE_D_TOPICS['/imu/data_fusion'],
                    '/odometry/filtered': THREE_D_TOPICS['/odometry/filtered'],
                })
                self._tf_target = 'odom'
            if SENSOR_STAGES.index(stage) >= SENSOR_STAGES.index('filtered'):
                topics['/points_icp_filtered'] = THREE_D_TOPICS['/points_icp_filtered']
            if SENSOR_STAGES.index(stage) >= SENSOR_STAGES.index('assembled'):
                topics['/points_assembled_icp'] = THREE_D_TOPICS['/points_assembled_icp']
            if stage == 'localization':
                topics['/localization_pose'] = (PoseWithCovarianceStamped, 15.0)
                topics['/map'] = (OccupancyGrid, None)
                self._tf_target = 'map'
        else:
            topics = dict(TOPICS)
            topics[zone_topic] = topics.pop('/cleaning_zone')
            if profile == '3d':
                topics.update(THREE_D_TOPICS)
        # 3D走行と位置推定診断は、GSeg3Dの非地面点群を照合に使う。
        if profile == '3d' or (profile == 'sensor' and stage == 'localization'):
            topics.pop('/points_icp_filtered', None)
            topics.pop('/points_assembled_icp', None)
            topics['/ground_segmentation/obstacle_points'] = (PointCloud2, 15.0)
        self._samples = {topic: deque(maxlen=600) for topic in topics}
        self._last_seen = {topic: None for topic in topics}
        self._thresholds = {topic: threshold for topic, (_, threshold) in topics.items()}
        self._received_since_zone = {topic: 0 for topic in topics}
        self._last_stamp_ns = {topic: None for topic in topics}
        self._repeated_stamps = {topic: 0 for topic in topics}
        self._backward_stamps = {topic: 0 for topic in topics}
        self._last_stamp_problem = {topic: None for topic in topics}
        self._last_point_count = {topic: None for topic in topics}
        self._last_scan_timing = {topic: None for topic in topics}
        self._zone_topic = zone_topic
        self._zone_received = False
        self._phase = 'IDLE'
        self._clock_sample: tuple[float, float] | None = None
        self._clock_last_seen: float | None = None
        self._sim_rate: float | None = None
        self._clock_backward_events = 0

        # BEST_EFFORT購読はRELIABLE/BEST_EFFORTの両publisherから受信できる。
        data_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        for topic, (msg_type, _) in topics.items():
            self.create_subscription(
                msg_type, topic,
                lambda msg, name=topic: self._on_topic(name, msg), data_qos)
        phase_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, '/coverage/phase', self._on_phase, phase_qos)
        if self._expect_clock:
            self.create_subscription(
                ClockMessage, '/clock', self._on_clock, data_qos)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        # Gazebo停止中も最終受信時刻を更新するため実時間1.0s周期を使う。
        self.create_timer(
            1.0, self._publish_diagnostics,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _on_topic(self, topic: str, msg) -> None:
        """受信時刻、ヘッダー時刻の重複、点数を記録する。"""
        now = time.monotonic()
        self._samples[topic].append(now)
        self._last_seen[topic] = now
        if topic in STAMP_TOPICS:
            stamp = msg.header.stamp
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            previous = self._last_stamp_ns[topic]
            if previous is not None and stamp_ns == previous:
                self._repeated_stamps[topic] += 1
                self._last_stamp_problem[topic] = now
            elif previous is not None and stamp_ns < previous:
                self._backward_stamps[topic] += 1
                self._last_stamp_problem[topic] = now
            self._last_stamp_ns[topic] = stamp_ns
        if isinstance(msg, PointCloud2):
            self._last_point_count[topic] = msg.width * msg.height
        elif isinstance(msg, LaserScan):
            self._last_scan_timing[topic] = (msg.time_increment, msg.scan_time)
        if topic == self._zone_topic:
            self._zone_received = True
            self._received_since_zone = {
                name: 0 for name in self._received_since_zone}
        self._received_since_zone[topic] += 1

    def _on_phase(self, msg) -> None:
        """bridgeが発行した処理段階を保持する。"""
        self._phase = msg.data

    def _on_clock(self, msg: ClockMessage) -> None:
        """シミュレーション時間と実時間の比を計算する。"""
        now = time.monotonic()
        self._clock_last_seen = now
        sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self._clock_sample is not None:
            previous_wall, previous_sim = self._clock_sample
            wall_delta = now - previous_wall
            sim_delta = sim - previous_sim
            if wall_delta >= 0.5 and sim_delta >= 0:
                self._sim_rate = sim_delta / wall_delta
                self._clock_sample = (now, sim)
            elif sim_delta < 0:
                self._sim_rate = None
                self._clock_backward_events += 1
                self._clock_sample = (now, sim)
        else:
            self._clock_sample = (now, sim)

    def _topic_status(self, topic: str, now: float) -> DiagnosticStatus:
        """一つのtopicについて実時間Hzと受信停止を表す。"""
        samples = self._samples[topic]
        while samples and now - samples[0] > 30.0:
            samples.popleft()
        last_seen = self._last_seen[topic]
        age = now - last_seen if last_seen is not None else None
        threshold = self._thresholds[topic]
        publishers = self.count_publishers(topic)
        hz = (
            (len(samples) - 1) / (samples[-1] - samples[0])
            if len(samples) >= 2 and samples[-1] > samples[0] else 0.0)
        status = DiagnosticStatus()
        status.name = f'coverage/topic{topic}'
        status.hardware_id = 'coverage'
        if publishers == 0:
            status.level = DiagnosticStatus.WARN
            status.message = 'NO_PUBLISHER'
        elif topic in STAMP_TOPICS and publishers > 1:
            status.level = DiagnosticStatus.WARN
            status.message = 'MULTIPLE_PUBLISHERS'
        elif age is None:
            status.level = (
                DiagnosticStatus.WARN if threshold is not None
                else DiagnosticStatus.OK)
            status.message = 'NO_MESSAGE'
        elif threshold is not None and age > threshold:
            status.level = DiagnosticStatus.WARN
            status.message = 'STALE'
        elif (self._last_stamp_problem[topic] is not None and
              now - self._last_stamp_problem[topic] <= 30.0):
            status.level = DiagnosticStatus.WARN
            status.message = 'NON_MONOTONIC_STAMP'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'RECEIVING' if threshold is not None else 'OBSERVED'
        status.values = [
            KeyValue(key='hz_wall', value=f'{hz:.3f}'),
            KeyValue(key='publisher_count', value=str(publishers)),
            KeyValue(key='age_wall_sec', value=f'{age:.1f}' if age is not None else 'never'),
            KeyValue(key='messages_since_zone', value=str(
                self._received_since_zone[topic]) if self._zone_received else 'no_zone'),
        ]
        if self._last_stamp_ns[topic] is not None:
            status.values.extend([
                KeyValue(key='last_stamp_ns', value=str(self._last_stamp_ns[topic])),
                KeyValue(key='repeated_stamps', value=str(self._repeated_stamps[topic])),
                KeyValue(key='backward_stamps', value=str(self._backward_stamps[topic])),
            ])
        if self._last_point_count[topic] is not None:
            status.values.append(KeyValue(
                key='last_point_count', value=str(self._last_point_count[topic])))
        if self._last_scan_timing[topic] is not None:
            increment, duration = self._last_scan_timing[topic]
            status.values.extend([
                KeyValue(key='time_increment_sec', value=f'{increment:.9f}'),
                KeyValue(key='scan_time_sec', value=f'{duration:.9f}'),
            ])
        return status

    def _publish_diagnostics(self) -> None:
        """Foxglove Diagnosticsパネル向けに各監視項目を1.0s周期で発行する。"""
        now = time.monotonic()
        statuses = [self._topic_status(topic, now) for topic in self._samples]
        if self._tf_target is not None:
            tf_status = DiagnosticStatus()
            tf_status.name = f'coverage/tf/{self._tf_target}_to_base_footprint'
            tf_status.hardware_id = 'coverage'
            available = self._tf_buffer.can_transform(
                self._tf_target, 'base_footprint', Time())
            tf_status.level = DiagnosticStatus.OK if available else DiagnosticStatus.WARN
            tf_status.message = 'AVAILABLE' if available else 'UNAVAILABLE'
            statuses.append(tf_status)
        if self._expect_clock:
            clock_status = DiagnosticStatus()
            clock_status.name = 'coverage/simulation_clock'
            clock_status.hardware_id = 'coverage'
            clock_age = (
                now - self._clock_last_seen
                if self._clock_last_seen is not None else None)
            clock_status.level = (
                DiagnosticStatus.OK if clock_age is not None and clock_age <= 2.0
                else DiagnosticStatus.WARN)
            clock_status.message = (
                'RECEIVING' if clock_age is not None and clock_age <= 2.0
                else 'NO_MESSAGE' if clock_age is None else 'STALE')
            clock_status.values = [
                KeyValue(
                    key='age_wall_sec',
                    value=f'{clock_age:.1f}' if clock_age is not None else 'never'),
                KeyValue(
                    key='simulation_speed',
                    value=f'{self._sim_rate:.2f}x'
                    if self._sim_rate is not None else 'unknown'),
                KeyValue(
                    key='backward_clock_events',
                    value=str(self._clock_backward_events)),
            ]
            statuses.append(clock_status)
        if not self._sensor_profile:
            flow = DiagnosticStatus()
            flow.name = 'coverage/flow'
            flow.hardware_id = 'coverage'
            flow.level = DiagnosticStatus.OK
            flow.message = self._phase
            flow.values = [KeyValue(key='bridge_phase', value=self._phase)]
            if self._sim_rate is not None:
                flow.values.append(KeyValue(
                    key='simulation_speed', value=f'{self._sim_rate:.2f}x'))
            statuses.append(flow)
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = statuses
        self._diagnostics_pub.publish(msg)


def main(args=None) -> None:
    """coverage_monitorをROS 2ノードとして実行する。"""
    rclpy.init(args=args)
    node = CoverageMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
