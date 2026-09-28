#!/usr/bin/env python3
"""cleaning_zoneをNavigateCompleteCoverageアクションへ中継するROS 2ノード。

入力ポリゴンの検証、Nav2ゴール送信、進捗と終了状態の通知を行う。
経路生成と追従はNav2のcoverage_serverとCoverageNavigatorが担当する。
"""

from __future__ import annotations

import functools
import threading
import time
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, Polygon, PolygonStamped
from opennav_coverage_msgs.action import NavigateCompleteCoverage
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from coverage.zone_field import NormalizedZone


def _coerce_float(value: Any, default: float) -> float:
    """ROSパラメータ値をfloatへ変換し、変換不能時は既定値を返す。"""
    try:
        if isinstance(value, str):
            value = value.strip().strip('"').strip("'")
        return float(value)
    except (TypeError, ValueError):
        return default


def _coerce_int(value: Any, default: int) -> int:
    """ROSパラメータ値を整数へ変換し、変換不能時は既定値を返す。"""
    try:
        if isinstance(value, str):
            value = value.strip().strip('"').strip("'")
        return int(float(value))
    except (TypeError, ValueError):
        return default


def _coerce_str(value: Any, default: str) -> str:
    """ROSパラメータ値を文字列へ変換し、Noneには既定値を返す。"""
    if value is None:
        return default
    if isinstance(value, str):
        return value
    return str(value)


class CoverageBridge(Node):
    """清掃ゾーンのNav2実行要求と状態通知を管理する。

    ``/cleaning_zone`` の最新値を送信し、サーバ不在やゴール拒否時は再試行する。
    新しいゾーンの受信時は実行中ゴールをキャンセルして最新値を優先する。
    """

    def __init__(self) -> None:
        """topic、action client、再試行タイマーを初期化する。"""
        super().__init__('coverage_bridge')
        self.declare_parameter('zone_topic', '/cleaning_zone')
        self.declare_parameter('state_topic', '/coverage/state')
        self.declare_parameter('action_name', 'navigate_complete_coverage')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('behavior_tree', '')
        self.declare_parameter('min_vertices', 3)
        self.declare_parameter('nav_timeout_sec', 5.0)
        self.declare_parameter('retry_limit', 3)
        self.declare_parameter('backoff_sec', 5.0)
        # use_sim_timeはrclpyが自動宣言するため、ここでは重複して宣言しない。

        raw = {n: self.get_parameter(n).value for n in (
            'zone_topic', 'state_topic', 'action_name', 'frame_id',
            'behavior_tree',
        )}
        self._zone_topic = _coerce_str(raw['zone_topic'], '/cleaning_zone')
        self._state_topic = _coerce_str(raw['state_topic'], '/coverage/state')
        self._action_name = _coerce_str(
            raw['action_name'], 'navigate_complete_coverage')
        self._default_frame = _coerce_str(raw['frame_id'], 'map')
        self._behavior_tree = _coerce_str(raw['behavior_tree'], '')

        self._pending_zone: NormalizedZone | None = None
        self._lock = threading.Lock()
        self._goal_handle = None
        self._executing = False
        # 最新世代以外の非同期応答を破棄し、goal切替後の状態上書きを防ぐ。
        self._generation = 0
        self._reject_count = 0
        self._next_retry_time = 0.0

        # state通知はQoS depth 10でpublishする。
        self._state_pub = self.create_publisher(String, self._state_topic, 10)
        # 入力ゾーンをQoS depth 10で受信する。
        self.create_subscription(
            PolygonStamped, self._zone_topic, self._on_zone, 10
        )
        self._nav_client: ActionClient = ActionClient(
            self, NavigateCompleteCoverage, self._action_name
        )
        # 保留ゾーンを1.0s周期で再送可能か確認する。
        self.create_timer(1.0, self._on_retry_timer)

        self.get_logger().info(
            f'coverage_bridge started: zone={self._zone_topic} '
            f'action={self._action_name}'
        )

    def _float_param(self, name: str, default: float) -> float:
        """指定パラメータをfloatとして取得する。"""
        return _coerce_float(self.get_parameter(name).value, default)

    def _int_param(self, name: str, default: int) -> int:
        """指定パラメータをintとして取得する。"""
        return _coerce_int(self.get_parameter(name).value, default)

    def _publish_state(self, text: str) -> None:
        """coverage状態文字列をstate topicへpublishする。"""
        msg = String()
        msg.data = text
        self._state_pub.publish(msg)

    def _on_zone(self, msg: PolygonStamped) -> None:
        """受信ゾーンを検証し、Nav2へ最新の実行要求を送る。"""
        min_vertices = self._int_param('min_vertices', 3)
        try:
            zone = NormalizedZone.from_points(
                frame_id=msg.header.frame_id or self._default_frame,
                points=[(p.x, p.y) for p in msg.polygon.points],
                min_vertices=min_vertices,
            )
        except ValueError as e:
            self.get_logger().error(f'不正なゾーンのため無視します: {e}')
            self._publish_state('FAILED_INVALID_ZONE')
            return
        self._cancel_current_goal()
        with self._lock:
            self._pending_zone = zone
            self._reject_count = 0
            self._next_retry_time = 0.0
        self._try_send(zone)

    def _on_retry_timer(self) -> None:
        """実行中要求がなく再試行時刻を過ぎた場合、保留ゾーンを再送する。"""
        with self._lock:
            pending = self._pending_zone
            executing = self._executing
            ready = time.monotonic() >= self._next_retry_time
        if pending is not None and not executing and ready:
            self._try_send(pending)

    def _make_goal(self, zone: NormalizedZone) -> NavigateCompleteCoverage.Goal:
        """正規化済みゾーンからCoverageNavigatorのgoalを組み立てる。"""
        goal = NavigateCompleteCoverage.Goal()
        goal.frame_id = zone.frame_id
        poly = Polygon()
        for x, y in zone.points:
            pt = Point32()
            pt.x, pt.y = x, y
            pt.z = 0.0
            poly.points.append(pt)
        goal.polygons.append(poly)
        if self._behavior_tree:
            goal.behavior_tree = self._behavior_tree
        return goal

    def _cancel_current_goal(self) -> None:
        """現在のgoalを無効化し、受理済みならNav2へcancelを要求する。"""
        with self._lock:
            self._generation += 1
            handle = self._goal_handle
            self._goal_handle = None
            self._executing = False
        if handle is not None:
            try:
                handle.cancel_goal_async()
                self.get_logger().info('実行中ゴールをキャンセルしました')
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'ゴールキャンセル失敗: {e}')

    def _try_send(self, zone: NormalizedZone) -> None:
        """action serverを待ち、ゾーンgoalを非同期で送信する。"""
        timeout = self._float_param('nav_timeout_sec', 5.0)
        if not self._nav_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().warn(
                f'{self._action_name}サーバ不在のため再試行します',
                throttle_duration_sec=10.0,
            )
            self._publish_state('WAITING_NAV_SERVER')
            return
        with self._lock:
            self._generation += 1
            gen = self._generation
            self._executing = True
        send_future = self._nav_client.send_goal_async(
            self._make_goal(zone),
            feedback_callback=functools.partial(self._on_feedback, gen),
        )
        send_future.add_done_callback(functools.partial(self._on_goal_response, gen))

    def _stale(self, gen: int) -> bool:
        """指定世代が最新でなく、結果を破棄すべきならTrueを返す。"""
        with self._lock:
            return gen != self._generation

    def _on_feedback(self, gen: int, msg) -> None:
        """最新goalの残距離とリカバリ回数を記録する。"""
        if self._stale(gen):
            return
        remaining = msg.feedback.distance_remaining
        self.get_logger().info(
            f'清掃中: 残り約{remaining:.1f}m '
            f'(recoveries={msg.feedback.number_of_recoveries})',
            throttle_duration_sec=10.0,
        )

    def _on_goal_response(self, gen: int, future) -> None:
        """goal受理結果を処理し、受理時は結果コールバックを登録する。"""
        if self._stale(gen):
            return
        try:
            handle = future.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'ゴール送信エラー: {e}')
            with self._lock:
                self._executing = False
            self._publish_state('FAILED')
            return
        if handle is None or not handle.accepted:
            self._on_rejected()
            return
        with self._lock:
            self._reject_count = 0
            self._goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(functools.partial(self._on_nav_result, gen))

    def _on_rejected(self) -> None:
        """拒否回数に応じて待機、再試行、または失敗状態へ遷移する。"""
        limit = self._int_param('retry_limit', 3)
        backoff = self._float_param('backoff_sec', 5.0)
        with self._lock:
            self._executing = False
            self._reject_count += 1
            count = self._reject_count
        if count >= limit:
            self.get_logger().error(
                f'Nav2にゴールを{count}回拒否されました。諦めます。'
                '初期位置(2D Pose Estimate)・lifecycle(bt_navigator active)・'
                'TF(map->base_footprint)を確認してください。'
            )
            self._publish_state('FAILED_REJECTED')
            with self._lock:
                self._pending_zone = None
            return
        with self._lock:
            self._next_retry_time = time.monotonic() + backoff
        self.get_logger().warn(
            f'Nav2にゴールを拒否されました({count}/{limit})。'
            f'{backoff:.0f}s後に再送します。',
            throttle_duration_sec=10.0,
        )
        self._publish_state(f'WAITING_NAV_RETRY {count}/{limit}')

    def _on_nav_result(self, gen: int, future) -> None:
        """最新goalの終了状態をDONE、CANCELED、失敗として通知する。"""
        if self._stale(gen):
            return
        with self._lock:
            self._executing = False
        try:
            status = future.result().status
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Nav2実行エラー: {e}')
            self._publish_state('FAILED')
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('清掃完了(DONE)')
            self._publish_state('DONE')
            with self._lock:
                self._pending_zone = None
                self._reject_count = 0
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('清掃キャンセル(CANCELED)')
            self._publish_state('CANCELED')
        else:
            self.get_logger().error(f'清掃失敗(status={status})')
            self._on_rejected()


def main(args=None) -> None:
    """coverage_bridgeを初期化し、ROS 2イベントを処理する。"""
    rclpy.init(args=args)
    node = CoverageBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
