#!/usr/bin/env python3
"""coverage_bridge: /cleaning_zone -> NavigateCompleteCoverage (opennav_coverage).

BLE経由で届く清掃ポリゴンを、Nav2のCoverageNavigatorアクション
(``navigate_complete_coverage``) にそのまま投げる薄いブリッジ。
経路生成(Fields2Cover)・追従・リカバリはすべてNav2側が担う。
本ノードは購読・ゴール投入・状態通知に徹する。

フロー:
    [ble_receiver_node] --/cleaning_zone--> [coverage_bridge]
        --NavigateCompleteCoverage--> [bt_navigator/CoverageNavigator]
        --ComputeCoveragePath--> [coverage_server] --nav_path--> FollowPath
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

from coverage.zone_field import normalize_zone


def _coerce_float(value: Any, default: float) -> float:
    """launch経由の文字列("0.35")も含めてfloat化する。"""
    try:
        if isinstance(value, str):
            value = value.strip().strip('"').strip("'")
        return float(value)
    except (TypeError, ValueError):
        return default


def _coerce_int(value: Any, default: int) -> int:
    try:
        if isinstance(value, str):
            value = value.strip().strip('"').strip("'")
        return int(float(value))
    except (TypeError, ValueError):
        return default


def _coerce_str(value: Any, default: str) -> str:
    if value is None:
        return default
    if isinstance(value, str):
        return value
    return str(value)


class CoverageBridge(Node):
    def __init__(self) -> None:
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
        # NOTE: use_sim_time は rclpy が自動宣言するため宣言しない。

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

        self._pending_zone: PolygonStamped | None = None
        self._lock = threading.Lock()
        self._goal_handle = None
        self._executing = False
        self._generation = 0  # 古いコールバックを無視するための世代番号
        self._reject_count = 0
        self._next_retry_time = 0.0

        self._state_pub = self.create_publisher(String, self._state_topic, 10)
        self.create_subscription(
            PolygonStamped, self._zone_topic, self._on_zone, 10
        )
        self._nav_client: ActionClient = ActionClient(
            self, NavigateCompleteCoverage, self._action_name
        )
        self.create_timer(1.0, self._on_retry_timer)

        self.get_logger().info(
            f'coverage_bridge started: zone={self._zone_topic} '
            f'action={self._action_name}'
        )

    # --- 小物 ---
    def _float_param(self, name: str, default: float) -> float:
        return _coerce_float(self.get_parameter(name).value, default)

    def _int_param(self, name: str, default: int) -> int:
        return _coerce_int(self.get_parameter(name).value, default)

    def _publish_state(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._state_pub.publish(msg)

    # --- 購読・再試行 ---
    def _on_zone(self, msg: PolygonStamped) -> None:
        try:
            normalize_zone([(p.x, p.y) for p in msg.polygon.points])
        except ValueError as e:
            self.get_logger().error(f'不正なゾーンのため無視します: {e}')
            self._publish_state('FAILED_INVALID_ZONE')
            return
        self._cancel_current_goal()
        with self._lock:
            self._pending_zone = msg
            self._reject_count = 0
            self._next_retry_time = 0.0
        self._try_send(msg)

    def _on_retry_timer(self) -> None:
        with self._lock:
            pending = self._pending_zone
            executing = self._executing
            ready = time.monotonic() >= self._next_retry_time
        if pending is not None and not executing and ready:
            self._try_send(pending)

    # --- Nav2投入 ---
    def _make_goal(self, msg: PolygonStamped) -> NavigateCompleteCoverage.Goal:
        goal = NavigateCompleteCoverage.Goal()
        goal.frame_id = msg.header.frame_id or self._default_frame
        poly = Polygon()
        for p in msg.polygon.points:
            pt = Point32()
            pt.x, pt.y = float(p.x), float(p.y)
            pt.z = 0.0
            poly.points.append(pt)
        goal.polygons.append(poly)
        if self._behavior_tree:
            goal.behavior_tree = self._behavior_tree
        return goal

    def _cancel_current_goal(self) -> None:
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

    def _try_send(self, msg: PolygonStamped) -> None:
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
            self._make_goal(msg),
            feedback_callback=functools.partial(self._on_feedback, gen),
        )
        send_future.add_done_callback(functools.partial(self._on_goal_response, gen))

    def _stale(self, gen: int) -> bool:
        with self._lock:
            return gen != self._generation

    def _on_feedback(self, gen: int, msg) -> None:
        if self._stale(gen):
            return
        remaining = msg.feedback.distance_remaining
        self.get_logger().info(
            f'清掃中: 残り約{remaining:.1f}m '
            f'(recoveries={msg.feedback.number_of_recoveries})',
            throttle_duration_sec=10.0,
        )

    def _on_goal_response(self, gen: int, future) -> None:
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
