#!/usr/bin/env python3
"""coverage_executor: /cleaning_zone -> /coverage/path -> Nav2 NavigateThroughPoses.

設計(DESIGN.md)の通り、BT自体は既存の
navigate_through_poses_w_replanning_and_recovery.xml を使い、
本ノードはウェイポイント列の生成とアクション投入に徹する。
"""

import math
import threading

import rclpy
from geometry_msgs.msg import PolygonStamped, PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from mirs_coverage.coverage_planner import plan_coverage


class CoverageExecutor(Node):
    def __init__(self) -> None:
        super().__init__('coverage_executor')
        self.declare_parameter('zone_topic', '/cleaning_zone')
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('path_topic', '/coverage/path')
        self.declare_parameter('state_topic', '/coverage/state')
        self.declare_parameter('spacing', 0.35)
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('lethal_cost', 80)
        self.declare_parameter('frame_id', 'map')

        zone_topic = self.get_parameter('zone_topic').get_parameter_value().string_value
        costmap_topic = self.get_parameter('costmap_topic').get_parameter_value().string_value
        self._path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self._state_topic = self.get_parameter('state_topic').get_parameter_value().string_value

        self._costmap: OccupancyGrid | None = None
        self._lock = threading.Lock()
        self._goal_handle = None

        self._path_pub = self.create_publisher(Path, self._path_topic, 10)
        self._state_pub = self.create_publisher(String, self._state_topic, 10)
        self.create_subscription(OccupancyGrid, costmap_topic, self._on_costmap, 10)
        self.create_subscription(PolygonStamped, zone_topic, self._on_zone, 10)

        self._nav_client: ActionClient = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses'
        )
        self.get_logger().info(
            f'coverage_executor started: zone={zone_topic} costmap={costmap_topic}'
        )

    def _publish_state(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._state_pub.publish(msg)

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._costmap = msg

    def _cost_at(self, x: float, y: float) -> int:
        """OccupancyGrid参照。unknown(-1)は-1のまま返し、範囲外も-1扱い。"""
        grid = self._costmap
        if grid is None:
            return -1
        res = grid.info.resolution
        ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
        mx = int((x - ox) / res)
        my = int((y - oy) / res)
        if not (0 <= mx < grid.info.width and 0 <= my < grid.info.height):
            return -1
        return int(grid.data[my * grid.info.width + mx])

    def _on_zone(self, msg: PolygonStamped) -> None:
        polygon = [(p.x, p.y) for p in msg.polygon.points]
        if len(polygon) < 3:
            self.get_logger().error('頂点数不足のため無視します')
            return
        if self._costmap is None:
            self.get_logger().warn('コストマップ未受信のため待機します')
            self._publish_state('WAITING_COSTMAP')
            return

        spacing = self.get_parameter('spacing').get_parameter_value().double_value
        resolution = self.get_parameter('resolution').get_parameter_value().double_value
        lethal = self.get_parameter('lethal_cost').get_parameter_value().integer_value
        frame_id = msg.header.frame_id or self.get_parameter('frame_id').get_parameter_value().string_value

        waypoints = plan_coverage(
            polygon, self._cost_at, spacing=spacing,
            resolution=resolution, lethal_cost=lethal,
        )
        if not waypoints:
            self.get_logger().error('走行可能な区間がありません(UNREACHABLE)')
            self._publish_state('FAILED_UNREACHABLE')
            return

        self._publish_path(waypoints, frame_id)
        self._publish_state(f'EXECUTING n={len(waypoints)}')
        self._send_to_nav2(waypoints, frame_id)

    def _publish_path(self, waypoints, frame_id: str) -> None:
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
        self._path_pub.publish(path)

    def _send_to_nav2(self, waypoints, frame_id: str) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateThroughPosesサーバに接続できません')
            self._publish_state('FAILED_NO_SERVER')
            return
        goal = NavigateThroughPoses.Goal()
        for x, y, yaw in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            goal.poses.append(pose)
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('Nav2にゴールを拒否されました')
            self._publish_state('FAILED_REJECTED')
            return
        self._goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        try:
            result = future.result()
            self.get_logger().info(f'Nav2完了: {result.status}')
            self._publish_state('DONE')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Nav2実行エラー: {e}')
            self._publish_state('FAILED')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoverageExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
