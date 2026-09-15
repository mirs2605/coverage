"""Coverage path generation (ROS-free pure functions).

入力: 4点(以上)の多角形 [(x, y), ...] (mapフレーム, メートル)
出力: boustrophedon(往復)走査のウェイポイント [(x, y, yaw), ...]

ROSへの依存を持たせないことで、ホスト側のpytestで検証できる。
コストマップの当たり判定は呼び出し側(executor)がOccupancyGridを
一次元の参照関数に落として渡す。
"""

from __future__ import annotations

import math
from typing import Callable, Sequence

Point2D = tuple[float, float]
Waypoint = tuple[float, float, float]

#: このコスト以上のセルは走行不可とみなす (OccupancyGrid: 0-100)
DEFAULT_LETHAL_COST = 80


def signed_area(polygon: Sequence[Point2D]) -> float:
    """符号付き面積 (Shoelace)。正=CCW、負=CW。"""
    area = 0.0
    n = len(polygon)
    for i in range(n):
        x0, y0 = polygon[i]
        x1, y1 = polygon[(i + 1) % n]
        area += x0 * y1 - x1 * y0
    return area / 2.0


def normalize_order(polygon: Sequence[Point2D]) -> list[Point2D]:
    """頂点を重心周りの偏角でソートし、凸包順に正規化する。

    アプリ/ble_server側は自己交差なしを保証するが、送信順序までは
    保証しないため、ここで巻き順を揃える。4点の凸四角形を想定し、
    凹・凸どちらの入力にも破綻しない。
    """
    pts = [(float(x), float(y)) for x, y in polygon]
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)
    return sorted(pts, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))


def point_in_polygon(x: float, y: float, polygon: Sequence[Point2D]) -> bool:
    """レイキャスティングによる内外判定。辺上は内側扱い。"""
    inside = False
    n = len(polygon)
    for i in range(n):
        x0, y0 = polygon[i]
        x1, y1 = polygon[(i + 1) % n]
        if ((y0 > y) != (y1 > y)) and (x < (x1 - x0) * (y - y0) / (y1 - y0) + x0):
            inside = not inside
    return inside


def _segments_intersection(
    a: Point2D, b: Point2D, y: float
) -> list[float]:
    """水平走査線 y と線分abの交点x座標(0または1個)を返す。"""
    (x0, y0), (x1, y1) = a, b
    if (y0 > y) == (y1 > y):
        return []
    if y1 == y0:
        return []
    t = (y - y0) / (y1 - y0)
    return [x0 + t * (x1 - x0)]


def _scanline_intervals(
    polygon: Sequence[Point2D], y: float
) -> list[tuple[float, float]]:
    """走査線yと多角形の交差区間 [(x_min, x_max), ...] を返す。"""
    xs: list[float] = []
    n = len(polygon)
    for i in range(n):
        xs.extend(_segments_intersection(polygon[i], polygon[(i + 1) % n], y))
    xs.sort()
    # 偶数個になるはず。奇数の場合は末尾を落とす(数値誤差対策)
    if len(xs) % 2 == 1:
        xs = xs[:-1]
    return [(xs[i], xs[i + 1]) for i in range(0, len(xs), 2)]


def generate_boustrophedon(
    polygon: Sequence[Point2D],
    spacing: float = 0.35,
) -> list[Waypoint]:
    """多角形内を往復走査するウェイポイント列を生成する。

    最小面積外接矩形の厳密解ではなく、Y方向のバウンディングボックスに
    対する水平スキャンで近似する。4点の清掃範囲では十分であり、
    凹多角形にも対応できる。yawは進行方向(±X)に付与する。
    """
    if len(polygon) < 3:
        raise ValueError("polygon must have at least 3 vertices")
    if spacing <= 0:
        raise ValueError("spacing must be positive")

    poly = normalize_order(polygon)
    min_y = min(p[1] for p in poly)
    max_y = max(p[1] for p in poly)

    waypoints: list[Waypoint] = []
    forward = True
    # 走査線を面積内に収めるため半間隔ずらして開始
    y = min_y + spacing / 2.0
    while y <= max_y:
        for x_min, x_max in _scanline_intervals(poly, y):
            if x_max - x_min < 1e-6:
                continue
            if forward:
                waypoints.append((x_min, y, 0.0))
                waypoints.append((x_max, y, 0.0))
            else:
                waypoints.append((x_max, y, math.pi))
                waypoints.append((x_min, y, math.pi))
            forward = not forward
        y += spacing
    return waypoints


def interpolate_waypoints(
    waypoints: Sequence[Waypoint],
    resolution: float = 0.1,
) -> list[Waypoint]:
    """ウェイポイント間を直線補間し、密な候補点列にする。"""
    if resolution <= 0:
        raise ValueError("resolution must be positive")
    if not waypoints:
        return []
    dense: list[Waypoint] = [waypoints[0]]
    for (x0, y0, yaw0), (x1, y1, yaw1) in zip(waypoints, waypoints[1:]):
        dist = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(math.ceil(dist / resolution)))
        yaw = yaw0 if abs(yaw0 - yaw1) < 1e-9 else yaw0
        for s in range(1, steps + 1):
            t = s / steps
            dense.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, yaw))
    return dense


def filter_by_cost(
    waypoints: Sequence[Waypoint],
    cost_at: Callable[[float, float], int],
    lethal_cost: int = DEFAULT_LETHAL_COST,
    min_segment_length: float = 0.5,
    max_waypoints: int = 100,
) -> list[Waypoint]:
    """コストマップ参照で走行不可点を除外し、連続区間のみ残す。

    cost_at(x, y): OccupancyGrid相当の参照関数。-1=unknown、0-100=cost。
    unknownは走行不可扱い(allow_unknown=False相当)。
    短い区間・孤立点は捨て、Nav2に投げる数を上限で打ち切る。
    """
    segments: list[list[Waypoint]] = []
    current: list[Waypoint] = []
    for wp in waypoints:
        cost = cost_at(wp[0], wp[1])
        if cost < 0 or cost >= lethal_cost:
            if current:
                segments.append(current)
                current = []
            continue
        current.append(wp)
    if current:
        segments.append(current)

    kept: list[Waypoint] = []
    for seg in segments:
        length = sum(
            math.hypot(seg[i + 1][0] - seg[i][0], seg[i + 1][1] - seg[i][1])
            for i in range(len(seg) - 1)
        )
        if length < min_segment_length and len(seg) < 2:
            continue
        if length < min_segment_length:
            continue
        kept.extend(seg)
        if len(kept) >= max_waypoints:
            break
    return kept[:max_waypoints]


def plan_coverage(
    polygon: Sequence[Point2D],
    cost_at: Callable[[float, float], int],
    spacing: float = 0.35,
    resolution: float = 0.1,
    lethal_cost: int = DEFAULT_LETHAL_COST,
) -> list[Waypoint]:
    """生成→補間→コストフィルタの一括実行。executorから呼ぶ想定。"""
    raw = generate_boustrophedon(polygon, spacing=spacing)
    dense = interpolate_waypoints(raw, resolution=resolution)
    return filter_by_cost(dense, cost_at, lethal_cost=lethal_cost)
