"""Zone polygon normalization (ROS-free pure functions).

``/cleaning_zone`` (geometry_msgs/PolygonStamped) の点列を、
opennav_coverage_msgs ``NavigateCompleteCoverage`` ゴールの
``polygons`` (geometry_msgs/Polygon[]) に積める形に整える。

ROSメッセージ型には触れない。ROS依存なしでpytestできる層。
実際の Point32/Polygon 組み立ては coverage_bridge 側で行う。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

Point2D = tuple[float, float]


@dataclass(frozen=True)
class NormalizedZone:
    """検証済みで不変な清掃ゾーン。

    ROSメッセージをcontrol flowに保持しないための値オブジェクト。入力メッセージが
    閉路表現でも、``points`` は始点を一度だけ持つ正規化済みの点列になる。
    """

    frame_id: str
    points: tuple[Point2D, ...]

    @classmethod
    def from_points(
        cls,
        frame_id: str,
        points: Sequence[Sequence[float]],
        min_vertices: int = 3,
    ) -> 'NormalizedZone':
        return cls(frame_id=frame_id, points=tuple(
            normalize_zone(points, min_vertices=min_vertices)
        ))


def normalize_zone(
    points: Sequence[Sequence[float]],
    min_vertices: int = 3,
) -> list[Point2D]:
    """清掃ゾーン点列を検証・正規化する。

    - 末尾の始点重複(閉路表現)を除去する
    - min_vertices未満・非有限値(x/yがNaN・inf)は ValueError
    - 巻き順の正規化はしない。Fields2Coverが面倒を見る
    """
    try:
        pts = [(float(p[0]), float(p[1])) for p in points]
    except (IndexError, TypeError, ValueError) as exc:
        raise ValueError('zone points must contain numeric x/y coordinates') from exc
    if len(pts) >= 2 and pts[0] == pts[-1]:
        pts = pts[:-1]
    if len(pts) < min_vertices:
        raise ValueError(
            f'zone must have at least {min_vertices} vertices, got {len(pts)}'
        )
    for x, y in pts:
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError(f'zone contains non-finite coordinate: {(x, y)}')
    return pts
