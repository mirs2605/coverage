"""清掃ゾーンの点列をROSメッセージに依存せず検証・正規化する。

入力点列は ``/cleaning_zone`` のPolygonStamped由来であり、正規化後は
NavigateCompleteCoverageゴールへ渡す。ROSメッセージの組み立てはbridgeが担当する。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

Point2D = tuple[float, float]


@dataclass(frozen=True)
class NormalizedZone:
    """frame_idと検証済み頂点列を保持する不変値。

    閉路表現で末尾に始点が重複していても、``points`` には始点を一度だけ保持する。
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
        """座標列を検証し、閉路終端を除いた不変ゾーンを生成する。"""
        return cls(frame_id=frame_id, points=tuple(
            normalize_zone(points, min_vertices=min_vertices)
        ))


def normalize_zone(
    points: Sequence[Sequence[float]],
    min_vertices: int = 3,
) -> list[Point2D]:
    """清掃ゾーンの頂点列を数値座標へ変換して検証する。

    閉路表現の末尾重複点は除去する。頂点数不足、数値変換不能、または
    非有限なx/y座標がある場合はValueErrorを送出する。頂点順序は維持する。

    Args:
        points: 各要素がx/yを持つ頂点列。座標単位は[m]。
        min_vertices: 正規化後に必要な最小頂点数。

    Returns:
        入力順を保った正規化済みのx/y座標列。
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
