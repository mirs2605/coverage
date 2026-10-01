"""清掃ゾーン内セルの地図占有率をROS非依存で求める。

``coverage_bridge`` が ``/map`` (OccupancyGrid) 受信時に使い、
静的障害物と重なるゾーンの実行要求を事前に拒否する。
座標単位は [m]、セル値は ``-1: unknown、0: free、1..100: occupied度``。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

Point2D = tuple[float, float]

OCCUPIED_THRESHOLD = 65
"""この値以上を占有セルとみなす。地図のoccupied_thresh(0.65)に合わせる。"""


@dataclass(frozen=True)
class ZoneOccupancy:
    """ゾーン内セルの内訳。"""

    inside_cells: int
    """中心がゾーン内にあるセル数。"""

    occupied_ratio: float
    """占有セルの割合 [0.0, 1.0]。"""

    unknown_ratio: float
    """unknownセルの割合 [0.0, 1.0]。unknownは許容し拒否理由にしない。"""


def point_in_polygon(x: float, y: float, polygon: Sequence[Point2D]) -> bool:
    """レイキャスト法で点の内外を判定する。辺上は内外いずれか（実装依存）。"""
    inside = False
    n = len(polygon)
    for i in range(n):
        x0, y0 = polygon[i]
        x1, y1 = polygon[(i + 1) % n]
        if (y0 > y) != (y1 > y):
            x_cross = x0 + (y - y0) * (x1 - x0) / (y1 - y0)
            if x < x_cross:
                inside = not inside
    return inside


def zone_occupancy(
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    data: Sequence[int],
    polygon: Sequence[Point2D],
) -> ZoneOccupancy:
    """ゾーン内セルを数え、占有率とunknown率を返す。

    Args:
        width: 地図の幅 [cell]。
        height: 地図の高さ [cell]。
        resolution: 解像度 [m/cell]。
        origin_x: 地図原点x [m]。
        origin_y: 地図原点y [m]。
        data: row-majorのセル値列。
        polygon: ゾーン頂点列 [m]（開路・閉路どちらも可）。

    Returns:
        ゾーン内セルが無い場合は全て0の :class:`ZoneOccupancy`。
    """
    if not polygon or resolution <= 0:
        return ZoneOccupancy(0, 0.0, 0.0)
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    cx0 = max(0, int((min(xs) - origin_x) / resolution))
    cx1 = min(width - 1, int((max(xs) - origin_x) / resolution))
    cy0 = max(0, int((min(ys) - origin_y) / resolution))
    cy1 = min(height - 1, int((max(ys) - origin_y) / resolution))

    inside = occupied = unknown = 0
    for cy in range(cy0, cy1 + 1):
        for cx in range(cx0, cx1 + 1):
            x = origin_x + (cx + 0.5) * resolution
            y = origin_y + (cy + 0.5) * resolution
            if not point_in_polygon(x, y, polygon):
                continue
            inside += 1
            v = data[cy * width + cx]
            if v < 0:
                unknown += 1
            elif v >= OCCUPIED_THRESHOLD:
                occupied += 1
    if inside == 0:
        return ZoneOccupancy(0, 0.0, 0.0)
    return ZoneOccupancy(
        inside_cells=inside,
        occupied_ratio=occupied / inside,
        unknown_ratio=unknown / inside,
    )
