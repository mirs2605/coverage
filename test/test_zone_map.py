"""ゾーン内セルの地図占有率計算を検証する（ROS不要）。"""

from coverage.zone_map import ZoneOccupancy, point_in_polygon, zone_occupancy


def _grid(width, height, fill=0):
    return [fill] * (width * height)


def test_point_in_polygon_square():
    sq = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
    assert point_in_polygon(1.0, 1.0, sq)
    assert not point_in_polygon(3.0, 1.0, sq)
    assert not point_in_polygon(1.0, -1.0, sq)


def test_free_zone_has_zero_occupancy():
    occ = zone_occupancy(10, 10, 1.0, 0.0, 0.0, _grid(10, 10),
                         [(1.0, 1.0), (4.0, 1.0), (4.0, 4.0), (1.0, 4.0)])
    assert occ.inside_cells == 9
    assert occ.occupied_ratio == 0.0
    assert occ.unknown_ratio == 0.0


def test_wall_inside_zone_counts_occupied():
    data = _grid(10, 10)
    for cy in range(10):
        data[cy * 10 + 2] = 100
    occ = zone_occupancy(10, 10, 1.0, 0.0, 0.0, data,
                         [(1.0, 1.0), (4.0, 1.0), (4.0, 4.0), (1.0, 4.0)])
    assert occ.inside_cells == 9
    assert occ.occupied_ratio == 3 / 9


def test_unknown_cells_are_reported_not_occupied():
    data = _grid(10, 10, fill=-1)
    occ = zone_occupancy(10, 10, 1.0, 0.0, 0.0, data,
                         [(1.0, 1.0), (4.0, 1.0), (4.0, 4.0), (1.0, 4.0)])
    assert occ.inside_cells == 9
    assert occ.occupied_ratio == 0.0
    assert occ.unknown_ratio == 1.0


def test_zone_outside_map_returns_empty():
    occ = zone_occupancy(10, 10, 1.0, 0.0, 0.0, _grid(10, 10),
                         [(20.0, 20.0), (22.0, 20.0), (22.0, 22.0)])
    assert occ == ZoneOccupancy(0, 0.0, 0.0)


def test_closed_ring_polygon_accepted():
    occ = zone_occupancy(10, 10, 1.0, 0.0, 0.0, _grid(10, 10),
                         [(1.0, 1.0), (4.0, 1.0), (4.0, 4.0),
                          (1.0, 4.0), (1.0, 1.0)])
    assert occ.inside_cells == 9
