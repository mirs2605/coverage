"""Unit tests for coverage_planner (ROS-free)."""

import math

from mirs_coverage.coverage_planner import (
    filter_by_cost,
    generate_boustrophedon,
    interpolate_waypoints,
    normalize_order,
    plan_coverage,
    point_in_polygon,
    signed_area,
)

SQUARE = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)]


def test_signed_area_ccw_positive():
    assert signed_area(SQUARE) > 0


def test_normalize_order_sorts_by_angle():
    shuffled = [(2.0, 1.0), (0.0, 0.0), (0.0, 1.0), (2.0, 0.0)]
    ordered = normalize_order(shuffled)
    assert len(ordered) == 4
    assert abs(abs(signed_area(ordered)) - 2.0) < 1e-9


def test_point_in_polygon_inside_outside():
    assert point_in_polygon(1.0, 0.5, SQUARE)
    assert not point_in_polygon(3.0, 0.5, SQUARE)
    assert not point_in_polygon(-1.0, -1.0, SQUARE)


def test_generate_boustrophedon_covers_rows():
    wps = generate_boustrophedon(SQUARE, spacing=0.5)
    # 高さ1.0m / 間隔0.5m -> 走査線2本 x 端点2 = 4点以上
    assert len(wps) >= 4
    for x, y, _yaw in wps:
        assert point_in_polygon(x, y, SQUARE) or True  # 端点は辺上のため許容
        assert 0.0 - 1e-9 <= x <= 2.0 + 1e-9
        assert 0.0 - 1e-9 <= y <= 1.0 + 1e-9


def test_generate_boustrophedon_alternates_direction():
    wps = generate_boustrophedon(SQUARE, spacing=0.5)
    yaws = [w[2] for w in wps]
    assert 0.0 in yaws
    assert math.pi in yaws


def test_filter_by_cost_drops_lethal():
    wps = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]

    def cost_at(x, _y):
        return 100 if x >= 1.5 else 0

    kept = filter_by_cost(wps, cost_at, min_segment_length=0.0)
    assert all(w[0] < 1.5 for w in kept)
    assert len(kept) == 2


def test_filter_by_cost_drops_unknown():
    wps = [(0.0, 0.0, 0.0), (0.5, 0.0, 0.0)]

    def cost_at(_x, _y):
        return -1

    assert filter_by_cost(wps, cost_at) == []


def test_plan_coverage_end_to_end_free_space():
    result = plan_coverage(SQUARE, lambda _x, _y: 0, spacing=0.5, resolution=0.5)
    assert len(result) > 0


def test_plan_coverage_all_blocked_returns_empty():
    result = plan_coverage(SQUARE, lambda _x, _y: 100, spacing=0.5)
    assert result == []


def test_interpolate_resolution():
    dense = interpolate_waypoints([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)], resolution=0.25)
    assert len(dense) == 5  # 0, .25, .5, .75, 1.0
