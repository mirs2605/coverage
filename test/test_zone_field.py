"""Unit tests for zone_field (ROS-free)."""

import pytest

from coverage.zone_field import normalize_zone


def test_square_ok():
    pts = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)]
    assert normalize_zone(pts) == pts


def test_closing_duplicate_removed():
    pts = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
    assert normalize_zone(pts) == pts[:4]


def test_too_few_vertices_raises():
    with pytest.raises(ValueError):
        normalize_zone([(0.0, 0.0), (1.0, 0.0)])


def test_empty_raises():
    with pytest.raises(ValueError):
        normalize_zone([])


def test_non_finite_raises():
    with pytest.raises(ValueError):
        normalize_zone([(0.0, 0.0), (float('nan'), 0.0), (0.0, 1.0)])
    with pytest.raises(ValueError):
        normalize_zone([(0.0, 0.0), (float('inf'), 0.0), (0.0, 1.0)])


def test_order_preserved():
    # 巻き順はF2Cに任せる。入力順を変えないこと
    pts = [(2.0, 1.0), (0.0, 0.0), (0.0, 1.0), (2.0, 0.0)]
    assert normalize_zone(pts) == pts
