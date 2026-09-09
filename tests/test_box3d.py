import math

import pytest

from pybox3d import BOX_KIND_AABB, BOX_KIND_OBB, Box3D, Quat, Vec3


def test_aabb_construction_defaults_kind():
    box = Box3D(Vec3(0, 0, 0), Vec3(1, 1, 1))
    assert box.kind == BOX_KIND_AABB


def test_obb_construction_sets_kind():
    q = Quat.from_axis_angle(Vec3(0, 1, 0), math.pi / 4)
    box = Box3D(Vec3(0, 0, 0), Vec3(1, 1, 1), q)
    assert box.kind == BOX_KIND_OBB


def test_contains_point_center_and_outside():
    box = Box3D(Vec3(0, 0, 0), Vec3(1, 1, 1))
    assert box.contains_point(Vec3(0, 0, 0)) is True
    assert box.contains_point(Vec3(5, 0, 0)) is False


def test_contains_point_boundary():
    box = Box3D(Vec3(0, 0, 0), Vec3(1, 1, 1))
    assert box.contains_point(Vec3(1, 0, 0)) is True
    assert box.contains_point(Vec3(1.00001, 0, 0)) is False


def test_overlap_axis_aligned_true():
    a = Box3D(Vec3(0, 0, 0), Vec3(1, 1, 1))
    b = Box3D(Vec3(1.5, 0, 0), Vec3(1, 1, 1))
    contact = a.overlaps(b)
    assert contact is not None
    assert contact.penetration == pytest.approx(0.5, abs=1e-3)
    assert contact.normal.x == pytest.approx(1.0, abs=1e-3)


def test_overlap_axis_aligned_false():
    a = Box3D(Vec3(0, 0, 0), Vec3(1, 1, 1))
    b = Box3D(Vec3(5, 0, 0), Vec3(1, 1, 1))
    assert a.overlaps(b) is None


def test_overlap_rotated_obb_true():
    # A 45-degree-rotated box has a "diamond" footprint reaching further
    # along the axes than its half-extents alone would suggest.
    q = Quat.from_axis_angle(Vec3(0, 0, 1), math.pi / 4)
    a = Box3D(Vec3(0, 0, 0), Vec3(1, 1, 1), q)
    b = Box3D(Vec3(1.9, 0, 0), Vec3(1, 1, 1))
    assert a.overlaps(b) is not None


def test_overlap_rotated_obb_false_when_aabbs_would_falsely_overlap():
    # Two boxes rotated 45 degrees around Z, offset diagonally so their
    # *world AABBs* overlap but the true oriented boxes do not -- this is
    # the case that only a real SAT test (not an AABB fallback) catches.
    q = Quat.from_axis_angle(Vec3(0, 0, 1), math.pi / 4)
    half = Vec3(1, 1, 1)
    a = Box3D(Vec3(0, 0, 0), half, q)
    b = Box3D(Vec3(2.5, 2.5, 0), half, q)

    a_lo, a_hi = a.aabb()
    b_lo, b_hi = b.aabb()
    aabbs_overlap = a_lo.x <= b_hi.x and a_hi.x >= b_lo.x and a_lo.y <= b_hi.y and a_hi.y >= b_lo.y
    assert aabbs_overlap, "test setup should produce overlapping AABBs"
    assert a.overlaps(b) is None, "true OBB test must reject what the AABB fallback would accept"


def test_raycast_hit():
    box = Box3D(Vec3(0, 0, 0), Vec3(1, 1, 1))
    hit = box.raycast(Vec3(-5, 0, 0), Vec3(1, 0, 0))
    assert hit is not None
    assert hit.t == pytest.approx(4.0)
    assert hit.point.to_tuple() == pytest.approx((-1, 0, 0))
    assert hit.normal.to_tuple() == pytest.approx((-1, 0, 0))


def test_raycast_miss():
    box = Box3D(Vec3(0, 0, 0), Vec3(1, 1, 1))
    hit = box.raycast(Vec3(-5, 5, 0), Vec3(1, 0, 0))
    assert hit is None


def test_raycast_respects_max_t():
    box = Box3D(Vec3(0, 0, 0), Vec3(1, 1, 1))
    hit = box.raycast(Vec3(-5, 0, 0), Vec3(1, 0, 0), max_t=2.0)
    assert hit is None


def test_compute_aabb_of_rotated_obb():
    q = Quat.from_axis_angle(Vec3(0, 0, 1), math.pi / 4)
    box = Box3D(Vec3(0, 0, 0), Vec3(1, 1, 1), q)
    lo, hi = box.aabb()
    expected_extent = math.sqrt(2)
    assert lo.x == pytest.approx(-expected_extent, abs=1e-3)
    assert hi.x == pytest.approx(expected_extent, abs=1e-3)
