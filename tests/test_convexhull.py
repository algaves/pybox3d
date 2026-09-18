import pytest

from pybox3d import Box3D, ConvexHull, RigidBody, Sphere, Vec3, World

CUBE_VERTICES = [Vec3(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]


def test_hull_contains_point():
    hull = ConvexHull(Vec3(0, 0, 0), CUBE_VERTICES)
    assert hull.contains_point(Vec3(0.5, 0, 0)) is True
    assert hull.contains_point(Vec3(2, 0, 0)) is False


def test_hull_aabb():
    hull = ConvexHull(Vec3(1, 2, 3), CUBE_VERTICES)
    lo, hi = hull.aabb()
    assert lo.to_tuple() == pytest.approx((0, 1, 2))
    assert hi.to_tuple() == pytest.approx((2, 3, 4))


def test_hull_matches_box_overlap_both_directions(approx):
    hull = ConvexHull(Vec3(0, 0, 0), CUBE_VERTICES)  # same geometry as Box3D(0,0,0, (1,1,1))
    box = Box3D(Vec3(1.5, 0, 0), Vec3(1, 1, 1))
    c1 = hull.overlaps(box)
    c2 = box.overlaps(hull)
    assert c1 is not None
    assert c2 is not None
    assert c1.penetration == approx(0.5, abs=0.01)
    assert c2.penetration == approx(0.5, abs=0.01)


def test_hull_overlaps_hull(approx):
    a = ConvexHull(Vec3(0, 0, 0), CUBE_VERTICES)
    b = ConvexHull(Vec3(1.5, 0, 0), CUBE_VERTICES)
    contact = a.overlaps(b)
    assert contact is not None
    assert contact.penetration == approx(0.5, abs=0.01)
    assert contact.normal.dot((1, 0, 0)) == approx(1.0, abs=0.05)


def test_hull_overlaps_sphere_is_consistent_both_directions(approx):
    hull = ConvexHull(Vec3(0, 0, 0), CUBE_VERTICES)
    sphere = Sphere(Vec3(1.5, 0, 0), 1.0)
    c1 = hull.overlaps(sphere)
    c2 = sphere.overlaps(hull)
    assert c1 is not None
    assert c2 is not None
    assert c1.penetration == approx(0.5, abs=0.05)
    assert c2.penetration == approx(0.5, abs=0.05)


def test_hull_no_overlap():
    a = ConvexHull(Vec3(0, 0, 0), CUBE_VERTICES)
    b = ConvexHull(Vec3(10, 0, 0), CUBE_VERTICES)
    assert a.overlaps(b) is None


def test_hull_raycast_hit():
    hull = ConvexHull(Vec3(0, 0, 0), CUBE_VERTICES)
    hit = hull.raycast(Vec3(-5, 0, 0), Vec3(1, 0, 0))
    assert hit is not None
    assert hit.point.to_tuple() == pytest.approx((-1, 0, 0))


def test_hull_raycast_miss():
    hull = ConvexHull(Vec3(0, 0, 0), CUBE_VERTICES)
    assert hull.raycast(Vec3(-5, 5, 0), Vec3(1, 0, 0)) is None


def test_hull_rejects_too_many_vertices():
    with pytest.raises(ValueError):
        ConvexHull(Vec3(0, 0, 0), [Vec3(0, 0, 0)] * 33)


def test_rigidbody_hull_classmethod_falls_and_rests_on_ground(approx):
    world = World(gravity=(0, -10, 0))
    ground = RigidBody(Vec3(0, -0.5, 0), Vec3(10, 0.5, 10), mass=0.0)
    world.add_body(ground)
    body = world.add_body(RigidBody.hull(Vec3(0, 3, 0), CUBE_VERTICES, mass=1.0))

    for _ in range(600):
        world.step(1 / 120)

    # A unit half-extent cube resting on the ground settles with its
    # center one unit above y=0.
    assert body.position.y == approx(1.0, abs=0.15)
    assert abs(body.linear_velocity.y) < 0.5


def test_rigidbody_hull_shape_property_is_convexhull():
    world = World(gravity=(0, 0, 0))
    body = world.add_body(RigidBody.hull(Vec3(0, 0, 0), CUBE_VERTICES, mass=1.0))
    assert isinstance(body.shape, ConvexHull)
    assert body.shape.vertex_count == 8
