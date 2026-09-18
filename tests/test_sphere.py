import pytest

from pybox3d import Box3D, RigidBody, Sphere, Vec3, World


def test_sphere_contains_point():
    s = Sphere(Vec3(0, 0, 0), 1.0)
    assert s.contains_point(Vec3(0.5, 0, 0)) is True
    assert s.contains_point(Vec3(2, 0, 0)) is False


def test_sphere_aabb():
    s = Sphere(Vec3(1, 2, 3), 2.0)
    lo, hi = s.aabb()
    assert lo.to_tuple() == pytest.approx((-1, 0, 1))
    assert hi.to_tuple() == pytest.approx((3, 4, 5))


def test_sphere_sphere_overlap(approx):
    a = Sphere(Vec3(0, 0, 0), 1.0)
    b = Sphere(Vec3(1.5, 0, 0), 1.0)
    contact = a.overlaps(b)
    assert contact is not None
    # GJK/EPA on two perfectly round shapes converges to an approximate
    # normal/penetration (see docs/limitations.md) -- exact for flat
    # shapes, small error here is expected and fine for contact response.
    assert contact.normal.dot((1, 0, 0)) == approx(1.0, abs=0.1)
    assert contact.penetration == approx(0.5, abs=0.05)


def test_sphere_sphere_no_overlap():
    a = Sphere(Vec3(0, 0, 0), 1.0)
    b = Sphere(Vec3(5, 0, 0), 1.0)
    assert a.overlaps(b) is None


def test_sphere_overlaps_box_is_consistent_both_directions(approx):
    box = Box3D(Vec3(0, 0, 0), Vec3(1, 1, 1))
    sphere = Sphere(Vec3(1.5, 0, 0), 1.0)
    c1 = box.overlaps(sphere)
    c2 = sphere.overlaps(box)
    assert c1 is not None
    assert c2 is not None
    assert c1.penetration == approx(0.5, abs=0.02)
    assert c2.penetration == approx(0.5, abs=0.02)
    # Swapping the call order flips which shape the normal points away
    # from, so the two normals should be roughly opposite.
    assert c1.normal.dot(c2.normal) == approx(-1.0, abs=0.1)


def test_sphere_raycast_hit():
    s = Sphere(Vec3(0, 0, 0), 1.0)
    hit = s.raycast(Vec3(-5, 0, 0), Vec3(1, 0, 0))
    assert hit is not None
    assert hit.t == pytest.approx(4.0)
    assert hit.point.to_tuple() == pytest.approx((-1, 0, 0))
    assert hit.normal.to_tuple() == pytest.approx((-1, 0, 0))


def test_sphere_raycast_miss():
    s = Sphere(Vec3(0, 0, 0), 1.0)
    assert s.raycast(Vec3(-5, 5, 0), Vec3(1, 0, 0)) is None


def test_rigidbody_sphere_classmethod_falls_and_rests_on_ground(approx):
    world = World(gravity=(0, -10, 0))
    ground = RigidBody(Vec3(0, -0.5, 0), Vec3(10, 0.5, 10), mass=0.0)
    world.add_body(ground)
    body = world.add_body(RigidBody.sphere(Vec3(0, 3, 0), 0.5, mass=1.0))

    for _ in range(600):  # 5 simulated seconds
        world.step(1 / 120)

    assert body.position.y == approx(0.5, abs=0.1)
    assert abs(body.linear_velocity.y) < 0.5


def test_rigidbody_sphere_shape_property_is_sphere():
    world = World(gravity=(0, 0, 0))
    body = world.add_body(RigidBody.sphere(Vec3(0, 0, 0), 1.5, mass=1.0))
    assert isinstance(body.shape, Sphere)
    assert body.shape.radius == pytest.approx(1.5)
