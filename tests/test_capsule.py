import pytest

from pybox3d import Box3D, Capsule, RigidBody, Sphere, Vec3, World


def test_capsule_contains_point_in_cylinder_and_caps():
    c = Capsule(Vec3(0, 0, 0), 0.5, 1.0)
    assert c.contains_point(Vec3(0, 0, 0)) is True
    assert c.contains_point(Vec3(0.4, 0, 0)) is True
    assert c.contains_point(Vec3(0, 1.4, 0)) is True  # inside the top hemispherical cap
    assert c.contains_point(Vec3(0, 2, 0)) is False


def test_capsule_aabb():
    c = Capsule(Vec3(0, 0, 0), 0.5, 1.0)
    lo, hi = c.aabb()
    assert lo.to_tuple() == pytest.approx((-0.5, -1.5, -0.5))
    assert hi.to_tuple() == pytest.approx((0.5, 1.5, 0.5))


def test_capsule_overlaps_sphere_at_cap(approx):
    c = Capsule(Vec3(0, 0, 0), 0.5, 1.0)
    s = Sphere(Vec3(0, 1.7, 0), 0.5)
    contact = c.overlaps(s)
    assert contact is not None
    assert contact.penetration == approx(0.3, abs=0.05)


def test_capsule_overlaps_box_is_consistent_both_directions(approx):
    box = Box3D(Vec3(0, 0, 0), Vec3(1, 1, 1))
    cap = Capsule(Vec3(1.3, 0, 0), 0.5, 1.0)
    c1 = box.overlaps(cap)
    c2 = cap.overlaps(box)
    assert c1 is not None
    assert c2 is not None
    assert c1.penetration == approx(0.2, abs=0.02)
    assert c2.penetration == approx(0.2, abs=0.02)


def test_capsule_raycast_along_axis_hits_cap():
    c = Capsule(Vec3(0, 0, 0), 0.5, 1.0)
    hit = c.raycast(Vec3(0, 5, 0), Vec3(0, -1, 0))
    assert hit is not None
    assert hit.point.to_tuple() == pytest.approx((0, 1.5, 0))
    assert hit.normal.to_tuple() == pytest.approx((0, 1, 0))


def test_capsule_raycast_perpendicular_hits_cylinder_body():
    c = Capsule(Vec3(0, 0, 0), 0.5, 1.0)
    hit = c.raycast(Vec3(5, 0, 0), Vec3(-1, 0, 0))
    assert hit is not None
    assert hit.point.to_tuple() == pytest.approx((0.5, 0, 0))
    assert hit.normal.to_tuple() == pytest.approx((1, 0, 0))


def test_capsule_raycast_miss():
    c = Capsule(Vec3(0, 0, 0), 0.5, 1.0)
    assert c.raycast(Vec3(5, 5, 0), Vec3(-1, 0, 0)) is None


def test_rigidbody_capsule_classmethod_falls_and_rests_on_ground(approx):
    world = World(gravity=(0, -10, 0))
    ground = RigidBody(Vec3(0, -0.5, 0), Vec3(10, 0.5, 10), mass=0.0)
    world.add_body(ground)
    # An upright capsule rests on its bottom hemisphere, so its center
    # settles at half_height + radius above the ground's top face (y=0).
    body = world.add_body(RigidBody.capsule(Vec3(0, 3, 0), 0.3, 0.5, mass=1.0))

    for _ in range(600):  # 5 simulated seconds
        world.step(1 / 120)

    assert body.position.y == approx(0.8, abs=0.15)
    assert abs(body.linear_velocity.y) < 0.5


def test_rigidbody_capsule_shape_property_is_capsule():
    world = World(gravity=(0, 0, 0))
    body = world.add_body(RigidBody.capsule(Vec3(0, 0, 0), 0.4, 0.6, mass=1.0))
    assert isinstance(body.shape, Capsule)
    assert body.shape.radius == pytest.approx(0.4)
    assert body.shape.half_height == pytest.approx(0.6)
