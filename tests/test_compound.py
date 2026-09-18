import pytest

from pybox3d import Box3D, Compound, RigidBody, Sphere, Vec3, World


def _dumbbell() -> Compound:
    # Two half-extent-0.5 boxes, two units apart (a real gap between them).
    return Compound(
        Vec3(0, 0, 0),
        [
            (Vec3(0, 0, 0), Box3D(Vec3(0, 0, 0), Vec3(0.5, 0.5, 0.5))),
            (Vec3(2, 0, 0), Box3D(Vec3(0, 0, 0), Vec3(0.5, 0.5, 0.5))),
        ],
    )


def test_compound_contains_point_in_either_child():
    compound = _dumbbell()
    assert compound.contains_point(Vec3(0, 0, 0)) is True
    assert compound.contains_point(Vec3(2, 0, 0)) is True
    assert compound.contains_point(Vec3(1, 0, 0)) is False  # gap between the two boxes
    assert compound.contains_point(Vec3(10, 0, 0)) is False


def test_compound_aabb_is_union_of_children():
    compound = _dumbbell()
    lo, hi = compound.aabb()
    assert lo.to_tuple() == pytest.approx((-0.5, -0.5, -0.5))
    assert hi.to_tuple() == pytest.approx((2.5, 0.5, 0.5))


def test_compound_overlaps_box(approx):
    compound = _dumbbell()
    box = Box3D(Vec3(2.7, 0, 0), Vec3(0.5, 0.5, 0.5))
    contact = compound.overlaps(box)
    assert contact is not None
    assert contact.penetration == approx(0.3, abs=0.02)


def test_compound_no_overlap():
    compound = _dumbbell()
    box = Box3D(Vec3(10, 0, 0), Vec3(0.5, 0.5, 0.5))
    assert compound.overlaps(box) is None


def test_compound_overlaps_sphere(approx):
    compound = _dumbbell()
    sphere = Sphere(Vec3(2.9, 0, 0), 0.5)
    contact = compound.overlaps(sphere)
    assert contact is not None
    assert contact.penetration == approx(0.1, abs=0.05)


def test_compound_raycast_hits_closest_child():
    compound = _dumbbell()
    hit = compound.raycast(Vec3(-5, 0, 0), Vec3(1, 0, 0))
    assert hit is not None
    assert hit.point.to_tuple() == pytest.approx((-0.5, 0, 0))


def test_compound_raycast_miss():
    compound = _dumbbell()
    assert compound.raycast(Vec3(-5, 5, 0), Vec3(1, 0, 0)) is None


def test_compound_rejects_nested_compound():
    inner = _dumbbell()
    with pytest.raises(TypeError):
        Compound(Vec3(0, 0, 0), [(Vec3(0, 0, 0), inner)])  # type: ignore[list-item]


def test_compound_rejects_too_many_children():
    child = (Vec3(0, 0, 0), Box3D(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1)))
    with pytest.raises(ValueError):
        Compound(Vec3(0, 0, 0), [child] * 9)


def test_rigidbody_compound_classmethod_falls_and_rests_on_ground(approx):
    world = World(gravity=(0, -10, 0))
    ground = RigidBody(Vec3(0, -0.5, 0), Vec3(10, 0.5, 10), mass=0.0)
    world.add_body(ground)
    body = world.add_body(
        RigidBody.compound(
            Vec3(0, 3, 0),
            [(Vec3(0, 0, 0), Box3D(Vec3(0, 0, 0), Vec3(0.5, 0.5, 0.5)))],
            mass=1.0,
        )
    )

    for _ in range(600):
        world.step(1 / 120)

    assert body.position.y == approx(0.5, abs=0.15)
    assert abs(body.linear_velocity.y) < 0.5


def test_rigidbody_compound_shape_property_is_compound():
    world = World(gravity=(0, 0, 0))
    body = world.add_body(
        RigidBody.compound(
            Vec3(0, 0, 0),
            [(Vec3(0, 0, 0), Box3D(Vec3(0, 0, 0), Vec3(0.5, 0.5, 0.5)))],
            mass=1.0,
        )
    )
    assert isinstance(body.shape, Compound)
    assert body.shape.child_count == 1
