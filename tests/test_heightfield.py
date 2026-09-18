import pytest

from pybox3d import HeightField, RigidBody, Sphere, Vec3, World

FLAT_HEIGHTS = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]


def test_heightfield_aabb():
    hf = HeightField(Vec3(0, 0, 0), FLAT_HEIGHTS, cell_size=2.0)
    lo, hi = hf.aabb()
    assert lo.to_tuple() == pytest.approx((-2, 0, -2))
    assert hi.to_tuple() == pytest.approx((2, 0, 2))


def test_heightfield_contains_point_always_false():
    hf = HeightField(Vec3(0, 0, 0), FLAT_HEIGHTS, cell_size=2.0)
    assert hf.contains_point(Vec3(0, 0, 0)) is False


def test_heightfield_overlaps_sphere_is_consistent_both_directions(approx):
    hf = HeightField(Vec3(0, 0, 0), FLAT_HEIGHTS, cell_size=2.0)
    sphere = Sphere(Vec3(0, 0.3, 0), 0.5)
    c1 = hf.overlaps(sphere)
    c2 = sphere.overlaps(hf)
    assert c1 is not None
    assert c2 is not None
    assert c1.penetration == approx(0.2, abs=0.05)
    assert c2.penetration == approx(0.2, abs=0.05)


def test_heightfield_no_overlap_far_away():
    hf = HeightField(Vec3(0, 0, 0), FLAT_HEIGHTS, cell_size=2.0)
    sphere = Sphere(Vec3(0, 10, 0), 0.5)
    assert hf.overlaps(sphere) is None


def test_heightfield_raycast_hit():
    hf = HeightField(Vec3(0, 0, 0), FLAT_HEIGHTS, cell_size=2.0)
    hit = hf.raycast(Vec3(0, 5, 0), Vec3(0, -1, 0))
    assert hit is not None
    assert hit.point.to_tuple() == pytest.approx((0, 0, 0))


def test_heightfield_raycast_miss():
    hf = HeightField(Vec3(0, 0, 0), FLAT_HEIGHTS, cell_size=2.0)
    assert hf.raycast(Vec3(0, 5, 0), Vec3(0, 1, 0)) is None


def test_heightfield_rejects_ragged_rows():
    with pytest.raises(ValueError):
        HeightField(Vec3(0, 0, 0), [[0.0, 0.0], [0.0, 0.0, 0.0]], cell_size=1.0)


def test_heightfield_rejects_too_many_rows():
    row = [0.0, 0.0]
    with pytest.raises(ValueError):
        HeightField(Vec3(0, 0, 0), [row] * 17, cell_size=1.0)


def test_rigidbody_heightfield_is_always_static():
    world = World(gravity=(0, 0, 0))
    ground = world.add_body(RigidBody.heightfield(Vec3(0, 0, 0), FLAT_HEIGHTS, cell_size=2.0))
    assert ground.is_static is True
    with pytest.raises(ValueError):
        ground.mass = 1.0


def test_rigidbody_heightfield_shape_property_is_heightfield():
    world = World(gravity=(0, 0, 0))
    body = world.add_body(RigidBody.heightfield(Vec3(0, 0, 0), FLAT_HEIGHTS, cell_size=2.0))
    assert isinstance(body.shape, HeightField)
    assert body.shape.rows == 3
    assert body.shape.cols == 3


def test_sphere_rests_on_heightfield_ground(approx):
    world = World(gravity=(0, -10, 0))
    # A large cell_size, not FLAT_HEIGHTS' small 4x4-unit test grid: with
    # real angular impulses (see docs/limitations.md), any spin the ball
    # picks up from an off-center contact makes it roll -- physically
    # correct, since nothing here models rolling resistance -- so a small
    # ground plane risks it rolling off the edge before the 5-second test
    # window ends, same as it would in real life on an equally short table.
    world.add_body(RigidBody.heightfield(Vec3(0, 0, 0), FLAT_HEIGHTS, cell_size=10.0))
    ball = world.add_body(RigidBody.sphere(Vec3(0, 3, 0), 0.5, mass=1.0))

    for _ in range(600):
        world.step(1 / 120)

    assert ball.position.y == approx(0.5, abs=0.1)
    assert abs(ball.linear_velocity.y) < 0.5
