import pytest

from pybox3d import RigidBody, Vec3, World


def test_falling_box_matches_closed_form_kinematics():
    gravity_y = -10.0
    world = World(gravity=(0, gravity_y, 0))
    box = RigidBody(Vec3(0, 10, 0), Vec3(0.5, 0.5, 0.5), mass=1.0)
    handle = world.add_body(box)

    dt = 1 / 240
    steps = 60  # 0.25s, short enough to stay well clear of the (absent) ground
    for _ in range(steps):
        world.step(dt)

    t = dt * steps
    expected_y = 10 + 0.5 * gravity_y * t * t
    assert handle.position.y == pytest.approx(expected_y, rel=2e-2)
    assert handle.linear_velocity.y == pytest.approx(gravity_y * t, rel=2e-2)


def test_box_resting_on_static_ground_settles():
    world = World(gravity=(0, -10, 0))
    ground = RigidBody(Vec3(0, -0.5, 0), Vec3(10, 0.5, 10), mass=0.0)
    world.add_body(ground)

    box = RigidBody(Vec3(0, 2, 0), Vec3(0.5, 0.5, 0.5), mass=1.0)
    handle = world.add_body(box)

    dt = 1 / 120
    for _ in range(600):  # 5 simulated seconds: long enough to settle
        world.step(dt)

    # Ground top face is at y=0, box half-height 0.5 -> resting center ~= 0.5.
    assert handle.position.y == pytest.approx(0.5, abs=0.05)
    assert abs(handle.linear_velocity.y) < 0.5


def test_far_apart_boxes_no_spurious_collision():
    world = World(gravity=(0, 0, 0))
    a = RigidBody(Vec3(-100, 0, 0), Vec3(1, 1, 1), mass=1.0)
    b = RigidBody(Vec3(100, 0, 0), Vec3(1, 1, 1), mass=1.0)
    handle_a = world.add_body(a)
    handle_b = world.add_body(b)

    for _ in range(10):
        world.step(1 / 60)

    assert handle_a.linear_velocity.to_tuple() == pytest.approx((0, 0, 0))
    assert handle_b.linear_velocity.to_tuple() == pytest.approx((0, 0, 0))


def test_add_body_returns_world_backed_handle_with_expected_index():
    world = World(gravity=(0, 0, 0))
    body = RigidBody(Vec3(1, 2, 3), Vec3(1, 1, 1), mass=1.0)
    handle = world.add_body(body)
    assert world.body_count == 1
    assert handle.position.to_tuple() == pytest.approx((1, 2, 3))

    # Mutating the original standalone `body` does not affect the world's
    # copy: add_body deep-copies.
    body.position = Vec3(99, 99, 99)
    assert handle.position.to_tuple() == pytest.approx((1, 2, 3))


def test_get_body_returns_fresh_handle_each_call():
    world = World(gravity=(0, 0, 0))
    world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(1, 1, 1), mass=1.0))
    h1 = world.get_body(0)
    h2 = world.get_body(0)
    assert h1 is not h2
    h1.position = Vec3(5, 5, 5)
    assert h2.position.to_tuple() == pytest.approx((5, 5, 5))


def test_get_body_out_of_range_raises_index_error():
    world = World(gravity=(0, 0, 0))
    with pytest.raises(IndexError):
        world.get_body(0)


def test_remove_body_swap_remove_semantics():
    world = World(gravity=(0, 0, 0))
    world.add_body(RigidBody(Vec3(1, 0, 0), Vec3(1, 1, 1), mass=1.0))
    world.add_body(RigidBody(Vec3(2, 0, 0), Vec3(1, 1, 1), mass=1.0))
    world.add_body(RigidBody(Vec3(3, 0, 0), Vec3(1, 1, 1), mass=1.0))

    last_handle = world.get_body(2)
    world.remove_body(0)  # swap-remove: index 0 now holds what was index 2

    assert world.body_count == 2
    # Documented v1 caveat: a handle constructed for the old index 2 now
    # resolves through index 2, which is out of range post-removal.
    with pytest.raises(ValueError):
        _ = last_handle.position


def test_removed_body_handle_raises_on_access():
    world = World(gravity=(0, 0, 0))
    world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(1, 1, 1), mass=1.0))
    handle = world.get_body(0)
    world.remove_body(0)
    with pytest.raises(ValueError):
        _ = handle.position
