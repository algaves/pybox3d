import pytest

from pybox3d import RigidBody, Vec3, World


def test_add_joint_defaults_rest_length_to_current_distance():
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))
    b = world.add_body(RigidBody(Vec3(3, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))

    joint = world.add_joint(a, b)

    assert world.joint_count == 1
    assert joint.rest_length == pytest.approx(3.0)


def test_add_joint_accepts_explicit_rest_length():
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))
    b = world.add_body(RigidBody(Vec3(3, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))

    joint = world.add_joint(a, b, rest_length=1.0)

    assert joint.rest_length == pytest.approx(1.0)


def test_joint_body_a_and_body_b_are_world_backed_handles():
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))
    b = world.add_body(RigidBody(Vec3(3, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))

    joint = world.add_joint(a, b, rest_length=1.0)

    assert joint.body_a.position.to_tuple() == pytest.approx((0, 0, 0))
    assert joint.body_b.position.to_tuple() == pytest.approx((3, 0, 0))
    # World-backed: joint.body_a is a fresh handle each access (like
    # World.get_body()), but it resolves through the same world_index, so
    # mutating it does persist.
    joint.body_a.position = Vec3(9, 9, 9)
    assert joint.body_a.position.to_tuple() == pytest.approx((9, 9, 9))
    assert a.position.to_tuple() == pytest.approx((9, 9, 9))


def test_add_joint_rejects_standalone_body():
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))
    standalone = RigidBody(Vec3(3, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0)

    with pytest.raises(ValueError):
        world.add_joint(a, standalone)


def test_add_joint_rejects_body_from_a_different_world():
    world = World(gravity=(0, 0, 0))
    other_world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))
    b = other_world.add_body(RigidBody(Vec3(3, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))

    with pytest.raises(ValueError):
        world.add_joint(a, b)


def test_distance_joint_pulls_bodies_to_rest_length(approx):
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(-5, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    b = world.add_body(RigidBody(Vec3(5, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    world.add_joint(a, b, rest_length=2.0)

    for _ in range(600):
        world.step(1 / 60)

    dist = (b.position - a.position).length()
    assert dist == approx(2.0, abs=0.05)


def test_distance_joint_holds_pendulum_near_rest_length_under_gravity(approx):
    world = World(gravity=(0, -10, 0))
    anchor = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    bob = world.add_body(RigidBody(Vec3(2, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    world.add_joint(anchor, bob, rest_length=2.0)

    for _ in range(300):  # 2.5s
        world.step(1 / 120)

    dist = (bob.position - anchor.position).length()
    assert dist == approx(2.0, abs=0.1)


def test_get_joint_out_of_range_raises_index_error():
    world = World(gravity=(0, 0, 0))
    with pytest.raises(IndexError):
        world.get_joint(0)


def test_remove_joint_swap_remove_semantics():
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))
    b = world.add_body(RigidBody(Vec3(1, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))
    c = world.add_body(RigidBody(Vec3(2, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))

    world.add_joint(a, b, rest_length=1.0)
    world.add_joint(a, b, rest_length=2.0)
    last_joint = world.add_joint(b, c, rest_length=3.0)

    world.remove_joint(0)  # swap-remove: index 0 now holds what was index 2

    assert world.joint_count == 2
    with pytest.raises(ValueError):
        _ = last_joint.rest_length
