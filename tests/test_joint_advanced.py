import pytest

from pybox3d import RigidBody, Vec3, World


def test_joint_anchors_default_to_body_centers():
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))
    b = world.add_body(RigidBody(Vec3(3, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))
    joint = world.add_joint(a, b)

    assert joint.anchor_a.to_tuple() == pytest.approx((0, 0, 0))
    assert joint.anchor_b.to_tuple() == pytest.approx((0, 0, 0))
    assert joint.has_limits is False
    assert joint.stiffness == 0.0
    assert joint.damping == 0.0


def test_add_joint_accepts_anchor_offsets(approx):
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.5, 0.5, 0.5), mass=0.0))
    b = world.add_body(RigidBody(Vec3(5, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))

    # anchor_a is offset 1 unit toward b, anchor_b is offset 1 unit back
    # toward a, so the anchor-to-anchor rest_length defaults to 3, not 5.
    joint = world.add_joint(a, b, anchor_a=Vec3(1, 0, 0), anchor_b=Vec3(-1, 0, 0))

    assert joint.rest_length == approx(3.0)


def test_joint_anchor_offset_induces_torque_on_the_anchored_body(approx):
    # A joint anchored away from a body's center pulls at that offset
    # point, not the center -- so a taut joint that suddenly has to arrest
    # sideways motion torques the body (full 6-DOF resolution, closing the
    # v1 "linear-only, center-only" gap).
    world = World(gravity=(0, 0, 0))
    anchor_body = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    swinger = world.add_body(RigidBody(Vec3(2, 0, 0), Vec3(0.3, 0.3, 0.3), mass=1.0))
    world.add_joint(anchor_body, swinger, anchor_b=Vec3(0.3, 0, 0), rest_length=2.0)

    swinger.linear_velocity = Vec3(0, -3, 0)
    for _ in range(10):
        world.step(1 / 120)

    assert swinger.angular_velocity.length() > 0.01


def test_joint_limits_allow_free_play_between_bounds(approx):
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    b = world.add_body(RigidBody(Vec3(1, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    world.add_joint(a, b, rest_length=1.0, min_length=0.5, max_length=2.0)

    # A push well within [min_length, max_length] should not be resisted --
    # the joint has no fixed rest-length pull in limits-only usage once a
    # velocity has carried it inside the free range.
    b.linear_velocity = Vec3(0.5, 0, 0)
    for _ in range(5):
        world.step(1 / 120)

    dist = (b.position - a.position).length()
    assert 1.0 < dist < 2.0


def test_joint_limits_stop_at_max_length(approx):
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    b = world.add_body(RigidBody(Vec3(1, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    world.add_joint(a, b, rest_length=1.0, min_length=0.5, max_length=1.5)

    b.linear_velocity = Vec3(5, 0, 0)
    for _ in range(120):
        world.step(1 / 60)

    dist = (b.position - a.position).length()
    assert dist == approx(1.5, abs=0.05)


def test_joint_limits_stop_at_min_length(approx):
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    b = world.add_body(RigidBody(Vec3(2, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    world.add_joint(a, b, rest_length=2.0, min_length=1.0, max_length=3.0)

    b.linear_velocity = Vec3(-5, 0, 0)
    for _ in range(120):
        world.step(1 / 60)

    dist = (b.position - a.position).length()
    assert dist == approx(1.0, abs=0.05)


def test_add_joint_setting_only_min_length_defaults_max_to_rest_length(approx):
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    b = world.add_body(RigidBody(Vec3(2, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))

    joint = world.add_joint(a, b, rest_length=2.0, min_length=1.0)

    assert joint.has_limits is True
    assert joint.min_length == approx(1.0)
    assert joint.max_length == approx(2.0)


def test_add_joint_rejects_max_length_below_min_length():
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    b = world.add_body(RigidBody(Vec3(2, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))

    with pytest.raises(ValueError):
        world.add_joint(a, b, min_length=2.0, max_length=1.0)


def test_add_joint_rejects_negative_stiffness_or_damping():
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    b = world.add_body(RigidBody(Vec3(2, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))

    with pytest.raises(ValueError):
        world.add_joint(a, b, stiffness=-1.0)
    with pytest.raises(ValueError):
        world.add_joint(a, b, damping=-1.0)


def test_joint_spring_pulls_toward_rest_length_but_is_softer_than_rigid(approx):
    # A stretched spring joint should pull back toward rest_length, but
    # (with modest stiffness) not snap all the way back within a handful
    # of steps the way the rigid mode's Baumgarte correction would.
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    b = world.add_body(RigidBody(Vec3(3, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    world.add_joint(a, b, rest_length=1.0, stiffness=5.0, damping=1.0)

    for _ in range(300):
        world.step(1 / 120)

    dist = (b.position - a.position).length()
    assert dist < 3.0  # pulled back in
    assert dist > 1.0  # but a soft spring, not a rigid snap to exactly 1.0


def test_joint_spring_with_high_damping_settles_near_rest_length(approx):
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    b = world.add_body(RigidBody(Vec3(3, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    world.add_joint(a, b, rest_length=1.0, stiffness=50.0, damping=20.0)

    for _ in range(600):
        world.step(1 / 120)

    dist = (b.position - a.position).length()
    assert dist == approx(1.0, abs=0.15)


def test_world_solver_iterations_defaults_to_four():
    world = World(gravity=(0, 0, 0))
    assert world.solver_iterations == 4


def test_world_solver_iterations_is_settable():
    world = World(gravity=(0, 0, 0), solver_iterations=8)
    assert world.solver_iterations == 8
    world.solver_iterations = 1
    assert world.solver_iterations == 1


def test_world_rejects_non_positive_solver_iterations():
    with pytest.raises(ValueError):
        World(gravity=(0, 0, 0), solver_iterations=0)
    world = World(gravity=(0, 0, 0))
    with pytest.raises(ValueError):
        world.solver_iterations = 0


def _chain_sag(solver_iterations: int, link_count: int = 8) -> float:
    world = World(gravity=(0, -9.81, 0), solver_iterations=solver_iterations)
    link_length = 0.5
    anchor = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    previous = anchor
    for i in range(link_count):
        y = -(i + 1) * link_length
        link = world.add_body(RigidBody(Vec3(0, y, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
        world.add_joint(previous, link, rest_length=link_length)
        previous = link

    for _ in range(600):
        world.step(1 / 120)

    return (previous.position - anchor.position).length()


def test_more_solver_iterations_reduces_joint_chain_sag():
    expected = 8 * 0.5
    sag_one_iteration = abs(expected - _chain_sag(solver_iterations=1))
    sag_default = abs(expected - _chain_sag(solver_iterations=4))

    assert sag_default < sag_one_iteration


def test_add_joint_rejects_nan_and_inf_numeric_arguments():
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    b = world.add_body(RigidBody(Vec3(2, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))

    with pytest.raises(ValueError):
        world.add_joint(a, b, rest_length=float("nan"))
    with pytest.raises(ValueError):
        world.add_joint(a, b, stiffness=float("inf"))
    with pytest.raises(ValueError):
        world.add_joint(a, b, min_length=float("nan"), max_length=2.0)
