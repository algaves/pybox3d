import pytest

from pybox3d import RigidBody, Vec3, World


def test_world_no_longer_exposes_global_default_materials():
    # Superseded by per-body restitution/friction actually being
    # consulted -- see test_per_body_restitution_is_used below.
    world = World(gravity=(0, 0, 0))
    with pytest.raises(AttributeError):
        _ = world.default_restitution  # type: ignore[attr-defined]
    with pytest.raises(AttributeError):
        _ = world.default_friction  # type: ignore[attr-defined]


def _drop_and_measure_bounce(restitution: float) -> float:
    world = World(gravity=(0, -10, 0))
    ground = world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(10, 0.5, 10), mass=0.0))
    ground.restitution = 0.0
    ball = world.add_body(RigidBody.sphere(Vec3(0, 3, 0), 0.5, mass=1.0))
    ball.restitution = restitution

    touched_ground = False
    peak_after_bounce = 0.0
    for _ in range(300):
        world.step(1 / 240)
        if not touched_ground:
            if ball.position.y < 0.6:
                touched_ground = True
        elif ball.linear_velocity.y > 0:
            peak_after_bounce = max(peak_after_bounce, ball.position.y)
        elif peak_after_bounce > 0.0:
            break  # started falling again after the bounce: done
    return peak_after_bounce


def test_per_body_restitution_is_used(approx):
    # A bouncier ball must rebound higher than a duller one -- this is
    # only possible if RigidBody.restitution (not a removed global
    # default) actually drives contact resolution.
    bouncy_peak = _drop_and_measure_bounce(restitution=0.9)
    dull_peak = _drop_and_measure_bounce(restitution=0.05)
    assert bouncy_peak > dull_peak + 0.3


def test_per_body_friction_is_used(approx):
    def slide_distance(friction: float) -> float:
        world = World(gravity=(0, -10, 0))
        ground = world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(20, 0.5, 20), mass=0.0))
        ground.friction = friction
        box = world.add_body(RigidBody(Vec3(0, 0.5, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))
        box.friction = friction
        box.linear_velocity = Vec3(5, 0, 0)

        for _ in range(300):
            world.step(1 / 120)
        return box.position.x

    low_friction_distance = slide_distance(0.05)
    high_friction_distance = slide_distance(1.0)
    assert low_friction_distance > high_friction_distance + 0.5


def test_friction_induces_rolling_spin():
    # A sliding sphere should pick up angular velocity (spin) from
    # friction once real 6-DOF contact impulses are in play -- with the
    # old linear-only contact resolution, angular_velocity never changed.
    world = World(gravity=(0, -10, 0))
    ground = world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(20, 0.5, 20), mass=0.0))
    ground.friction = 0.8
    ball = world.add_body(RigidBody.sphere(Vec3(0, 0.5, 0), 0.5, mass=1.0))
    ball.friction = 0.8
    ball.linear_velocity = Vec3(5, 0, 0)

    for _ in range(120):
        world.step(1 / 120)

    assert ball.angular_velocity.length() > 1.0


def test_static_body_unaffected_by_contact_forces():
    world = World(gravity=(0, -10, 0))
    ground = world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(10, 0.5, 10), mass=0.0))
    world.add_body(RigidBody.sphere(Vec3(0, 3, 0), 0.5, mass=1.0))

    for _ in range(300):
        world.step(1 / 120)

    assert ground.position.to_tuple() == pytest.approx((0, -0.5, 0))
    assert ground.linear_velocity.to_tuple() == pytest.approx((0, 0, 0))
    assert ground.angular_velocity.to_tuple() == pytest.approx((0, 0, 0))
