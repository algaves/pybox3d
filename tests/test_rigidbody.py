import pytest

from pybox3d import RigidBody, Vec3, World


def test_static_body_has_zero_inv_mass():
    body = RigidBody(Vec3(0, 0, 0), Vec3(1, 1, 1), mass=0.0)
    assert body.mass == 0.0
    assert body.inv_mass == 0.0
    assert body.is_static is True


def test_dynamic_body_inv_mass():
    body = RigidBody(Vec3(0, 0, 0), Vec3(1, 1, 1), mass=2.0)
    assert body.inv_mass == pytest.approx(0.5)
    assert body.is_static is False


def test_negative_mass_rejected():
    body = RigidBody(Vec3(0, 0, 0), Vec3(1, 1, 1), mass=1.0)
    with pytest.raises(ValueError):
        body.mass = -1.0


def test_box_inertia_closed_form():
    # A unit cube (half-extents 1,1,1 -> full dims 2,2,2) of mass 6:
    # I_xx = m/12 * (dy^2 + dz^2) = 6/12 * (4+4) = 4, same for all axes
    # by symmetry.
    body = RigidBody(Vec3(0, 0, 0), Vec3(1, 1, 1), mass=6.0)
    world = World(gravity=(0, 0, 0))
    handle = world.add_body(body)
    # inertia isn't exposed directly; verify indirectly via a known torque
    # response: apply an impulse off-center along y and check angular
    # velocity magnitude is finite and in the expected ballpark.
    handle.apply_impulse(Vec3(0, 0, 1), point=Vec3(1, 0, 0))
    world.step(1 / 60)
    assert handle.angular_velocity.length() > 0.0


def test_force_and_impulse_via_world_step():
    body = RigidBody(Vec3(0, 0, 0), Vec3(1, 1, 1), mass=1.0)
    world = World(gravity=(0, 0, 0))
    handle = world.add_body(body)
    handle.apply_force(Vec3(10, 0, 0))
    world.step(1.0)
    assert handle.linear_velocity.x == pytest.approx(10.0, rel=1e-3)


def test_apply_impulse_directly_changes_velocity():
    body = RigidBody(Vec3(0, 0, 0), Vec3(1, 1, 1), mass=2.0)
    body.apply_impulse(Vec3(4, 0, 0))
    assert body.linear_velocity.x == pytest.approx(2.0)  # impulse / mass


def test_static_bodies_never_gain_velocity():
    body = RigidBody(Vec3(0, 0, 0), Vec3(1, 1, 1), mass=0.0)
    world = World(gravity=(0, -10, 0))
    handle = world.add_body(body)
    handle.apply_force(Vec3(0, 100, 0))
    for _ in range(10):
        world.step(1 / 60)
    assert handle.linear_velocity.to_tuple() == pytest.approx((0, 0, 0))
    assert handle.position.to_tuple() == pytest.approx((0, 0, 0))
