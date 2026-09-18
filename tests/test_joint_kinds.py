import pytest

from pybox3d import RigidBody, Vec3, World


def test_spherical_joint_holds_offset_anchors_together(approx):
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    b = world.add_body(RigidBody(Vec3(2, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    joint = world.add_spherical_joint(a, b, anchor_b=Vec3(-1.9, 0, 0))

    assert joint.kind == "spherical"
    b.linear_velocity = Vec3(0, -3, 0)
    for _ in range(120):
        world.step(1 / 60)

    anchor_b_world = b.position + b.orientation.rotate_vec3(Vec3(-1.9, 0, 0))
    assert anchor_b_world.to_tuple() == approx((0, 0, 0), abs=0.05)


def test_spherical_joint_free_to_rotate(approx):
    # Anchors at each body's own center (the default): the anchor point
    # coincides with body B's center of mass, so spinning body B about
    # its own center doesn't move the anchor at all -- a clean way to
    # verify rotation is genuinely unconstrained (no angular lock for
    # Spherical), without also needing a matching orbital linear
    # velocity the way a spin about an *offset* anchor would.
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    b = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    world.add_spherical_joint(a, b)

    b.angular_velocity = Vec3(0, 5, 0)
    for _ in range(30):
        world.step(1 / 60)

    # rotation is unconstrained: angular velocity should survive mostly
    # undamped (no angular lock for Spherical)
    assert b.angular_velocity.length() > 1.0


def test_add_spherical_joint_rejects_standalone_body():
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))
    standalone = RigidBody(Vec3(3, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0)

    with pytest.raises(ValueError):
        world.add_spherical_joint(a, standalone)


def test_revolute_joint_holds_hinge_point_and_swings_around_axis(approx):
    world = World(gravity=(0, 0, 0))
    hinge = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.05, 1, 0.05), mass=0.0))
    door = world.add_body(RigidBody(Vec3(1, 0, 0), Vec3(1, 1, 0.05), mass=1.0))
    joint = world.add_revolute_joint(hinge, door, axis=Vec3(0, 1, 0), anchor_b=Vec3(-1, 0, 0))
    assert joint.kind == "revolute"

    door.angular_velocity = Vec3(0, 2.0, 0)
    for _ in range(120):
        world.step(1 / 60)

    anchor_world = door.position + door.orientation.rotate_vec3(Vec3(-1, 0, 0))
    assert anchor_world.to_tuple() == approx((0, 0, 0), abs=0.1)
    # it actually swung (moved away from its starting position)
    assert (door.position - Vec3(1, 0, 0)).length() > 0.2


def test_revolute_joint_locks_non_hinge_rotation(approx):
    world = World(gravity=(0, 0, 0))
    hinge = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.05, 1, 0.05), mass=0.0))
    door = world.add_body(RigidBody(Vec3(1, 0, 0), Vec3(1, 1, 0.05), mass=1.0))
    world.add_revolute_joint(hinge, door, axis=Vec3(0, 1, 0), anchor_b=Vec3(-1, 0, 0))

    # spin about X (perpendicular to the hinge axis) should get canceled
    door.angular_velocity = Vec3(5.0, 0, 0)
    for _ in range(30):
        world.step(1 / 60)

    assert abs(door.angular_velocity.x) < 1.0


def test_revolute_joint_motor_drives_toward_target_speed(approx):
    world = World(gravity=(0, 0, 0))
    hinge = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.05, 1, 0.05), mass=0.0))
    door = world.add_body(RigidBody(Vec3(1, 0, 0), Vec3(1, 1, 0.05), mass=1.0))
    joint = world.add_revolute_joint(
        hinge,
        door,
        axis=Vec3(0, 1, 0),
        anchor_b=Vec3(-1, 0, 0),
        enable_motor=True,
        motor_speed=1.0,
        max_motor_effort=50.0,
    )
    assert joint.enable_motor is True

    for _ in range(120):
        world.step(1 / 60)

    assert door.angular_velocity.y == approx(1.0, abs=0.2)


def test_prismatic_joint_slides_along_axis_only(approx):
    world = World(gravity=(0, 0, 0))
    rail = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    cart = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    joint = world.add_prismatic_joint(rail, cart, axis=Vec3(1, 0, 0))
    assert joint.kind == "prismatic"

    cart.linear_velocity = Vec3(2, 5, 3)  # only the X component should survive
    for _ in range(10):
        world.step(1 / 60)

    assert cart.position.y == approx(0.0, abs=0.05)
    assert cart.position.z == approx(0.0, abs=0.05)
    assert cart.position.x > 0.1


def test_prismatic_joint_limits_stop_at_max_translation(approx):
    world = World(gravity=(0, 0, 0))
    rail = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    cart = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    world.add_prismatic_joint(
        rail, cart, axis=Vec3(1, 0, 0), min_translation=0.0, max_translation=2.0
    )

    cart.linear_velocity = Vec3(5, 0, 0)
    for _ in range(120):
        world.step(1 / 60)

    assert cart.position.x == approx(2.0, abs=0.05)


def test_prismatic_joint_locks_relative_rotation(approx):
    world = World(gravity=(0, 0, 0))
    rail = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    cart = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    world.add_prismatic_joint(rail, cart, axis=Vec3(1, 0, 0))

    cart.angular_velocity = Vec3(0, 5, 0)
    for _ in range(30):
        world.step(1 / 60)

    assert cart.angular_velocity.length() < 1.0


def test_prismatic_joint_motor_drives_toward_target_speed(approx):
    world = World(gravity=(0, 0, 0))
    rail = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    cart = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    world.add_prismatic_joint(
        rail, cart, axis=Vec3(1, 0, 0), enable_motor=True, motor_speed=1.0, max_motor_effort=50.0
    )

    for _ in range(60):
        world.step(1 / 60)

    assert cart.linear_velocity.x == approx(1.0, abs=0.2)


def test_weld_joint_full_6dof_lock(approx):
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    b = world.add_body(RigidBody(Vec3(1, 0, 0), Vec3(0.2, 0.2, 0.2), mass=1.0))
    joint = world.add_weld_joint(a, b, anchor_b=Vec3(-1, 0, 0))
    assert joint.kind == "weld"

    b.angular_velocity = Vec3(0, 3, 0)
    b.linear_velocity = Vec3(0, 2, 0)
    for _ in range(60):
        world.step(1 / 60)

    assert b.angular_velocity.length() < 1.0
    anchor_world = b.position + b.orientation.rotate_vec3(Vec3(-1, 0, 0))
    assert anchor_world.to_tuple() == approx((0, 0, 0), abs=0.1)


def test_motor_joint_springs_toward_linear_offset(approx):
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    b = world.add_body(RigidBody(Vec3(5, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    joint = world.add_motor_joint(
        a, b, linear_offset=Vec3(2, 0, 0), linear_stiffness=20.0, linear_damping=5.0
    )
    assert joint.kind == "motor"

    for _ in range(300):
        world.step(1 / 120)

    assert b.position.to_tuple() == approx((2, 0, 0), abs=0.1)


def test_motor_joint_springs_toward_angular_offset(approx):
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    b = world.add_body(RigidBody(Vec3(1, 0, 0), Vec3(0.2, 0.2, 0.2), mass=1.0))
    from pybox3d import Quat

    target = Quat.from_axis_angle(Vec3(0, 1, 0), 1.0)
    world.add_motor_joint(a, b, angular_offset=target, angular_stiffness=30.0, angular_damping=8.0)

    for _ in range(300):
        world.step(1 / 120)

    assert b.orientation.y == approx(target.y, abs=0.1)
    assert b.orientation.w == approx(target.w, abs=0.1)


def test_motor_joint_zero_stiffness_does_nothing(approx):
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    b = world.add_body(RigidBody(Vec3(5, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    world.add_motor_joint(a, b, linear_offset=Vec3(2, 0, 0))  # stiffness defaults to 0

    for _ in range(60):
        world.step(1 / 60)

    assert b.position.to_tuple() == approx((5, 0, 0))


def test_wheel_joint_suspension_spring_holds_wheel_up(approx):
    world = World(gravity=(0, -9.81, 0))
    chassis = world.add_body(RigidBody(Vec3(0, 2, 0), Vec3(1, 0.2, 0.5), mass=0.0))
    wheel = world.add_body(RigidBody.sphere(Vec3(0, 0.5, 0), 0.5, mass=1.0))
    joint = world.add_wheel_joint(
        chassis,
        wheel,
        suspension_axis=Vec3(0, 1, 0),
        axle_axis=Vec3(0, 0, 1),
        anchor_a=Vec3(0, -1.5, 0),
        suspension_stiffness=200.0,
        suspension_damping=20.0,
    )
    assert joint.kind == "wheel"

    for _ in range(300):
        world.step(1 / 120)

    # settles close to its rest offset (0.5), not in free-fall
    assert wheel.position.y == approx(0.5, abs=0.15)


def test_wheel_joint_free_spin_around_axle(approx):
    world = World(gravity=(0, 0, 0))
    chassis = world.add_body(RigidBody(Vec3(0, 2, 0), Vec3(1, 0.2, 0.5), mass=0.0))
    wheel = world.add_body(RigidBody.sphere(Vec3(0, 0.5, 0), 0.5, mass=1.0))
    world.add_wheel_joint(
        chassis,
        wheel,
        suspension_axis=Vec3(0, 1, 0),
        axle_axis=Vec3(0, 0, 1),
        anchor_a=Vec3(0, -1.5, 0),
    )

    wheel.angular_velocity = Vec3(0, 0, 5.0)
    for _ in range(30):
        world.step(1 / 60)

    assert wheel.angular_velocity.z == approx(5.0, abs=0.5)


def test_wheel_joint_limits_stop_translation():
    world = World(gravity=(0, -50, 0))
    chassis = world.add_body(RigidBody(Vec3(0, 2, 0), Vec3(1, 0.2, 0.5), mass=0.0))
    wheel = world.add_body(RigidBody.sphere(Vec3(0, 1.5, 0), 0.5, mass=1.0))
    world.add_wheel_joint(
        chassis,
        wheel,
        suspension_axis=Vec3(0, 1, 0),
        axle_axis=Vec3(0, 0, 1),
        anchor_a=Vec3(0, -0.5, 0),
        min_translation=-0.3,
        max_translation=0.3,
    )

    for _ in range(120):
        world.step(1 / 60)

    # falls but gets hard-capped at min_translation, not free-falling forever
    assert wheel.position.y > 0.0


def test_filter_joint_disables_collision_between_pair():
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(1, 1, 1), mass=1.0))
    b = world.add_body(RigidBody(Vec3(0.5, 0, 0), Vec3(1, 1, 1), mass=1.0))
    joint = world.add_filter_joint(a, b)
    assert joint.kind == "filter"

    for _ in range(30):
        world.step(1 / 60)

    # no push-apart: they stay exactly as overlapping as they started
    assert (b.position - a.position).length() == pytest.approx(0.5)


def test_filter_joint_does_not_affect_unrelated_pairs():
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(1, 1, 1), mass=1.0))
    b = world.add_body(RigidBody(Vec3(0.5, 0, 0), Vec3(1, 1, 1), mass=1.0))
    c = world.add_body(RigidBody(Vec3(3, 0, 0), Vec3(1, 1, 1), mass=1.0))
    world.add_filter_joint(a, b)

    for _ in range(30):
        world.step(1 / 60)

    # a/b stay overlapped (filtered); nothing pathological happens to c
    assert (b.position - a.position).length() == pytest.approx(0.5)
    assert c.position.x == pytest.approx(3.0)


def test_parallel_joint_locks_orientation_but_not_position(approx):
    world = World(gravity=(0, -9.81, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    b = world.add_body(RigidBody(Vec3(1, 5, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    joint = world.add_parallel_joint(a, b)
    assert joint.kind == "parallel"

    for _ in range(60):
        world.step(1 / 60)

    assert (b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w) == approx(
        (0, 0, 0, 1), abs=0.05
    )
    assert b.position.y < 4.0  # fell freely, translation unconstrained


def test_joint_attribute_raises_for_wrong_kind():
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    b = world.add_body(RigidBody(Vec3(1, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    joint = world.add_spherical_joint(a, b)

    with pytest.raises(AttributeError):
        _ = joint.axis
    with pytest.raises(AttributeError):
        _ = joint.linear_offset


def test_joint_cannot_be_constructed_directly():
    from pybox3d import Joint

    with pytest.raises(TypeError):
        Joint()


def test_get_joint_returns_correct_type_per_kind():
    from pybox3d import DistanceJoint, Joint

    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    b = world.add_body(RigidBody(Vec3(1, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    world.add_joint(a, b)
    world.add_weld_joint(a, b)

    assert isinstance(world.get_joint(0), DistanceJoint)
    assert isinstance(world.get_joint(1), Joint)


def test_remove_body_also_drops_new_kind_joints():
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    b = world.add_body(RigidBody(Vec3(1, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    joint = world.add_weld_joint(a, b)

    world.remove_body(0)

    assert world.joint_count == 0
    with pytest.raises(ValueError):
        _ = joint.anchor_a
