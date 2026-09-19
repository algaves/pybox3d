import pytest

from pybox3d import Box3D, CharacterMover, RigidBody, Sphere, Vec3, World

# ---- Debug Draw ----


def test_box_debug_lines_has_twelve_edges():
    body = RigidBody(Vec3(0, 0, 0), Vec3(1, 1, 1), mass=1.0)

    lines = body.debug_lines()

    assert len(lines) == 12
    for a, b in lines:
        assert isinstance(a, Vec3)
        assert isinstance(b, Vec3)


def test_sphere_debug_lines_are_three_circles():
    body = RigidBody.sphere(Vec3(0, 0, 0), 1.0, mass=1.0)

    lines = body.debug_lines()

    assert len(lines) == 3 * 16  # B3_DEBUG_CIRCLE_SEGMENTS


def test_capsule_debug_lines_nonempty():
    body = RigidBody.capsule(Vec3(0, 0, 0), 0.5, 1.0, mass=1.0)

    lines = body.debug_lines()

    assert len(lines) > 0


def test_convexhull_debug_lines_draws_aabb_box():
    vertices = [Vec3(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
    body = RigidBody.hull(Vec3(0, 0, 0), vertices, mass=1.0)

    lines = body.debug_lines()

    assert len(lines) == 12  # approximated as its AABB, same as a box


def test_compound_debug_lines_covers_every_child():
    children: list[tuple[Vec3, Box3D | Sphere]] = [
        (Vec3(-1, 0, 0), Box3D(Vec3(0, 0, 0), Vec3(0.2, 0.2, 0.2))),
        (Vec3(1, 0, 0), Sphere(Vec3(0, 0, 0), 0.3)),
    ]
    body = RigidBody.compound(Vec3(0, 0, 0), children, mass=1.0)

    lines = body.debug_lines()

    assert len(lines) == 12 + 3 * 16  # box child + sphere child


def test_mesh_debug_lines_three_edges_per_triangle():
    triangles = [
        (Vec3(-1, 0, -1), Vec3(1, 0, -1), Vec3(1, 0, 1)),
        (Vec3(-1, 0, -1), Vec3(1, 0, 1), Vec3(-1, 0, 1)),
    ]
    body = RigidBody.mesh(Vec3(0, 0, 0), triangles)

    lines = body.debug_lines()

    assert len(lines) == 2 * 3


def test_heightfield_debug_lines_are_grid_lines():
    heights = [[0.0, 0.0, 0.0], [0.0, 0.5, 0.0]]
    body = RigidBody.heightfield(Vec3(0, 0, 0), heights, cell_size=1.0)

    lines = body.debug_lines()

    rows, cols = 2, 3
    expected = rows * (cols - 1) + cols * (rows - 1)
    assert len(lines) == expected


def test_debug_lines_move_with_the_body():
    body = RigidBody(Vec3(0, 0, 0), Vec3(1, 1, 1), mass=1.0)
    before = body.debug_lines()

    body.position = Vec3(10, 0, 0)
    after = body.debug_lines()

    assert before[0][0].x != pytest.approx(after[0][0].x)
    assert after[0][0].x == pytest.approx(10.0, abs=1.5)


def test_world_debug_contacts_reports_overlapping_pairs():
    world = World(gravity=(0, 0, 0))
    world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(1, 1, 1), mass=1.0))
    world.add_body(RigidBody(Vec3(0.5, 0, 0), Vec3(1, 1, 1), mass=1.0))
    world.add_body(RigidBody(Vec3(10, 0, 0), Vec3(1, 1, 1), mass=1.0))

    contacts = world.debug_contacts()

    assert len(contacts) == 1
    point, normal = contacts[0]
    assert isinstance(point, Vec3)
    assert isinstance(normal, Vec3)


def test_world_debug_contacts_ignores_filter_joints():
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(1, 1, 1), mass=1.0))
    b = world.add_body(RigidBody(Vec3(0.5, 0, 0), Vec3(1, 1, 1), mass=1.0))
    world.add_filter_joint(a, b)

    # unlike World.step, debug_contacts reports the raw geometric overlap
    # regardless of collision filtering
    contacts = world.debug_contacts()

    assert len(contacts) == 1


def test_world_debug_joint_anchors_matches_joint_count():
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    b = world.add_body(RigidBody(Vec3(2, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    world.add_joint(a, b, rest_length=2.0, anchor_b=Vec3(-0.5, 0, 0))

    anchors = world.debug_joint_anchors()

    assert len(anchors) == 1
    anchor_a, anchor_b = anchors[0]
    assert anchor_a.to_tuple() == pytest.approx((0, 0, 0))
    assert anchor_b.to_tuple() == pytest.approx((1.5, 0, 0))


def test_world_debug_joint_anchors_empty_when_no_joints():
    world = World(gravity=(0, 0, 0))
    world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))

    assert world.debug_joint_anchors() == []


# ---- Character Mover ----


def test_character_mover_cannot_move_below_the_id_of_construction():
    shape = Box3D(Vec3(0, 0, 0), Vec3(0.4, 0.9, 0.4))
    mover = CharacterMover(Vec3(0, 5, 0), shape)

    assert mover.position.to_tuple() == pytest.approx((0, 5, 0))
    assert mover.velocity.to_tuple() == pytest.approx((0, 0, 0))
    assert mover.is_grounded is False


def test_character_mover_falls_and_lands_on_ground(approx):
    world = World(gravity=(0, 0, 0))
    world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(10, 0.5, 10), mass=0.0))

    shape = Box3D(Vec3(0, 0, 0), Vec3(0.4, 0.9, 0.4))
    mover = CharacterMover(Vec3(0, 5, 0), shape)

    for _ in range(200):
        mover.move(world, Vec3(0, -0.05, 0))

    assert mover.position.y == approx(0.9, abs=0.05)
    assert mover.is_grounded is True


def test_character_mover_slides_along_a_wall(approx):
    world = World(gravity=(0, 0, 0))
    world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(10, 0.5, 10), mass=0.0))
    world.add_body(RigidBody(Vec3(3, 1, 0), Vec3(0.5, 2, 10), mass=0.0))

    shape = Box3D(Vec3(0, 0, 0), Vec3(0.4, 0.9, 0.4))
    mover = CharacterMover(Vec3(0, 0.9, 0), shape)

    for _ in range(60):
        mover.move(world, Vec3(0.1, 0, 0.1))

    # stopped by the wall in x, but kept advancing in z (a slide, not a
    # full stop)
    assert mover.position.x < 2.5
    assert mover.position.z == approx(6.0, abs=0.1)


def test_character_mover_does_not_tunnel_through_a_thin_wall():
    world = World(gravity=(0, 0, 0))
    world.add_body(RigidBody(Vec3(2, 0, 0), Vec3(0.1, 2, 10), mass=0.0))

    shape = Box3D(Vec3(0, 0, 0), Vec3(0.4, 0.9, 0.4))
    mover = CharacterMover(Vec3(0, 0, 0), shape)

    for _ in range(30):
        mover.move(world, Vec3(0.1, 0, 0))

    assert mover.position.x < 1.6  # stopped in front of the wall, not past it


def test_character_mover_custom_shape_and_parameters(approx):
    shape = Sphere(Vec3(0, 0, 0), 0.5)
    mover = CharacterMover(Vec3(0, 0, 0), shape)

    assert isinstance(mover.shape, Sphere)
    mover.skin_width = 0.02
    mover.max_slide_iterations = 8
    mover.ground_normal_min_y = 0.7
    assert mover.skin_width == approx(0.02)
    assert mover.max_slide_iterations == 8
    assert mover.ground_normal_min_y == approx(0.7)


def test_character_mover_rejects_invalid_parameters():
    shape = Box3D(Vec3(0, 0, 0), Vec3(0.4, 0.9, 0.4))
    mover = CharacterMover(Vec3(0, 0, 0), shape)

    with pytest.raises(ValueError):
        mover.skin_width = -1.0
    with pytest.raises(ValueError):
        mover.max_slide_iterations = 0


def test_character_mover_move_requires_a_world():
    shape = Box3D(Vec3(0, 0, 0), Vec3(0.4, 0.9, 0.4))
    mover = CharacterMover(Vec3(0, 0, 0), shape)

    with pytest.raises(TypeError):
        mover.move("not a world", Vec3(0, 0, 0))  # type: ignore[arg-type]


def test_character_mover_rejects_nan_and_inf():
    world = World(gravity=(0, 0, 0))
    shape = Box3D(Vec3(0, 0, 0), Vec3(0.4, 0.9, 0.4))
    mover = CharacterMover(Vec3(0, 0, 0), shape)

    with pytest.raises(ValueError):
        mover.position = Vec3(float("nan"), 0, 0)
    with pytest.raises(ValueError):
        mover.velocity = Vec3(0, float("inf"), 0)
    with pytest.raises(ValueError):
        mover.skin_width = float("nan")
    with pytest.raises(ValueError):
        mover.ground_normal_min_y = float("inf")
    with pytest.raises(ValueError):
        mover.move(world, Vec3(float("nan"), 0, 0))
