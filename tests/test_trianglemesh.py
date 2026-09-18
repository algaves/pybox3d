import pytest

from pybox3d import Box3D, RigidBody, TriangleMesh, Vec3, World

# A flat 10x10 quad (two triangles) at y=0, spanning x,z in [-5, 5].
QUAD_TRIANGLES = [
    (Vec3(-5, 0, -5), Vec3(5, 0, -5), Vec3(5, 0, 5)),
    (Vec3(-5, 0, -5), Vec3(5, 0, 5), Vec3(-5, 0, 5)),
]


def test_mesh_aabb():
    mesh = TriangleMesh(Vec3(0, 0, 0), QUAD_TRIANGLES)
    lo, hi = mesh.aabb()
    assert lo.to_tuple() == pytest.approx((-5, 0, -5))
    assert hi.to_tuple() == pytest.approx((5, 0, 5))


def test_mesh_contains_point_always_false():
    mesh = TriangleMesh(Vec3(0, 0, 0), QUAD_TRIANGLES)
    assert mesh.contains_point(Vec3(0, 0, 0)) is False


def test_mesh_overlaps_box_is_consistent_both_directions(approx):
    mesh = TriangleMesh(Vec3(0, 0, 0), QUAD_TRIANGLES)
    box = Box3D(Vec3(0, 0.3, 0), Vec3(0.5, 0.5, 0.5))
    c1 = mesh.overlaps(box)
    c2 = box.overlaps(mesh)
    assert c1 is not None
    assert c2 is not None
    assert c1.penetration == approx(0.2, abs=0.02)
    assert c2.penetration == approx(0.2, abs=0.02)
    assert c1.normal.dot((0, 1, 0)) == approx(1.0, abs=0.01)
    assert c2.normal.dot((0, -1, 0)) == approx(1.0, abs=0.01)


def test_mesh_no_overlap_far_away():
    mesh = TriangleMesh(Vec3(0, 0, 0), QUAD_TRIANGLES)
    box = Box3D(Vec3(0, 10, 0), Vec3(0.5, 0.5, 0.5))
    assert mesh.overlaps(box) is None


def test_mesh_raycast_hit():
    mesh = TriangleMesh(Vec3(0, 0, 0), QUAD_TRIANGLES)
    hit = mesh.raycast(Vec3(0, 5, 0), Vec3(0, -1, 0))
    assert hit is not None
    assert hit.point.to_tuple() == pytest.approx((0, 0, 0))


def test_mesh_raycast_miss():
    mesh = TriangleMesh(Vec3(0, 0, 0), QUAD_TRIANGLES)
    assert mesh.raycast(Vec3(0, 5, 0), Vec3(0, 1, 0)) is None


def test_mesh_rejects_too_many_triangles():
    tri = QUAD_TRIANGLES[0]
    with pytest.raises(ValueError):
        TriangleMesh(Vec3(0, 0, 0), [tri] * 65)


def test_rigidbody_mesh_is_always_static():
    world = World(gravity=(0, 0, 0))
    ground = world.add_body(RigidBody.mesh(Vec3(0, 0, 0), QUAD_TRIANGLES))
    assert ground.is_static is True
    with pytest.raises(ValueError):
        ground.mass = 1.0


def test_rigidbody_mesh_shape_property_is_trianglemesh():
    world = World(gravity=(0, 0, 0))
    body = world.add_body(RigidBody.mesh(Vec3(0, 0, 0), QUAD_TRIANGLES))
    assert isinstance(body.shape, TriangleMesh)
    assert body.shape.triangle_count == 2


def test_sphere_rests_on_mesh_ground(approx):
    world = World(gravity=(0, -10, 0))
    world.add_body(RigidBody.mesh(Vec3(0, 0, 0), QUAD_TRIANGLES))
    ball = world.add_body(RigidBody.sphere(Vec3(0, 3, 0), 0.5, mass=1.0))

    for _ in range(600):
        world.step(1 / 120)

    assert ball.position.y == approx(0.5, abs=0.1)
    assert abs(ball.linear_velocity.y) < 0.5
