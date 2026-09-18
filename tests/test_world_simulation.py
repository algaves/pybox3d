import pytest

from pybox3d import RigidBody, Vec3, World, WorldSnapshot

# ---- Query API ----


def test_query_aabb_returns_overlapping_bodies_only():
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))
    world.add_body(RigidBody(Vec3(10, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))

    hits = world.query_aabb(Vec3(-1, -1, -1), Vec3(1, 1, 1))

    assert len(hits) == 1
    assert hits[0].position.to_tuple() == pytest.approx(a.position.to_tuple())


def test_query_aabb_empty_when_nothing_overlaps():
    world = World(gravity=(0, 0, 0))
    world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))

    hits = world.query_aabb(Vec3(100, 100, 100), Vec3(101, 101, 101))

    assert hits == []


def test_raycast_all_hits_every_body_along_the_ray():
    world = World(gravity=(0, 0, 0))
    world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))
    world.add_body(RigidBody(Vec3(5, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))
    world.add_body(RigidBody(Vec3(0, 5, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))  # off the ray

    hits = world.raycast_all(Vec3(-5, 0, 0), Vec3(1, 0, 0), max_distance=20)

    assert len(hits) == 2
    xs = sorted(body.position.x for body, _ in hits)
    assert xs == pytest.approx([0.0, 5.0])


def test_raycast_all_respects_max_distance():
    world = World(gravity=(0, 0, 0))
    world.add_body(RigidBody(Vec3(10, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))

    hits = world.raycast_all(Vec3(-5, 0, 0), Vec3(1, 0, 0), max_distance=5)

    assert hits == []


def test_raycast_all_returns_rayhit_with_point_and_normal():
    world = World(gravity=(0, 0, 0))
    world.add_body(RigidBody(Vec3(5, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))

    hits = world.raycast_all(Vec3(-5, 0, 0), Vec3(1, 0, 0), max_distance=20)

    assert len(hits) == 1
    _, hit = hits[0]
    assert hit.point.x == pytest.approx(4.5)
    assert hit.normal.to_tuple() == pytest.approx((-1, 0, 0))


# ---- Snapshot / restore ----


def test_snapshot_records_every_body():
    world = World(gravity=(0, 0, 0))
    world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))
    world.add_body(RigidBody(Vec3(5, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))

    snapshot = world.snapshot()

    assert isinstance(snapshot, WorldSnapshot)
    assert len(snapshot) == 2


def test_restore_resets_position_and_velocity(approx):
    world = World(gravity=(0, 0, 0))
    body = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))
    snapshot = world.snapshot()

    body.position = Vec3(9, 9, 9)
    body.linear_velocity = Vec3(1, 2, 3)
    world.restore(snapshot)

    assert body.position.to_tuple() == approx((0, 0, 0))
    assert body.linear_velocity.to_tuple() == approx((0, 0, 0))


def test_restore_skips_bodies_removed_since_the_snapshot(approx):
    world = World(gravity=(0, 0, 0))
    world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))
    b = world.add_body(RigidBody(Vec3(5, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))
    snapshot = world.snapshot()

    world.remove_body(0)  # removes `a`
    b.position = Vec3(9, 9, 9)
    world.restore(snapshot)  # should not raise, just skip the now-gone `a`

    assert b.position.to_tuple() == approx((5, 0, 0))
    assert world.body_count == 1


def test_restore_wakes_the_body(approx):
    world = World(gravity=(0, -9.81, 0))
    world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(5, 0.5, 5), mass=0.0))
    ball = world.add_body(RigidBody.sphere(Vec3(0, 0.5, 0), 0.5, mass=1.0))
    for _ in range(120):
        world.step(1 / 60)
    assert ball.is_sleeping is True

    snapshot = world.snapshot()
    world.restore(snapshot)

    assert ball.is_sleeping is False


def test_worldsnapshot_cannot_be_constructed_directly():
    with pytest.raises(TypeError):
        WorldSnapshot()


# ---- Contact begin/end events ----


def test_contacts_began_fires_when_bodies_start_overlapping():
    world = World(gravity=(0, -9.81, 0))
    ground = world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(5, 0.5, 5), mass=0.0))
    ball = world.add_body(RigidBody.sphere(Vec3(0, 3, 0), 0.5, mass=1.0))

    assert world.contacts_began == []
    fired = False
    for _ in range(200):
        world.step(1 / 60)
        if world.contacts_began:
            fired = True
            pair = world.contacts_began[0]
            positions = {body.position.to_tuple() for body in pair}
            assert ground.position.to_tuple() in positions
            assert ball.position.to_tuple() in positions
            break
    assert fired


def test_contacts_ended_fires_on_bounce_separation():
    world = World(gravity=(0, -9.81, 0))
    world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(5, 0.5, 5), mass=0.0))
    ball = world.add_body(RigidBody.sphere(Vec3(0, 3, 0), 0.5, mass=1.0))
    ball.restitution = 0.9

    saw_began = False
    saw_ended = False
    for _ in range(60):
        world.step(1 / 60)
        if world.contacts_began:
            saw_began = True
        if saw_began and world.contacts_ended:
            saw_ended = True
            break
    assert saw_ended


def test_contact_events_are_empty_with_no_bodies_touching():
    world = World(gravity=(0, 0, 0))
    world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))
    world.add_body(RigidBody(Vec3(10, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))

    world.step(1 / 60)

    assert world.contacts_began == []
    assert world.contacts_ended == []


# ---- Sleeping ----


def test_body_falls_asleep_after_settling():
    world = World(gravity=(0, -9.81, 0))
    world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(5, 0.5, 5), mass=0.0))
    ball = world.add_body(RigidBody.sphere(Vec3(0, 0.5, 0), 0.5, mass=1.0))

    assert ball.is_sleeping is False
    for _ in range(200):
        world.step(1 / 60)
    assert ball.is_sleeping is True


def test_sleeping_body_stops_moving_in_position(approx):
    world = World(gravity=(0, -9.81, 0))
    world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(5, 0.5, 5), mass=0.0))
    ball = world.add_body(RigidBody.sphere(Vec3(0, 0.5, 0), 0.5, mass=1.0))
    for _ in range(200):
        world.step(1 / 60)
    assert ball.is_sleeping is True

    frozen = ball.position
    for _ in range(30):
        world.step(1 / 60)

    assert ball.position.to_tuple() == approx(frozen.to_tuple())


def test_wake_method_resumes_integration(approx):
    world = World(gravity=(0, -9.81, 0))
    world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(5, 0.5, 5), mass=0.0))
    ball = world.add_body(RigidBody.sphere(Vec3(0, 0.5, 0), 0.5, mass=1.0))
    for _ in range(200):
        world.step(1 / 60)
    assert ball.is_sleeping is True

    ball.wake()
    assert ball.is_sleeping is False
    ball.apply_impulse(Vec3(0, 5, 0))
    for _ in range(10):
        world.step(1 / 60)

    assert ball.position.y > 0.6


def test_setting_position_wakes_a_sleeping_body():
    world = World(gravity=(0, -9.81, 0))
    world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(5, 0.5, 5), mass=0.0))
    ball = world.add_body(RigidBody.sphere(Vec3(0, 0.5, 0), 0.5, mass=1.0))
    for _ in range(200):
        world.step(1 / 60)
    assert ball.is_sleeping is True

    ball.position = Vec3(0, 5, 0)

    assert ball.is_sleeping is False


def test_sleeping_body_wakes_when_something_lands_on_it():
    world = World(gravity=(0, -9.81, 0))
    world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(5, 0.5, 5), mass=0.0))
    resting = world.add_body(RigidBody.sphere(Vec3(0, 0.5, 0), 0.5, mass=1.0))
    for _ in range(120):
        world.step(1 / 60)
    assert resting.is_sleeping is True

    world.add_body(RigidBody.sphere(Vec3(0, 4, 0), 0.5, mass=1.0))
    woke = False
    for _ in range(200):
        world.step(1 / 60)
        if not resting.is_sleeping:
            woke = True
            break
    assert woke


def test_sleeping_can_be_disabled():
    world = World(gravity=(0, -9.81, 0))
    world.sleeping_enabled = False
    world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(5, 0.5, 5), mass=0.0))
    ball = world.add_body(RigidBody.sphere(Vec3(0, 0.5, 0), 0.5, mass=1.0))

    for _ in range(200):
        world.step(1 / 60)

    assert ball.is_sleeping is False


def test_sleep_thresholds_are_settable():
    world = World(gravity=(0, 0, 0))
    assert world.sleeping_enabled is True
    assert world.sleep_linear_threshold == pytest.approx(0.05)
    assert world.sleep_angular_threshold == pytest.approx(0.05)
    assert world.sleep_time_threshold == pytest.approx(0.5)

    world.sleep_linear_threshold = 1.0
    world.sleep_angular_threshold = 1.0
    world.sleep_time_threshold = 0.1
    assert world.sleep_linear_threshold == pytest.approx(1.0)
    assert world.sleep_angular_threshold == pytest.approx(1.0)
    assert world.sleep_time_threshold == pytest.approx(0.1)


def test_standalone_body_is_never_sleeping():
    body = RigidBody(Vec3(0, 0, 0), Vec3(0.5, 0.5, 0.5), mass=1.0)
    assert body.is_sleeping is False


def test_static_body_is_never_sleeping():
    world = World(gravity=(0, -9.81, 0))
    ground = world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(5, 0.5, 5), mass=0.0))
    for _ in range(120):
        world.step(1 / 60)
    assert ground.is_sleeping is False


# ---- Sort-and-sweep broad phase (regression: same results as before) ----


def test_many_separated_bodies_all_settle_independently(approx):
    # Balls centered over the ground, not spread from one edge -- a large
    # box vs. a small sphere far from the box's *center* is a known
    # source of extra float32 GJK/EPA imprecision (see
    # docs/limitations.md's round-shape caveat) that's orthogonal to what
    # this test checks -- that the sort-and-sweep broad phase correctly
    # leaves well-separated bodies uncoupled from each other.
    world = World(gravity=(0, -9.81, 0))
    world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(20, 0.5, 20), mass=0.0))
    starts = [i * 3 - 10.5 for i in range(8)]
    balls = [world.add_body(RigidBody.sphere(Vec3(x, 3, 0), 0.5, mass=1.0)) for x in starts]

    for _ in range(300):
        world.step(1 / 60)

    for x0, ball in zip(starts, balls, strict=True):
        assert ball.position.x == approx(x0, abs=0.6)
        assert ball.position.y == approx(0.5, abs=0.1)
    # neighboring balls never touched (stayed near their 3-unit spacing,
    # well outside their combined 1-unit diameter) -- the actual thing
    # this test is checking.
    for i in range(len(balls) - 1):
        gap = balls[i + 1].position.x - balls[i].position.x
        assert gap == approx(3.0, abs=0.5)


def test_filter_joint_still_works_with_sweep_broad_phase():
    world = World(gravity=(0, 0, 0))
    a = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(1, 1, 1), mass=1.0))
    b = world.add_body(RigidBody(Vec3(0.5, 0, 0), Vec3(1, 1, 1), mass=1.0))
    world.add_filter_joint(a, b)

    for _ in range(30):
        world.step(1 / 60)

    assert (b.position - a.position).length() == pytest.approx(0.5)
