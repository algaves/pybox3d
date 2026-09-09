"""Minimal demo: two free dynamic bodies (no static anchor at all) joined
by a DistanceJoint spin around their shared center of mass while that
center drifts in a straight line, showing the constraint holding
symmetrically between two bodies that are *both* dynamic -- not just the
anchor-plus-bob pendulum shape.

Run with:
    uv run python examples/dumbbell_demo.py
"""

from __future__ import annotations

from pybox3d import RigidBody, Vec3, World


def main() -> None:
    world = World(gravity=(0, 0, 0))  # isolate the joint from gravity/drift

    a = world.add_body(RigidBody(Vec3(-1, 0, 0), Vec3(0.2, 0.2, 0.2), mass=1.0))
    b = world.add_body(RigidBody(Vec3(1, 0, 0), Vec3(0.2, 0.2, 0.2), mass=1.0))
    world.add_joint(a, b, rest_length=2.0)

    # Equal-and-opposite spin velocities plus a shared drift velocity: the
    # pair should rotate about its center of mass while that center moves
    # in a straight line, with the joint holding the two bodies apart at a
    # constant distance throughout.
    drift = Vec3(0.3, 0, 0)
    a.linear_velocity = Vec3(0, 1, 0) + drift
    b.linear_velocity = Vec3(0, -1, 0) + drift

    dt = 1 / 120
    steps = 240  # 2 seconds

    print(f"{'t':>6} {'center_x':>10} {'center_y':>10} {'distance':>10}")
    for step in range(steps):
        world.step(dt)
        if step % 20 == 0:
            t = step * dt
            center = (a.position + b.position) * 0.5
            dist = (b.position - a.position).length()
            print(f"{t:6.2f} {center.x:10.4f} {center.y:10.4f} {dist:10.4f}")


if __name__ == "__main__":
    main()
