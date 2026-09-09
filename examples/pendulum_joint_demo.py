"""Minimal demo: a bob hangs from a fixed anchor via a DistanceJoint and
swings under gravity, using pybox3d's World/RigidBody/DistanceJoint API
end-to-end.

Run with:
    uv run python examples/pendulum_joint_demo.py
"""

from __future__ import annotations

from pybox3d import RigidBody, Vec3, World


def main() -> None:
    world = World(gravity=(0, -9.81, 0))

    anchor = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    bob = world.add_body(RigidBody(Vec3(2, 0, 0), Vec3(0.2, 0.2, 0.2), mass=1.0))
    world.add_joint(anchor, bob, rest_length=2.0)

    dt = 1 / 120
    steps = 360  # 3 seconds

    print(f"{'t':>6} {'x':>8} {'y':>8} {'distance':>10}")
    for step in range(steps):
        world.step(dt)
        if step % 20 == 0:
            t = step * dt
            pos = bob.position
            dist = (bob.position - anchor.position).length()
            print(f"{t:6.2f} {pos.x:8.3f} {pos.y:8.3f} {dist:10.4f}")

    print(f"\nFinal distance from anchor: {(bob.position - anchor.position).length():.4f}")


if __name__ == "__main__":
    main()
