"""Minimal demo: a box falls under gravity and comes to rest on a static
"ground" box, using pybox3d's World/RigidBody API end-to-end.

Run with:
    uv run python examples/falling_box_demo.py
"""

from __future__ import annotations

from pybox3d import RigidBody, Vec3, World


def main() -> None:
    world = World(gravity=(0, -9.81, 0))

    ground = RigidBody(Vec3(0, -0.5, 0), Vec3(5, 0.5, 5), mass=0.0)
    world.add_body(ground)

    box = RigidBody(Vec3(0, 5, 0), Vec3(0.5, 0.5, 0.5), mass=1.0)
    handle = world.add_body(box)

    dt = 1 / 60
    steps = 300  # 5 seconds

    print(f"{'t':>6} {'y':>10} {'vy':>10}")
    for step in range(steps):
        world.step(dt)
        if step % 30 == 0:
            t = step * dt
            pos = handle.position
            vel = handle.linear_velocity
            print(f"{t:6.2f} {pos.y:10.4f} {vel.y:10.4f}")

    print(f"\nFinal resting height: {handle.position.y:.4f} (ground top is at y=0)")


if __name__ == "__main__":
    main()
