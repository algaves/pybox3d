"""Minimal demo: a CharacterMover falls onto a ground plane, then walks
into and slides along a wall, using World.query-free discrete push-out +
slide collision -- CharacterMover is driven directly by the caller each
step (not a RigidBody, never added to the World) and only ever tested
against the World's other bodies for collision.

Run with:
    uv run python examples/character_mover_demo.py
"""

from __future__ import annotations

from pybox3d import Box3D, CharacterMover, RigidBody, Vec3, World


def main() -> None:
    world = World(gravity=(0, -9.81, 0))
    world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(10, 0.5, 10), mass=0.0))  # ground
    world.add_body(RigidBody(Vec3(3, 1, 0), Vec3(0.5, 2, 10), mass=0.0))  # wall at x=3

    shape = Box3D(Vec3(0, 0, 0), Vec3(0.4, 0.9, 0.4))
    player = CharacterMover(Vec3(0, 5, 0), shape)

    dt = 1 / 60
    gravity = Vec3(0, -9.81, 0)

    print("-- falling --")
    print(f"{'t':>6} {'y':>8} {'grounded':>10}")
    for step in range(90):  # 1.5 seconds: falls and lands
        player.velocity = player.velocity + gravity * dt
        player.move(world, player.velocity * dt)
        if step % 15 == 0:
            print(f"{step * dt:6.2f} {player.position.y:8.3f} {player.is_grounded!s:>10}")

    # is_grounded reflects only the most recent move() call, not a
    # sticky "on the ground" flag -- see CharacterMover's docs. It reads
    # True once the fall resolves a downward contact, then False again on
    # a step with nothing left to push out of (already resting).
    print(f"\nLanded: y={player.position.y:.3f}")

    print("\n-- walking into the wall --")
    print(f"{'t':>6} {'x':>8} {'z':>8}")
    walk_velocity = Vec3(2.0, 0, 2.0)  # diagonally toward and along the wall
    for step in range(90):  # 1.5 seconds
        player.velocity = Vec3(walk_velocity.x, player.velocity.y, walk_velocity.z)
        player.move(world, player.velocity * dt)
        if step % 15 == 0:
            print(f"{step * dt:6.2f} {player.position.x:8.3f} {player.position.z:8.3f}")

    print(f"\nStopped by the wall at x={player.position.x:.3f} (wall face is at x=2.5),")
    print(f"but kept sliding along it: z={player.position.z:.3f} (not stuck at 0)")


if __name__ == "__main__":
    main()
