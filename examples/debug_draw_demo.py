"""Minimal demo: Debug Draw is pure data, not a renderer -- this project
has no GUI/rendering dependency at all, in v1 or otherwise. This script
builds a small scene, then prints what RigidBody.debug_lines() and
World.debug_contacts()/.debug_joint_anchors() return, showing the shape
each API is meant to hand off to whatever rendering backend you already
have (Vec3 line-segment pairs and point/normal pairs, all plain data).

Run with:
    uv run python examples/debug_draw_demo.py
"""

from __future__ import annotations

from pybox3d import RigidBody, Vec3, World


def main() -> None:
    world = World(gravity=(0, -9.81, 0))

    ground = world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(5, 0.5, 5), mass=0.0))
    box = world.add_body(RigidBody(Vec3(0, 0.5, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))
    ball = world.add_body(RigidBody.sphere(Vec3(2, 3, 0), 0.5, mass=1.0))
    world.add_joint(ground, ball, rest_length=3.0)

    print("-- RigidBody.debug_lines(): a wireframe per body, in world space --")
    for name, body in [("ground (Box3D)", ground), ("box", box), ("ball (Sphere)", ball)]:
        lines = body.debug_lines()
        print(f"{name}: {len(lines)} line segments")
        start, end = lines[0]
        print(f"  first segment: {start} -> {end}")

    # Settle the scene, then inspect what's touching and where the
    # joint's anchors ended up.
    for _ in range(120):
        world.step(1 / 60)

    print("\n-- World.debug_contacts(): (point, normal) per overlapping pair --")
    for point, normal in world.debug_contacts():
        print(f"  point={point}  normal={normal}")

    print("\n-- World.debug_joint_anchors(): (anchor_a, anchor_b) per joint --")
    for anchor_a, anchor_b in world.debug_joint_anchors():
        print(f"  anchor_a={anchor_a}  anchor_b={anchor_b}")


if __name__ == "__main__":
    main()
