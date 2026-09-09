"""Minimal demo: a chain of bodies linked end-to-end by DistanceJoints
hangs from a static anchor and settles under gravity, showing
World.add_joint() used repeatedly to build a multi-link constraint chain
rather than just a single pendulum.

Note: World.step solves each joint once per step, sequentially, with no
inner iteration loop (like contact resolution, see README limitations).
For a short chain like this one that converges close enough to look
right, but longer chains visibly sag well past their rest_length as
per-step corrections take multiple frames to propagate from one end to
the other -- see docs/limitations.md.

Run with:
    uv run python examples/joint_chain_demo.py
"""

from __future__ import annotations

from pybox3d import RigidBody, Vec3, World


def main() -> None:
    world = World(gravity=(0, -9.81, 0))

    link_length = 0.5
    link_count = 3

    anchor = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
    links: list[RigidBody] = []
    previous = anchor
    for i in range(link_count):
        y = -(i + 1) * link_length
        link = world.add_body(RigidBody(Vec3(0, y, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
        world.add_joint(previous, link, rest_length=link_length)
        links.append(link)
        previous = link

    dt = 1 / 120
    for _ in range(600):  # 5 seconds: long enough to settle
        world.step(dt)

    print(f"World.joint_count = {world.joint_count}")
    print(f"{'link':>5} {'y':>8} {'dist_from_previous':>20}")
    prev_pos = anchor.position
    for i, link in enumerate(links):
        dist = (link.position - prev_pos).length()
        print(f"{i:5d} {link.position.y:8.3f} {dist:20.4f}")
        prev_pos = link.position


if __name__ == "__main__":
    main()
