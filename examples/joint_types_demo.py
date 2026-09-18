"""Minimal demo: a simple robot-arm rig combining three of the joint
kinds added alongside DistanceJoint -- a shoulder (Spherical), a
motorized elbow (Revolute), and a welded hand (Weld) -- using
World.add_spherical_joint()/add_revolute_joint()/add_weld_joint()
end-to-end.

Gravity is off so the motor's effect on the elbow is the only thing
driving the rig -- with gravity on, the whole arm would swing like a
pendulum from the (free-rotating) shoulder, which would make the
elbow's own relative rotation harder to read from the printed output.

Run with:
    uv run python examples/joint_types_demo.py
"""

from __future__ import annotations

from pybox3d import RigidBody, Vec3, World


def main() -> None:
    world = World(gravity=(0, 0, 0))

    # A fixed shoulder anchor, an upper arm, a motorized forearm, and a
    # hand welded rigidly to the end of the forearm.
    shoulder = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.15, 0.15, 0.15), mass=0.0))
    upper_arm = world.add_body(RigidBody(Vec3(1, 0, 0), Vec3(1, 0.15, 0.15), mass=1.0))
    forearm = world.add_body(RigidBody(Vec3(3, 0, 0), Vec3(1, 0.15, 0.15), mass=1.0))
    hand = world.add_body(RigidBody(Vec3(4.3, 0, 0), Vec3(0.3, 0.3, 0.3), mass=0.5))

    # Shoulder: a ball-and-socket point constraint, free rotation --
    # anchored at the shoulder's own center and the upper arm's -x end.
    world.add_spherical_joint(shoulder, upper_arm, anchor_b=Vec3(-1, 0, 0))

    # Elbow: a hinge around Z (so the forearm swings in the XY plane),
    # anchored at the upper arm's +x end and the forearm's -x end, driven
    # by a motor toward a target relative angular velocity.
    elbow = world.add_revolute_joint(
        upper_arm,
        forearm,
        axis=Vec3(0, 0, 1),
        anchor_a=Vec3(1, 0, 0),
        anchor_b=Vec3(-1, 0, 0),
        enable_motor=True,
        motor_speed=1.0,
        max_motor_effort=200.0,
    )

    # Hand: welded rigidly to the forearm's +x end -- no relative motion
    # at all once settled. Forearm half_extents.x=1 (local +x end at
    # world x=+1 from its center) and the hand's own anchor is 1.3 units
    # from ITS center, so a taut weld puts the hand's center 2.3 units
    # from the forearm's center.
    world.add_weld_joint(forearm, hand, anchor_a=Vec3(1, 0, 0), anchor_b=Vec3(-1.3, 0, 0))

    dt = 1 / 120
    steps = 240  # 2 seconds

    print(f"{'t':>6} {'elbow_rel_speed':>15}")
    for step in range(steps):
        world.step(dt)
        if step % 40 == 0:
            t = step * dt
            relative_speed = elbow.body_b.angular_velocity.z - elbow.body_a.angular_velocity.z
            print(f"{t:6.2f} {relative_speed:15.3f}")

    final_relative_speed = elbow.body_b.angular_velocity.z - elbow.body_a.angular_velocity.z
    print(f"\nFinal elbow relative angular speed: {final_relative_speed:.3f} rad/s")
    print("(motor target: 1.0)")
    hand_offset = (hand.position - forearm.position).length()
    print(f"Hand stayed welded to the forearm: offset length {hand_offset:.3f} (expected ~2.3)")


if __name__ == "__main__":
    main()
