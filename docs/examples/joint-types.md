# Joint types

`examples/joint_types_demo.py` builds a small robot-arm rig combining
three of the newer joint kinds: a [Spherical](../reference/classes/joint.md)
shoulder (free rotation), a motorized [Revolute](../reference/classes/joint.md)
elbow, and a [Weld](../reference/classes/joint.md)ed hand:

```python
from pybox3d import RigidBody, Vec3, World

world = World(gravity=(0, 0, 0))

shoulder = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.15, 0.15, 0.15), mass=0.0))
upper_arm = world.add_body(RigidBody(Vec3(1, 0, 0), Vec3(1, 0.15, 0.15), mass=1.0))
forearm = world.add_body(RigidBody(Vec3(3, 0, 0), Vec3(1, 0.15, 0.15), mass=1.0))
hand = world.add_body(RigidBody(Vec3(4.3, 0, 0), Vec3(0.3, 0.3, 0.3), mass=0.5))

world.add_spherical_joint(shoulder, upper_arm, anchor_b=Vec3(-1, 0, 0))
elbow = world.add_revolute_joint(
    upper_arm, forearm, axis=Vec3(0, 0, 1), anchor_a=Vec3(1, 0, 0), anchor_b=Vec3(-1, 0, 0),
    enable_motor=True, motor_speed=1.0, max_motor_effort=200.0,
)
world.add_weld_joint(forearm, hand, anchor_a=Vec3(1, 0, 0), anchor_b=Vec3(-1.3, 0, 0))

for _ in range(240):
    world.step(1 / 120)
```

Run it with:

```sh
uv run python examples/joint_types_demo.py
```

Gravity is off so the elbow motor's effect on relative angular velocity
is the only thing driving the rig and easy to read from the printed
output. See the [A simple robot arm](../tutorials/joint-types.md)
tutorial for the step-by-step version, and
[Joint](../reference/classes/joint.md) for the other five kinds this
demo doesn't use (Prismatic, Motor, Wheel, Filter, Parallel).
