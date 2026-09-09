# Dumbbell

`examples/dumbbell_demo.py` joins two *free* dynamic bodies (no static
body at all) with a single `DistanceJoint`, then gives them equal and
opposite spin velocities plus a shared drift velocity. The pair rotates
around its common center of mass while that center drifts in a straight
line, with the joint holding the two bodies at a constant distance
throughout -- demonstrating that a joint works symmetrically between two
moving dynamic bodies, not just an anchor-plus-bob pendulum:

```python
from pybox3d import RigidBody, Vec3, World

world = World(gravity=(0, 0, 0))

a = world.add_body(RigidBody(Vec3(-1, 0, 0), Vec3(0.2, 0.2, 0.2), mass=1.0))
b = world.add_body(RigidBody(Vec3(1, 0, 0), Vec3(0.2, 0.2, 0.2), mass=1.0))
world.add_joint(a, b, rest_length=2.0)

drift = Vec3(0.3, 0, 0)
a.linear_velocity = Vec3(0, 1, 0) + drift
b.linear_velocity = Vec3(0, -1, 0) + drift

for _ in range(240):
    world.step(1 / 120)
```

Run it with:

```sh
uv run python examples/dumbbell_demo.py
```

See the [A drifting dumbbell](../tutorials/dumbbell.md) tutorial for the
step-by-step version.