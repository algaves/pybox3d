# Pendulum joint

`examples/pendulum_joint_demo.py` hangs a dynamic body from a static
anchor via a [`DistanceJoint`](../reference/classes/distancejoint.md) and
lets it swing under gravity:

```python
from pybox3d import RigidBody, Vec3, World

world = World(gravity=(0, -9.81, 0))

anchor = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
bob = world.add_body(RigidBody(Vec3(2, 0, 0), Vec3(0.2, 0.2, 0.2), mass=1.0))
world.add_joint(anchor, bob, rest_length=2.0)

for _ in range(360):
    world.step(1 / 120)
```

Run it with:

```sh
uv run python examples/pendulum_joint_demo.py
```

See the [A swinging pendulum](../tutorials/pendulum.md) tutorial for the
step-by-step version.