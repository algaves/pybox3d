# Joint chain

`examples/joint_chain_demo.py` links several bodies end-to-end with
[`DistanceJoint`](../reference/classes/distancejoint.md)s hanging from a
static anchor, showing `World.add_joint()` used repeatedly to build a
multi-link chain rather than a single pendulum:

```python
from pybox3d import RigidBody, Vec3, World

world = World(gravity=(0, -9.81, 0))

link_length = 0.5
anchor = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
previous = anchor
for i in range(3):
    y = -(i + 1) * link_length
    link = world.add_body(RigidBody(Vec3(0, y, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    world.add_joint(previous, link, rest_length=link_length)
    previous = link

for _ in range(600):
    world.step(1 / 120)
```

Run it with:

```sh
uv run python examples/joint_chain_demo.py
```

The demo deliberately keeps the chain short (3 links). `World.step`
solves each joint once per step with no inner iteration loop, so a
correction at one joint only reaches its immediate neighbor within a given
step; a longer chain converges to a visibly saggy steady state well past
`rest_length` rather than a taut one -- see
[Known limitations](../limitations.md). The
[A joint chain](../tutorials/joint-chain.md) tutorial explains why.