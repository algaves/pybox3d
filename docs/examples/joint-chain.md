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

`World.step` resolves each joint over `World.solver_iterations` passes
per step (default 4, settable via `World(..., solver_iterations=...)`),
which substantially reduces -- but doesn't eliminate -- sag past
`rest_length` for longer chains; a short chain like this demo's 3 links
already looks taut at the default. See
[Known limitations](../limitations.md). The
[A joint chain](../tutorials/joint-chain.md) tutorial explains why and
shows the effect of raising `solver_iterations` on a longer chain.