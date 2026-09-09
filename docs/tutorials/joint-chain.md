# A joint chain

A chain is just a pendulum repeated: link several bodies end-to-end with
[`DistanceJoint`](../reference/classes/distancejoint.md)s, each hanging
from the previous one.

## 1. Build the bodies

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
```

Each iteration adds one hanging link and joins it to the body above it.
`previous` chains the joints: `anchor-link1`, then `link1-link2`, then
`link2-link3`.

## 2. Simulate

```python
for _ in range(600):
    world.step(1 / 120)
```

The chain settles under gravity into a hanging curve.

## 3. A real v1 limitation

The demo deliberately keeps the chain to **3 links**. `World.step` solves
each joint **once per step** with no inner iteration loop, so a
correction at one joint only reaches its *immediate* neighbor within a
given step. A longer chain converges to a visibly saggy steady state well
past its `rest_length` instead of a taut one -- this is a known
[limitation](../limitations.md), not a bug in your code:

```python
# A naive 6-link chain sags badly: the topmost link can end up
# stretched to ~1.8x its rest_length.
```

Try it: bump the loop to `range(6)` and watch the distance between the
anchor and the bottom link grow.

## Key ideas

- **Joints compose**: any body can join more than one chain, so
  structures emerge from repeated single joins.
- **Solver iteration matters**: one pass per step is fine for short
  chains and degrades gracefully for longer ones.

Next tutorial: [A drifting dumbbell](dumbbell.md).