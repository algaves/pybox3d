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

`World.step` resolves each joint over `World.solver_iterations` passes
per step (default 4), which substantially reduces chain sag compared to
a single pass but doesn't eliminate it -- a fixed iteration count isn't
the same as solving to convergence. A longer chain still settles a bit
past its `rest_length`:

```python
# A 6-link chain at the default solver_iterations=4 settles to about
# 1.04x its total rest_length; at solver_iterations=1 it's closer to
# 1.5x. Raising solver_iterations tightens this further (~1.01x at 8)
# at the cost of more work per step -- see docs/limitations.md.
world = World(gravity=(0, -9.81, 0), solver_iterations=8)
```

Try it: bump the loop to `range(6)` and compare `solver_iterations=1` vs.
the default vs. `solver_iterations=8`.

## Key ideas

- **Joints compose**: any body can join more than one chain, so
  structures emerge from repeated single joins.
- **Solver iteration matters**: `World.solver_iterations` trades step
  cost for how taut a chain settles -- see [Known limitations](../limitations.md)
  for why it doesn't fully eliminate sag.

Next tutorial: [A drifting dumbbell](dumbbell.md).