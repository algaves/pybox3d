# A drifting dumbbell

So far every joint linked a static anchor to a dynamic body. A
[`DistanceJoint`](../reference/classes/distancejoint.md) works just as
well between **two free dynamic bodies** -- giving you a dumbbell.

## 1. Two free bodies, no anchor

```python
from pybox3d import RigidBody, Vec3, World

world = World(gravity=(0, 0, 0))

a = world.add_body(RigidBody(Vec3(-1, 0, 0), Vec3(0.2, 0.2, 0.2), mass=1.0))
b = world.add_body(RigidBody(Vec3(1, 0, 0), Vec3(0.2, 0.2, 0.2), mass=1.0))
world.add_joint(a, b, rest_length=2.0)
```

Gravity is zero so the pair only does what we tell it to. The two bodies
start 2 units apart, matching `rest_length`.

## 2. Give them opposite velocities

```python
drift = Vec3(0.3, 0, 0)
a.linear_velocity = Vec3(0, 1, 0) + drift
b.linear_velocity = Vec3(0, -1, 0) + drift
```

Equal-and-opposite spin velocities plus a shared drift velocity: the pair
should rotate around its common center of mass while that center drifts
in a straight line.

## 3. Simulate and check

```python
for _ in range(240):
    world.step(1 / 120)

print((a.position - b.position).length())  # holds steady at 2.0
```

## What this demonstrates

- The joint constraint is **symmetric**: it pulls both bodies toward
  `rest_length`, not one "attached" body after another.
- Because the masses are equal and the joint is centered, the rotation
  stays balanced around the shared center of mass.
- The `2.0` distance is held even though **both** bodies are moving --
  the joint resolves their relative motion along the rod axis every step.

## Key ideas

- **Joints don't need a static body**: any two bodies in a `World` can be
  constrained.
- **Linear-only v1 scope**: the impulse has no torque term, so this
  dumbbell's free rotation is never damped by the joint itself. See
  [Known limitations](../limitations.md).

You've now covered the whole core API. Browse the
[Examples](../examples/index.md), or dig into the
[Reference](../reference/modules/pybox3d.md) for the full details.