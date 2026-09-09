# A swinging pendulum

A pendulum is two bodies -- a fixed anchor and a swinging bob -- held a
constant distance apart. In `pybox3d` that constraint is a
[`DistanceJoint`](../reference/classes/distancejoint.md).

## 1. Set up the anchor and bob

```python
from pybox3d import RigidBody, Vec3, World

world = World(gravity=(0, -9.81, 0))

anchor = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
bob = world.add_body(RigidBody(Vec3(2, 0, 0), Vec3(0.2, 0.2, 0.2), mass=1.0))
```

The anchor is static (`mass=0`); the bob hangs 2 units away and is
dynamic, so gravity will pull it down.

## 2. Join them

```python
world.add_joint(anchor, bob, rest_length=2.0)
```

`add_joint(body_a, body_b, rest_length)` connects the **centers** of the
two bodies and holds them `rest_length` apart -- a stiff rod, not a
spring. `rest_length` defaults to the distance between the bodies at the
moment you create the joint, so here it would have been `2.0` anyway.

The returned joint handle (like body handles) is world-backed: mutating
`body_a`/`body_b` or `rest_length` writes the world's live state.

## 3. Let it swing

```python
for _ in range(360):
    world.step(1 / 120)
```

At 120 Hz for 3 seconds the bob swings and settles. Two values to watch:

```python
print((bob.position - anchor.position).length())  # stays near 2.0
print(bob.position.y)                            # oscillates, then quiets
```

## Key ideas

- **A joint constrains distance, not direction**: the rod can rotate
  freely, which is exactly what makes a pendulum.
- **`add_joint` needs world-backed bodies**: both arguments must already
  belong to the same `World`.
- **Joints are linear-only in v1**: the impulse holds the distance
  between centers but contributes no torque -- fine for a pendulum.

Next tutorial: [A joint chain](joint-chain.md).