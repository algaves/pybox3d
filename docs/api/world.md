# World

`World(gravity=(0, -9.81, 0), initial_capacity=8)`

A rigid-body simulation world: naive O(n²) box-box collision detection
and simple impulse-based resolution.

```python
class World:
    gravity: Vec3
    default_restitution: float
    default_friction: float

    @property
    def body_count(self) -> int: ...

    def __init__(self, gravity: VecLike = ..., initial_capacity: int = 8) -> None: ...
```

`World.step` uses `default_restitution`/`default_friction` for *every*
contact rather than per-body mixing rules — see
[Known limitations](../limitations.md).

## Methods

| Method | Returns | Description |
|---|---|---|
| `add_body(body)` | `RigidBody` | Deep-copies `body` in; returns a new world-backed handle. |
| `get_body(index)` | `RigidBody` | A fresh world-backed handle for the body at `index`. |
| `remove_body(index)` | `None` | Swap-remove; see [RigidBody](rigidbody.md) on stale handles. |
| `step(dt)` | `None` | Advance the simulation by `dt` seconds. |

`add_body` can raise `CapacityError` (a `Box3DError`) if the world's
fixed-capacity body storage is full.

### On `remove_body`

Removing a body swap-removes it: the last body in storage moves into the
freed slot. Any `RigidBody` handle still holding the old index for that
displaced body will silently resolve to a different body (or raise
`ValueError` if the slot is now out of range).

## Example

```python
from pybox3d import RigidBody, Vec3, World

world = World(gravity=(0, -9.81, 0))

ground = RigidBody(Vec3(0, -0.5, 0), Vec3(5, 0.5, 5), mass=0.0)
world.add_body(ground)

box = RigidBody(Vec3(0, 5, 0), Vec3(0.5, 0.5, 0.5), mass=1.0)
handle = world.add_body(box)

for _ in range(300):
    world.step(1 / 60)

print(handle.position.y)  # settles near 0.5 (half_extents.y), resting on the ground
```

See [Examples](../examples.md) for the full runnable demo.
