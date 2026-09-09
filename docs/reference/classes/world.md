# World

`World(gravity=(0, -9.81, 0), initial_capacity=8)`

A rigid-body simulation world: naive O(n²) box-box collision detection
and simple impulse-based resolution, plus rigid distance joints (see
[DistanceJoint](distancejoint.md)).

```python
class World:
    gravity: Vec3
    default_restitution: float
    default_friction: float

    @property
    def body_count(self) -> int: ...
    @property
    def joint_count(self) -> int: ...

    def __init__(self, gravity: VecLike = ..., initial_capacity: int = 8) -> None: ...
```

## Attributes

| Attribute | Type | Description |
|---|---|---|
| `gravity` | `Vec3` | Gravitational acceleration applied to all dynamic bodies. |
| `default_restitution` | `float` | Bounciness used for every contact (v1: global only — see below). |
| `default_friction` | `float` | Friction coefficient used for every contact (v1: global only — see below). |

| Property | Type | Description |
|---|---|---|
| `body_count` | `int` | Current number of bodies in the world. |
| `joint_count` | `int` | Current number of joints in the world. |

`World.step` uses `default_restitution`/`default_friction` for *every*
contact rather than per-body mixing rules — see
[Known limitations](../../limitations.md).

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

## Methods

See the [World functions](../functions/world.md) page for the full
method reference (`add_body`, `get_body`, `remove_body`, `add_joint`,
`get_joint`, `remove_joint`, `step`).