# World

`World(gravity=(0, -9.81, 0), initial_capacity=8, solver_iterations=4)`

A rigid-body simulation world: a sort-and-sweep broad phase and full
6-DOF impulse-based resolution (using each pair's own
[`RigidBody.restitution`/`.friction`](rigidbody.md), combined via
standard mixing rules), plus nine joint kinds (see
[DistanceJoint](distancejoint.md) and [Joint](joint.md)), contact events,
AABB/raycast queries, per-body sleeping, and state snapshot/restore.

```python
class World:
    gravity: Vec3
    solver_iterations: int
    sleeping_enabled: bool
    sleep_linear_threshold: float
    sleep_angular_threshold: float
    sleep_time_threshold: float

    @property
    def body_count(self) -> int: ...
    @property
    def joint_count(self) -> int: ...
    @property
    def contacts_began(self) -> list[tuple[RigidBody, RigidBody]]: ...
    @property
    def contacts_ended(self) -> list[tuple[RigidBody, RigidBody]]: ...

    def __init__(
        self, gravity: VecLike = ..., initial_capacity: int = 8, solver_iterations: int = 4
    ) -> None: ...
```

## Attributes

| Attribute | Type | Description |
|---|---|---|
| `gravity` | `Vec3` | Gravitational acceleration applied to all dynamic bodies. |
| `solver_iterations` | `int` | Number of velocity-resolution passes per step over the same joint/contact set (>= 1, default 4). |
| `sleeping_enabled` | `bool` | Whether `step()` lets bodies fall asleep (default `True`). |
| `sleep_linear_threshold` | `float` | Linear speed (m/s) a dynamic body must stay below to accumulate its sleep timer (default 0.05). |
| `sleep_angular_threshold` | `float` | Angular speed (rad/s) a dynamic body must stay below to accumulate its sleep timer (default 0.05). |
| `sleep_time_threshold` | `float` | Seconds below both thresholds before a body falls asleep (default 0.5). |

| Property | Type | Description |
|---|---|---|
| `body_count` | `int` | Current number of bodies in the world. |
| `joint_count` | `int` | Current number of joints in the world. |
| `contacts_began` | `list[tuple[RigidBody, RigidBody]]` | Pairs that started overlapping on the last `step()` call (read-only). |
| `contacts_ended` | `list[tuple[RigidBody, RigidBody]]` | Pairs that stopped overlapping on the last `step()` call -- also fires when a pair falls asleep together (read-only). |

`World.step` combines each contact pair's own
[`RigidBody.restitution`](rigidbody.md)/`.friction` (max and geometric
mean respectively) -- see [Known limitations](../../limitations.md) for
the resulting impulse solve's own caveats (restitution is linear-only,
friction is full 6-DOF). `solver_iterations` velocity-resolution passes
run per step over the same detected joint/contact set (and, for joints
only, that many positional passes too) -- see
[Known limitations](../../limitations.md) for what raising it does and
doesn't fix.

## Sleeping

A dynamic body that stays below `sleep_linear_threshold`/
`sleep_angular_threshold` for `sleep_time_threshold` seconds falls
asleep (`RigidBody.is_sleeping` becomes `True`): `step()` stops
integrating it (its position/orientation freeze) until something wakes
it back up -- a joint/contact impulse that leaves it with an
above-threshold velocity, or an explicit `RigidBody.wake()` (also called
implicitly by setting `.position`/`.orientation`/`.linear_velocity`/
`.angular_velocity`/`.mass`, or calling `.apply_force()`/
`.apply_impulse()`). See [Known limitations](../../limitations.md) for
why this is per-body rather than full connected-component islands.

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
print(handle.is_sleeping)  # True -- it's been resting long enough to fall asleep
```

## Methods

See the [World functions](../functions/world.md) page for the full
method reference (`add_body`, `get_body`, `remove_body`, `add_joint` and
the other `add_*_joint` methods, `get_joint`, `remove_joint`,
`query_aabb`, `raycast_all`, `snapshot`, `restore`, `debug_contacts`,
`debug_joint_anchors`, `step`).
