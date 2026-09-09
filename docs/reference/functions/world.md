# World methods

See the [World class](../classes/world.md) for attributes, the
constructor, and the example.

## Body management

| Method | Signature | Returns | Description |
|---|---|---|---|
| `add_body` | `add_body(body: RigidBody) -> RigidBody` | `RigidBody` | Deep-copies `body` in; returns a new world-backed handle. |
| `get_body` | `get_body(index: int) -> RigidBody` | `RigidBody` | A fresh world-backed handle for the body at `index`. |
| `remove_body` | `remove_body(index: int) -> None` | `None` | Swap-remove a body by index. |

## Joint management

| Method | Signature | Returns | Description |
|---|---|---|---|
| `add_joint` | `add_joint(body_a: RigidBody, body_b: RigidBody, rest_length: float \| None = None) -> DistanceJoint` | `DistanceJoint` | Connects two bodies already in this world. |
| `get_joint` | `get_joint(index: int) -> DistanceJoint` | `DistanceJoint` | A fresh world-backed handle for the joint at `index`. |
| `remove_joint` | `remove_joint(index: int) -> None` | `None` | Swap-remove a joint by index. |

## Simulation

| Method | Signature | Returns | Description |
|---|---|---|---|
| `step` | `step(dt: float) -> None` | `None` | Advance the simulation by `dt` seconds. |

## Error cases

`add_body` / `add_joint` can raise `CapacityError` (a `Box3DError`) if
the world's fixed-capacity storage is full. `add_joint` raises
`ValueError` if `body_a`/`body_b` aren't already world-backed handles
belonging to this same `World`.

## `add_joint`

Connects two bodies already in the world. `rest_length` defaults to
the current distance between the bodies if omitted:

```python
joint = world.add_joint(anchor, bob)             # uses current distance
joint = world.add_joint(anchor, bob, rest_length=2.0)  # explicit
```

## `step`

`World.step` does the following in order each tick:

1. Integrate velocities and positions
2. Detect box-box overlaps (naive O(n²) pair check)
3. Resolve contacts (global restitution/friction from `World`)
4. Solve distance joints (linear impulse toward `rest_length`)

`dt` is passed in seconds, so a typical 60 Hz game loop uses
`world.step(1 / 60)`.

## On `remove_body` / `remove_joint`

Both swap-remove: the last item in storage moves into the freed slot. Any
handle still holding the old index for that displaced item will silently
resolve to a different body/joint (or raise `ValueError` if the slot is
now out of range). Removing a body does not remove joints that reference
it -- a joint whose body index now points past `body_count`, or at an
unrelated swapped-in body, is a known v1 gap (see
[Known limitations](../../limitations.md)).