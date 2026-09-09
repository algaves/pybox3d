# RigidBody

`RigidBody(position, half_extents, mass=0.0)`

A box-shaped rigid body. `mass=0` creates a static body.

```python
class RigidBody:
    position: Vec3
    orientation: Quat
    linear_velocity: Vec3
    angular_velocity: Vec3
    mass: float
    restitution: float
    friction: float
    is_static: bool

    @property
    def inv_mass(self) -> float: ...
    @property
    def shape(self) -> Box3D: ...

    def __init__(self, position: VecLike, half_extents: VecLike, mass: float = 0.0) -> None: ...
```

## Attributes

| Attribute | Type | Description |
|---|---|---|
| `position` | `Vec3` | World-space center of the body. |
| `orientation` | `Quat` | Current rotation. |
| `linear_velocity` | `Vec3` | Current velocity (m/s). |
| `angular_velocity` | `Vec3` | Current angular velocity (rad/s). |
| `mass` | `float` | Mass in kg; `0` marks the body static. |
| `restitution` | `float` | Bounciness (v1: not used in contact resolution — see below). |
| `friction` | `float` | Surface friction (v1: not used in contact resolution — see below). |
| `is_static` | `bool` | `True` when `mass == 0`. |

- `inv_mass` is `0.0` for static bodies (`mass == 0`), otherwise `1 / mass`.
- `shape` is the body's `Box3D`, kept in sync with `position`/`orientation`.
- `restitution`/`friction` are carried per-body for forward compatibility,
  but v1's `World.step` doesn't consult them — see
  [Known limitations](../../limitations.md).

## World-backed handles

A `RigidBody` you construct yourself is a free-standing object. Once
passed to `World.add_body()`, the world **deep-copies it in** and hands
back a *world-backed handle*: reading/writing its attributes reads/writes
the body's live state inside the `World`. `World.get_body(index)` returns
a fresh handle object each call — `is` comparisons don't identify the same
body across two calls, only equal underlying state does.

## Example

```python
from pybox3d import RigidBody, Vec3

box = RigidBody(Vec3(0, 5, 0), Vec3(0.5, 0.5, 0.5), mass=1.0)
box.apply_impulse(Vec3(0, 0, 2))
```

## Methods

See the [RigidBody functions](../functions/rigidbody.md) page for the
full method reference (`apply_force`, `apply_impulse`,
`clear_accumulators`).