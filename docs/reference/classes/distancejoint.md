# DistanceJoint

A rigid distance constraint between two bodies' centers, created via
[`World.add_joint()`](world.md). Holds the distance between
`body_a.position` and `body_b.position` at `rest_length` -- a stiff rod,
not a spring (no softness, no min/max limits).

```python
class DistanceJoint:
    rest_length: float

    @property
    def body_a(self) -> RigidBody: ...
    @property
    def body_b(self) -> RigidBody: ...
```

## Attributes

| Attribute | Type | Description |
|---|---|---|
| `rest_length` | `float` | The target distance (default: distance between the bodies at creation). |

## Properties

| Property | Type | Description |
|---|---|---|
| `body_a` | `RigidBody` | World-backed handle for the first body. |
| `body_b` | `RigidBody` | World-backed handle for the second body. |

`DistanceJoint` cannot be constructed directly -- it only exists bound to
two specific bodies inside a `World`, created via `World.add_joint()`.
Like [`RigidBody`](rigidbody.md) handles from `World.add_body()`/
`World.get_body()`, `body_a`/`body_b` are fresh **world-backed** handles
each access: reading/writing them reads/writes the connected body's live
state inside the `World`.

## v1 scope cut

Anchored at body centers only -- there's no per-body local anchor offset
(e.g. attaching to a corner rather than the center), and like contact
resolution, the joint impulse is linear-only with no angular/torque
contribution. See [Known limitations](../../limitations.md).

## Example

```python
from pybox3d import RigidBody, Vec3, World

world = World(gravity=(0, -10, 0))

anchor = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
bob = world.add_body(RigidBody(Vec3(2, 0, 0), Vec3(0.2, 0.2, 0.2), mass=1.0))
joint = world.add_joint(anchor, bob, rest_length=2.0)  # rest_length defaults
                                                        # to their current
                                                        # distance apart if omitted

for _ in range(300):
    world.step(1 / 120)

print((bob.position - anchor.position).length())  # stays close to 2.0
```

See `World.get_joint()`/`World.remove_joint()`/`World.joint_count` in the
[World reference](world.md) for the rest of the joint-management API.