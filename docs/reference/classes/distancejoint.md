# DistanceJoint

A distance constraint between two bodies, created via
[`World.add_joint()`](world.md). Anchored at `anchor_a`/`anchor_b`
(body-local offsets, default each body's center) -- holds the distance
between the two world-space anchors at `rest_length`, either rigidly
(default) or as a soft spring, optionally bounded by `min_length`/
`max_length`.

```python
class DistanceJoint:
    rest_length: float
    anchor_a: Vec3
    anchor_b: Vec3
    has_limits: bool
    min_length: float
    max_length: float
    stiffness: float
    damping: float

    @property
    def body_a(self) -> RigidBody: ...
    @property
    def body_b(self) -> RigidBody: ...
```

## Attributes

| Attribute | Type | Description |
|---|---|---|
| `rest_length` | `float` | The target distance (default: distance between the two anchors at creation). |
| `anchor_a` | `Vec3` | `body_a`-local offset from its center that this joint is anchored to. |
| `anchor_b` | `Vec3` | `body_b`-local offset from its center that this joint is anchored to. |
| `has_limits` | `bool` | Whether `min_length`/`max_length` are enforced. |
| `min_length` | `float` | Lower distance bound, only enforced while `has_limits` is `True`. |
| `max_length` | `float` | Upper distance bound, only enforced while `has_limits` is `True`. |
| `stiffness` | `float` | Spring stiffness; `<= 0` (default) means a rigid constraint instead of a spring. |
| `damping` | `float` | Spring damping coefficient, only used while `stiffness > 0`. |

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

## Modes

- **Rigid** (`stiffness <= 0`, the default): holds the distance between
  the two world-space anchors at exactly `rest_length`, ignoring
  `min_length`/`max_length`/`damping` -- a stiff rod. The constraint
  impulse is applied at the anchor points (full 6-DOF: it torques both
  bodies when an anchor isn't at the body's center).
- **Spring** (`stiffness > 0`): applies a Hooke's-law force toward
  `rest_length` (`force = -stiffness * (dist - rest_length) - damping *
  closing_speed`) instead of rigidly enforcing it. With `has_limits` set,
  the anchors are additionally hard-stopped at `min_length`/`max_length`
  (like a bungee cord with a rope backup); without it the spring is
  unbounded.
- **Limits-only "rope"** (`has_limits` set, `stiffness <= 0`): no pull
  toward `rest_length` at all -- the anchors are free to move anywhere
  in `[min_length, max_length]`, with a hard stop at either bound.

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

A rope with slack, hard-capped at both ends, and no pull toward any
particular length in between:

```python
rope = world.add_joint(anchor, bob, min_length=1.0, max_length=3.0)
```

A soft spring instead of a rigid rod:

```python
spring = world.add_joint(anchor, bob, rest_length=2.0, stiffness=30.0, damping=5.0)
```

## v1 scope cut

Only the *velocity*-phase constraint torques both bodies via the anchor's
lever arm -- the separate positional-drift cleanup pass still just
translates each body's whole center along the anchor-to-anchor direction,
not accounting for the anchor's own offset (a v1 simplification,
consistent with contact resolution's own linear-only position pass). Also
still the only joint type in v1 -- no angular constraint of any kind.
See [Known limitations](../../limitations.md).

See `World.get_joint()`/`World.remove_joint()`/`World.joint_count`/
`World.solver_iterations` in the [World reference](world.md) for the rest
of the joint-management API.
