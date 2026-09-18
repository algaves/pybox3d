# CharacterMover

`CharacterMover(position, shape)`

A kinematic move-and-slide character controller, driven directly by the
caller each step (via `move()`) and tested against a
[World](world.md)'s bodies for collision. It is not a
[`RigidBody`](rigidbody.md) and is never added to a `World`: gravity,
joints, and contact response between the character and anything else are
all entirely up to the caller -- `CharacterMover` only ever does the
"move by this much, then push me back out of anything I now overlap"
part.

```python
class CharacterMover:
    position: Vec3
    velocity: Vec3
    skin_width: float
    max_slide_iterations: int
    ground_normal_min_y: float

    @property
    def is_grounded(self) -> bool: ...
    @property
    def shape(self) -> ShapeLike: ...

    def __init__(self, position: VecLike, shape: ShapeLike) -> None: ...
    def move(self, world: World, displacement: VecLike) -> None: ...
```

## Attributes

| Attribute | Type | Description |
|---|---|---|
| `position` | `Vec3` | World-space position. Setting it also keeps `shape` in sync. |
| `velocity` | `Vec3` | Current velocity; `move()` removes any into-a-surface component it resolves. |
| `skin_width` | `float` | Depenetration margin beyond exact contact, added to every push-out (default `0.01`). |
| `max_slide_iterations` | `int` | Discrete push-out passes per `move()` call, `>= 1` (default `4`). |
| `ground_normal_min_y` | `float` | `dot(resolved contact normal, +Y)` threshold for `is_grounded` (default `0.5`). |

## Properties

| Property | Type | Description |
|---|---|---|
| `is_grounded` | `bool` | Whether the last `move()` call resolved a contact whose normal pointed "up" enough. Not sticky between calls -- call `move()` every step (even with a zero displacement) if you need this to stay current (read-only). |
| `shape` | `ShapeLike` | The controller's current collision shape as a snapshot, kept in sync with `position` (read-only). |

`shape` accepts any of the seven shape types (`Box3D`, `Sphere`,
`Capsule`, `ConvexHull`, `Compound`, `TriangleMesh`, `HeightField`) at
construction time, though a capsule or box is the usual choice for a
character.

## `move()`

```python
mover.move(world, displacement)
```

1. Moves the controller by `displacement` -- the caller's own
   responsibility to fold in gravity/input for this step; there's no
   separate integration here, unlike `RigidBody`.
2. Runs up to `max_slide_iterations` passes: each finds the single
   worst-penetrating overlap against `world`'s bodies, pushes the
   controller out along that overlap's normal by
   `penetration + skin_width`, and removes that normal's into-the-
   surface component from `velocity` -- so a caller deriving next step's
   `displacement` from `velocity` naturally slides along an obstacle
   instead of pushing back into it.
3. Sets `is_grounded` from whether any pass resolved a contact with a
   normal pointing "up" enough.

## Example

```python
from pybox3d import Box3D, CharacterMover, RigidBody, Vec3, World

world = World(gravity=(0, -9.81, 0))
world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(10, 0.5, 10), mass=0.0))

shape = Box3D(Vec3(0, 0, 0), Vec3(0.4, 0.9, 0.4))
player = CharacterMover(Vec3(0, 5, 0), shape)

dt = 1 / 60
gravity = Vec3(0, -9.81, 0)
for _ in range(200):
    player.velocity = player.velocity + gravity * dt
    player.move(world, player.velocity * dt)

print(player.position.y)  # settled on the ground, ~0.9 (half_extents.y)
print(player.is_grounded)  # True
```

## v1 scope cut

Discrete collision only -- `move()` tests overlaps *after* moving, not a
continuous/swept sweep-test-before-you-move, the same caveat every other
collision test in this library has. See
[Known limitations](../../limitations.md).
