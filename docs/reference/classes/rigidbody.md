# RigidBody

`RigidBody(position, half_extents, mass=0.0)`

A rigid body. The default constructor makes a box-shaped body; use
`RigidBody.sphere()`/`.capsule()`/`.hull()`/`.compound()`/`.mesh()`/
`.heightfield()` for the other shape kinds. `mass=0` creates a static
body (the last two shape kinds are always static -- no `mass` parameter).

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
    def shape(self) -> ShapeLike: ...
    @property
    def is_sleeping(self) -> bool: ...

    def __init__(self, position: VecLike, half_extents: VecLike, mass: float = 0.0) -> None: ...
    @classmethod
    def sphere(cls, position: VecLike, radius: float, mass: float = 0.0) -> RigidBody: ...
    @classmethod
    def capsule(
        cls, position: VecLike, radius: float, half_height: float, mass: float = 0.0
    ) -> RigidBody: ...
    @classmethod
    def hull(cls, position: VecLike, vertices: Sequence[VecLike], mass: float = 0.0) -> RigidBody: ...
    @classmethod
    def compound(
        cls, position: VecLike, children: Sequence[CompoundChildEntry], mass: float = 0.0
    ) -> RigidBody: ...
    @classmethod
    def mesh(cls, position: VecLike, triangles: Sequence[Sequence[VecLike]]) -> RigidBody: ...
    @classmethod
    def heightfield(
        cls, position: VecLike, heights: Sequence[Sequence[float]], cell_size: float = 1.0
    ) -> RigidBody: ...
```

## Attributes

| Attribute | Type | Description |
|---|---|---|
| `position` | `Vec3` | World-space center of the body. |
| `orientation` | `Quat` | Current rotation. |
| `linear_velocity` | `Vec3` | Current velocity (m/s). |
| `angular_velocity` | `Vec3` | Current angular velocity (rad/s). |
| `mass` | `float` | Mass in kg; `0` marks the body static. |
| `restitution` | `float` | Bounciness; combined with the other body's via `max()` for each contact. |
| `friction` | `float` | Surface friction; combined with the other body's via `sqrt(a * b)` for each contact. |
| `is_static` | `bool` | `True` when `mass == 0`. |
| `is_sleeping` | `bool` | `True` while [`World.step()`](world.md) is skipping this body's integration (see `World.sleeping_enabled`); always `False` for a standalone body or a static one (read-only). |

- `inv_mass` is `0.0` for static bodies (`mass == 0`), otherwise `1 / mass`.
- `shape` is the body's [`Box3D`](box3d.md)/[`Sphere`](sphere.md)/
  [`Capsule`](capsule.md)/[`ConvexHull`](convexhull.md)/
  [`Compound`](compound.md)/[`TriangleMesh`](trianglemesh.md)/
  [`HeightField`](heightfield.md) (whichever it was constructed with),
  kept in sync with `position`/`orientation`.
- `restitution`/`friction` are consulted by [`World.step()`](world.md)
  for every contact this body is in (see the mixing rules above); the
  resulting impulse's own linear-vs-6-DOF caveats are in
  [Known limitations](../../limitations.md).

## Other shape kinds

```python
from pybox3d import Box3D, RigidBody, Vec3

ball = RigidBody.sphere(Vec3(0, 5, 0), radius=0.5, mass=1.0)
pill = RigidBody.capsule(Vec3(2, 5, 0), radius=0.3, half_height=0.5, mass=1.0)
cube_vertices = [Vec3(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
rock = RigidBody.hull(Vec3(4, 5, 0), cube_vertices, mass=1.0)
dumbbell = RigidBody.compound(
    Vec3(6, 5, 0),
    [
        (Vec3(-1, 0, 0), Box3D(Vec3(0, 0, 0), Vec3(0.3, 0.3, 0.3))),
        (Vec3(1, 0, 0), Box3D(Vec3(0, 0, 0), Vec3(0.3, 0.3, 0.3))),
    ],
    mass=1.0,
)
ground = RigidBody.mesh(
    Vec3(0, 0, 0),
    [(Vec3(-5, 0, -5), Vec3(5, 0, -5), Vec3(5, 0, 5))],
)  # always static -- no mass parameter
terrain = RigidBody.heightfield(
    Vec3(0, 0, 0), [[0.0, 0.0], [0.0, 0.5]], cell_size=2.0
)  # always static -- no mass parameter
```

Mass/inertia is computed from a closed-form (sphere, capsule) or
approximate (hull, compound -- see [Known limitations](../../limitations.md))
formula for each shape kind. `mesh`/`heightfield` bodies are always
static and have zero mass/inertia.

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
`clear_accumulators`, `wake`, `debug_lines`).