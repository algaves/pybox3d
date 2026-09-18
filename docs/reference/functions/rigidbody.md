# RigidBody methods

See the [RigidBody class](../classes/rigidbody.md) for attributes, the
constructor, and the world-backed handle explanation.

## Class methods

| Method | Signature | Returns | Description |
|---|---|---|---|
| `sphere` | `sphere(position: VecLike, radius: float, mass: float = 0.0) -> RigidBody` | `RigidBody` | A standalone sphere-shaped body. |
| `capsule` | `capsule(position: VecLike, radius: float, half_height: float, mass: float = 0.0) -> RigidBody` | `RigidBody` | A standalone capsule-shaped body. |
| `hull` | `hull(position: VecLike, vertices: Sequence[VecLike], mass: float = 0.0) -> RigidBody` | `RigidBody` | A standalone convex-hull-shaped body. |
| `compound` | `compound(position: VecLike, children: Sequence[CompoundChildEntry], mass: float = 0.0) -> RigidBody` | `RigidBody` | A standalone compound-shaped body. |
| `mesh` | `mesh(position: VecLike, triangles: Sequence[Sequence[VecLike]]) -> RigidBody` | `RigidBody` | A standalone, always-static triangle-mesh body. |
| `heightfield` | `heightfield(position: VecLike, heights: Sequence[Sequence[float]], cell_size: float = 1.0) -> RigidBody` | `RigidBody` | A standalone, always-static height-field body. |

## Instance methods

| Method | Signature | Returns | Description |
|---|---|---|---|
| `apply_force` | `apply_force(force: VecLike, point: VecLike \| None = None) -> None` | `None` | Accumulate a force (applied on the next `World.step`). |
| `apply_impulse` | `apply_impulse(impulse: VecLike, point: VecLike \| None = None) -> None` | `None` | Apply an impulse -- changes velocity immediately, not accumulated. |
| `clear_accumulators` | `clear_accumulators() -> None` | `None` | Reset accumulated force/torque to zero. |
| `wake` | `wake() -> None` | `None` | Clear `is_sleeping`/its sleep timer (see [World](../classes/world.md)'s sleeping section); a no-op if not attached to a `World`, or already awake. |
| `debug_lines` | `debug_lines() -> list[tuple[Vec3, Vec3]]` | `list[tuple[Vec3, Vec3]]` | A wireframe approximation of this body's current shape, in world space. |

## `apply_force` and `apply_impulse`

Both take an optional `point` argument (the world-space point where the
force/impulse is applied). When `point` is `None` (the default), the
force acts on the body's position directly -- i.e. with zero torque
lever arm.

```python
from pybox3d import RigidBody, Vec3

box = RigidBody(Vec3(0, 5, 0), Vec3(0.5, 0.5, 0.5), mass=1.0)

# Push the body forward at its center (no torque)
box.apply_impulse(Vec3(0, 0, 2))

# Push at the top corner (produces torque)
box.apply_impulse(Vec3(0, 0, 2), point=Vec3(0, 5.5, 0))
```

## Accumulator behaviour

Forces (and the torque they produce via `point`) are accumulated and
only take effect on the next call to [`World.step`](../functions/world.md);
impulses take effect immediately, changing `linear_velocity`/
`angular_velocity` directly. `clear_accumulators()` resets the pending
force/torque to zero without applying them.

## Waking a sleeping body

`apply_force()`/`apply_impulse()`, and the `position`/`orientation`/
`linear_velocity`/`angular_velocity`/`mass` setters, all wake the body
(see [World](../classes/world.md)'s sleeping section) -- acting on it
directly is taken to mean you want it active. `wake()` itself is for the
rarer case where you just want it awake next step without otherwise
touching it.

## `debug_lines`

Pure data, no rendering: a wireframe approximation of the body's current
shape as world-space line segments (`(start, end)` `Vec3` pairs), meant
to be fed to whatever rendering backend you already have. `Sphere`/
`Capsule` circles are 16-segment polygon approximations; `ConvexHull`
draws its AABB rather than an exact wireframe. See
[Known limitations](../../limitations.md).

```python
for start, end in box.debug_lines():
    my_renderer.draw_line(start, end)
```