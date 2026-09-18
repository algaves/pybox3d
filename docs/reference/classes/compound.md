# Compound

`Compound(center, children, orientation=None)`

Multiple leaf shapes (`Box3D`/`Sphere`/`Capsule`/`ConvexHull` -- no
nested `Compound`, see [Known limitations](../../limitations.md)) rigidly
attached at fixed local offsets.

```python
class Compound:
    center: Vec3
    orientation: Quat

    @property
    def child_count(self) -> int: ...

    def __init__(
        self,
        center: VecLike,
        children: Sequence[CompoundChildEntry],
        orientation: QuatLike | None = None,
    ) -> None: ...
```

## Attributes

| Attribute | Type | Description |
|---|---|---|
| `center` | `Vec3` | World-space center. |
| `orientation` | `Quat` | Rotation applied to every child's local offset/orientation. |
| `child_count` | `int` | Number of children (read-only). |

## `children` entries

Each entry in `children` is either a 2-tuple `(local_position, shape)` or
a 3-tuple `(local_position, local_orientation, shape)` (orientation
defaults to identity); `shape` is any of `Box3D`/`Sphere`/`Capsule`/
`ConvexHull`. At most `COMPOUND_MAX_CHILDREN` (8) entries.

## Example

```python
from pybox3d import Box3D, Compound, Vec3

dumbbell = Compound(
    Vec3(0, 0, 0),
    [
        (Vec3(-1, 0, 0), Box3D(Vec3(0, 0, 0), Vec3(0.3, 0.3, 0.3))),
        (Vec3(1, 0, 0), Box3D(Vec3(0, 0, 0), Vec3(0.3, 0.3, 0.3))),
    ],
)
```

## Methods

See the [Compound functions](../functions/compound.md) page for the
full method reference. `overlaps()`/`raycast()` consider every child;
`aabb()` is the union of each child's own bounding box.
