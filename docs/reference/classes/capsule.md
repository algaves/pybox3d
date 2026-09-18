# Capsule

`Capsule(center, radius, half_height, orientation=None)`

A 3D capsule: a line segment of length `2 * half_height` along the local
+Y axis (rotated by `orientation`), swept by `radius`.

```python
class Capsule:
    center: Vec3
    orientation: Quat
    radius: float
    half_height: float

    def __init__(
        self,
        center: VecLike,
        radius: float,
        half_height: float,
        orientation: QuatLike | None = None,
    ) -> None: ...
```

## Attributes

| Attribute | Type | Description |
|---|---|---|
| `center` | `Vec3` | World-space center of the capsule's inner segment. |
| `orientation` | `Quat` | Rotation applied to the local +Y (segment) axis. |
| `radius` | `float` | Radius of the cylindrical body and the two hemispherical caps. |
| `half_height` | `float` | Half-length of the inner segment (not including the caps). |

## Example

```python
from pybox3d import Capsule, Vec3

c = Capsule(Vec3(0, 0, 0), 0.5, 1.0)
hit = c.raycast(Vec3(0, 5, 0), Vec3(0, -1, 0))
if hit is not None:
    print(hit.t, hit.point)  # hits the top hemispherical cap
```

## Methods

See the [Capsule functions](../functions/capsule.md) page for the full
method reference (`contains_point`, `aabb`, `overlaps`, `raycast`).
`overlaps()` accepts any of [`Box3D`](box3d.md), [`Sphere`](sphere.md),
or `Capsule` -- see [Known limitations](../../limitations.md) for the
GJK/EPA precision caveat on round shapes.
