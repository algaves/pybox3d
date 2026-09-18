# Sphere

`Sphere(center, radius)`

A 3D sphere shape.

```python
class Sphere:
    center: Vec3
    radius: float

    def __init__(self, center: VecLike, radius: float) -> None: ...
```

## Attributes

| Attribute | Type | Description |
|---|---|---|
| `center` | `Vec3` | World-space center of the sphere. |
| `radius` | `float` | Sphere radius. |

## Example

```python
from pybox3d import Sphere, Vec3

a = Sphere(Vec3(0, 0, 0), 1.0)
b = Sphere(Vec3(1.5, 0, 0), 1.0)
contact = a.overlaps(b)
if contact is not None:
    print(contact.normal, contact.penetration)

hit = a.raycast(Vec3(-5, 0, 0), Vec3(1, 0, 0))
if hit is not None:
    print(hit.t, hit.point)
```

## Methods

See the [Sphere functions](../functions/sphere.md) page for the full
method reference (`contains_point`, `aabb`, `overlaps`, `raycast`).
`overlaps()` accepts any of [`Box3D`](box3d.md), `Sphere`, or
[`Capsule`](capsule.md) -- see [Known limitations](../../limitations.md)
for the GJK/EPA precision caveat on round shapes.
