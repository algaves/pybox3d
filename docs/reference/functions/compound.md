# Compound methods

See the [Compound class](../classes/compound.md) for attributes, the
constructor, and the `children` entry format.

## Instance methods

| Method | Signature | Returns | Description |
|---|---|---|---|
| `contains_point` | `contains_point(point: VecLike) -> bool` | `bool` | Whether `point` lies inside any child. |
| `aabb` | `aabb() -> tuple[Vec3, Vec3]` | `(Vec3 min, Vec3 max)` | The union of every child's world-space AABB. |
| `overlaps` | `overlaps(other: ShapeLike) -> ContactInfo \| None` | `ContactInfo \| None` | Overlap test against every child (GJK/EPA, or exact SAT for a `Box3D` child vs. another `Box3D`); `None` if none overlap. |
| `raycast` | `raycast(origin: VecLike, direction: VecLike, max_t: float = ...) -> RayHit \| None` | `RayHit \| None` | The closest hit among all children; `None` if none are hit within `max_t`. |

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
hit = dumbbell.raycast(Vec3(-5, 0, 0), Vec3(1, 0, 0))
if hit is not None:
    print(hit.t, hit.point)
```
