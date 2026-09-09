# Box3D methods

See the [Box3D class](../classes/box3d.md) for attributes and the constructor.

## Instance methods

| Method | Signature | Returns | Description |
|---|---|---|---|
| `contains_point` | `contains_point(point: VecLike) -> bool` | `bool` | Whether `point` lies inside the box. |
| `aabb` | `aabb() -> tuple[Vec3, Vec3]` | `(Vec3 min, Vec3 max)` | The box's world-space axis-aligned bounding box. |
| `overlaps` | `overlaps(other: Box3D) -> ContactInfo \| None` | `ContactInfo \| None` | SAT overlap test; `None` if disjoint. |
| `raycast` | `raycast(origin: VecLike, direction: VecLike, max_t: float = ...) -> RayHit \| None` | `RayHit \| None` | Ray/box intersection; `None` if no hit within `max_t`. |

## `overlaps` return value

Returns a [`ContactInfo`](../classes/contactinfo.md) when the two boxes
are intersecting:

```python
from pybox3d import Box3D, Vec3

a = Box3D(Vec3(0, 0, 0), Vec3(1, 1, 1))
b = Box3D(Vec3(1.5, 0, 0), Vec3(1, 1, 1))
contact = a.overlaps(b)
if contact is not None:
    print(contact.normal, contact.penetration)
```

`overlaps()` runs a full 15-axis SAT test to decide *whether* two boxes
overlap, but only derives the reported `normal`/`penetration` from face
axes. See [Known limitations](../../limitations.md) for the edge-edge
case.

## `raycast` return value

Returns a [`RayHit`](../classes/rayhit.md) when the ray intersects the
box:

```python
hit = a.raycast(Vec3(-5, 0, 0), Vec3(1, 0, 0))
if hit is not None:
    print(hit.t, hit.point)
```

`t` is the ray parameter at the hit (`point = origin + t * direction`).