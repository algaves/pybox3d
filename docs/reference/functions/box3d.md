# Box3D methods

See the [Box3D class](../classes/box3d.md) for attributes and the constructor.

## Instance methods

| Method | Signature | Returns | Description |
|---|---|---|---|
| `contains_point` | `contains_point(point: VecLike) -> bool` | `bool` | Whether `point` lies inside the box. |
| `aabb` | `aabb() -> tuple[Vec3, Vec3]` | `(Vec3 min, Vec3 max)` | The box's world-space axis-aligned bounding box. |
| `overlaps` | `overlaps(other: ShapeLike) -> ContactInfo \| None` | `ContactInfo \| None` | Overlap test (exact SAT vs. another `Box3D`, GJK/EPA otherwise); `None` if disjoint. |
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

`overlaps()` runs a full 15-axis SAT test against another `Box3D` and
reports the exact least-penetrating axis -- a face normal, or (when it's
genuinely the tightest of all 15) an exact edge-edge normal, with
`point` placed at the two edges' closest points rather than approximated
via support points. Against any other shape type, this goes through the
generic GJK/EPA core instead -- see
[Known limitations](../../limitations.md) for its round-shape precision
caveat.

## `raycast` return value

Returns a [`RayHit`](../classes/rayhit.md) when the ray intersects the
box:

```python
hit = a.raycast(Vec3(-5, 0, 0), Vec3(1, 0, 0))
if hit is not None:
    print(hit.t, hit.point)
```

`t` is the ray parameter at the hit (`point = origin + t * direction`).