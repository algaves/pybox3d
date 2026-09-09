# Box3D

`Box3D(center, half_extents, orientation=None)`

A 3D box: an axis-aligned bounding box (AABB) when `orientation` is
`None`, an oriented bounding box (OBB) otherwise.

```python
class Box3D:
    center: Vec3
    half_extents: Vec3
    orientation: Quat
    kind: int  # BOX_KIND_AABB or BOX_KIND_OBB

    def __init__(
        self, center: VecLike, half_extents: VecLike, orientation: QuatLike | None = None
    ) -> None: ...
```

`kind` is one of the module-level constants `BOX_KIND_AABB` /
`BOX_KIND_OBB`, reflecting whether an `orientation` was supplied.

## Methods

| Method | Returns | Description |
|---|---|---|
| `contains_point(point)` | `bool` | Whether `point` lies inside the box. |
| `aabb()` | `(Vec3 min, Vec3 max)` | The box's world-space axis-aligned bounding box. |
| `overlaps(other)` | `ContactInfo \| None` | SAT overlap test against another `Box3D`; `None` if disjoint. |
| `raycast(origin, direction, max_t=inf)` | `RayHit \| None` | Ray/box intersection; `None` if no hit within `max_t`. |

### `overlaps` — `ContactInfo`

```python
class ContactInfo(NamedTuple):
    normal: Vec3
    penetration: float
```

`overlaps()` runs a full 15-axis SAT test to decide *whether* two boxes
overlap, but only derives the reported `normal`/`penetration` from face
axes — see [Known limitations](../limitations.md) for the edge-edge case.

### `raycast` — `RayHit`

```python
class RayHit(NamedTuple):
    t: float
    point: Vec3
    normal: Vec3
```

`t` is the ray parameter at the hit (`point = origin + t * direction`).

## Example

```python
from pybox3d import Box3D, Vec3

a = Box3D(Vec3(0, 0, 0), Vec3(1, 1, 1))
b = Box3D(Vec3(1.5, 0, 0), Vec3(1, 1, 1))
contact = a.overlaps(b)
if contact is not None:
    print(contact.normal, contact.penetration)

hit = a.raycast(Vec3(-5, 0, 0), Vec3(1, 0, 0))
if hit is not None:
    print(hit.t, hit.point)
```
