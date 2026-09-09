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

## Attributes

| Attribute | Type | Description |
|---|---|---|
| `center` | `Vec3` | World-space center of the box. |
| `half_extents` | `Vec3` | Half-widths in each axis (width = `2 * half_extents`). |
| `orientation` | `Quat` | Rotation of the box (`identity` for AABBs). |
| `kind` | `int` | `BOX_KIND_AABB` or `BOX_KIND_OBB` — see [Module: pybox3d](../modules/pybox3d.md). |

`kind` reflects whether an `orientation` was supplied to the constructor.

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

## Methods

See the [Box3D functions](../functions/box3d.md) page for the full method
reference (`contains_point`, `aabb`, `overlaps`, `raycast`).