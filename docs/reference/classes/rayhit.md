# RayHit

```python
class RayHit(NamedTuple):
    t: float
    point: Vec3
    normal: Vec3
```

A named tuple returned by [`Box3D.raycast()`](box3d.md) when a ray
intersects a box.

## Attributes

| Attribute | Type | Description |
|---|---|---|
| `t` | `float` | Ray parameter at the hit: `point = origin + t * direction`. |
| `point` | `Vec3` | World-space hit point. |
| `normal` | `Vec3` | Surface normal at the hit point. |

## Example

```python
from pybox3d import Box3D, Vec3

a = Box3D(Vec3(0, 0, 0), Vec3(1, 1, 1))
hit = a.raycast(Vec3(-5, 0, 0), Vec3(1, 0, 0))
if hit is not None:
    print(hit.t, hit.point, hit.normal)
```

## Tuple behaviour

`RayHit` is returned as a regular `NamedTuple`:

```python
t, point, normal = a.raycast(origin, direction)
```

`t` is the distance-along-ray at which the ray enters the box. If the
ray misses, `raycast` returns `None`.