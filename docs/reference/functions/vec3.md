# Vec3 methods

See the [Vec3 class](../classes/vec3.md) for attributes and the constructor.

## Instance methods

| Method | Signature | Returns | Description |
|---|---|---|---|
| `dot` | `dot(other: VecLike) -> float` | `float` | Dot product with `other`. |
| `cross` | `cross(other: VecLike) -> Vec3` | `Vec3` | Cross product with `other`. |
| `length` | `length() -> float` | `float` | Euclidean length. |
| `length_squared` | `length_squared() -> float` | `float` | Squared length (avoids a `sqrt`). |
| `normalized` | `normalized() -> Vec3` | `Vec3` | Unit-length copy; the zero vector normalizes to itself. |
| `to_tuple` | `to_tuple() -> tuple[float, float, float]` | `tuple[float, float, float]` | `(x, y, z)` as a plain tuple. |

## Notes

`other` accepts anything `VecLike` (a `Vec3` or a 3-element
`Sequence[float]`), so plain tuples and lists work too:

```python
from pybox3d import Vec3

a = Vec3(1, 0, 0)
a.dot((0, 1, 0))      # 0.0 -- plain tuple works as VecLike
a.dot([0, 1, 0])      # 0.0 -- list works too
```