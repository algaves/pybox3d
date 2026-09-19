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
| `to_numpy` | `to_numpy() -> numpy.ndarray` | `numpy.ndarray` (`float32`, shape `(3,)`) | A new, writable array; mutating it does not affect the `Vec3`. |

## Class methods

| Method | Signature | Returns | Description |
|---|---|---|---|
| `from_numpy` | `Vec3.from_numpy(arr: ArrayLike) -> Vec3` | `Vec3` | Build from any array-like of three numbers. |

## Notes

`other` accepts anything `VecLike` (a `Vec3`, a 3-element
`Sequence[float]`, or a length-3 NumPy array), so plain tuples, lists, and
arrays work too:

```python
import numpy as np
from pybox3d import Vec3

a = Vec3(1, 0, 0)
a.dot((0, 1, 0))            # 0.0 -- plain tuple works as VecLike
a.dot([0, 1, 0])            # 0.0 -- list works too
a.dot(np.array([0, 1, 0]))  # 0.0 -- NumPy array works too
```

`np.asarray(v)` is a zero-copy, read-only buffer view (see the
[Vec3 class](../classes/vec3.md) page for details); `to_numpy()` always
copies.