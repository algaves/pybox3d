# Quat methods

See the [Quat class](../classes/quat.md) for attributes and the constructor.

## Class methods

| Method | Signature | Returns | Description |
|---|---|---|---|
| `from_axis_angle` | `Quat.from_axis_angle(axis: VecLike, angle_radians: float) -> Quat` | `Quat` | Build a rotation of `angle_radians` around `axis`. |
| `from_numpy` | `Quat.from_numpy(arr: ArrayLike) -> Quat` | `Quat` | Build from any array-like of four numbers `(x, y, z, w)`. |

## Instance methods

| Method | Signature | Returns | Description |
|---|---|---|---|
| `normalized` | `normalized() -> Quat` | `Quat` | Unit-length copy. |
| `conjugate` | `conjugate() -> Quat` | `Quat` | Conjugate `(-x, -y, -z, w)`. |
| `rotate_vec3` | `rotate_vec3(v: VecLike) -> Vec3` | `Vec3` | Rotate vector `v` by this quaternion. |
| `to_mat3` | `to_mat3() -> tuple[float, ...]` | `tuple[float, ...]` | The equivalent 3x3 rotation matrix, as a flat 9-tuple (row-major). |
| `to_numpy` | `to_numpy() -> numpy.ndarray` | `numpy.ndarray` (`float32`, shape `(4,)`) | A new, writable array ordered `(x, y, z, w)`; mutating it does not affect the `Quat`. |

## Notes

`axis` and `v` accept anything `VecLike` (a `Vec3`, a 3-element
`Sequence[float]`, or a length-3 NumPy array).

`from_axis_angle` normalizes the axis internally; passing an unnormalized
axis does not raise an error.

`np.asarray(q)` is a zero-copy, read-only buffer view; `to_numpy()` always
copies (see the [Quat class](../classes/quat.md) page for details).

```python
import math
import numpy as np
from pybox3d import Quat, Vec3

q = Quat.from_axis_angle(Vec3(0, 1, 0), math.pi / 2)
q.rotate_vec3(Vec3(1, 0, 0))  # ~Vec3(0, 0, -1): a quarter-turn about Y
q.rotate_vec3(np.array([1, 0, 0]))  # same, with a NumPy array
```