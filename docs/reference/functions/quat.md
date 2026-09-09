# Quat methods

See the [Quat class](../classes/quat.md) for attributes and the constructor.

## Class methods

| Method | Signature | Returns | Description |
|---|---|---|---|
| `from_axis_angle` | `Quat.from_axis_angle(axis: VecLike, angle_radians: float) -> Quat` | `Quat` | Build a rotation of `angle_radians` around `axis`. |

## Instance methods

| Method | Signature | Returns | Description |
|---|---|---|---|
| `normalized` | `normalized() -> Quat` | `Quat` | Unit-length copy. |
| `conjugate` | `conjugate() -> Quat` | `Quat` | Conjugate `(-x, -y, -z, w)`. |
| `rotate_vec3` | `rotate_vec3(v: VecLike) -> Vec3` | `Vec3` | Rotate vector `v` by this quaternion. |
| `to_mat3` | `to_mat3() -> tuple[float, ...]` | `tuple[float, ...]` | The equivalent 3x3 rotation matrix, as a flat 9-tuple (row-major). |

## Notes

`axis` and `v` accept anything `VecLike` (a `Vec3` or a 3-element
`Sequence[float]`).

`from_axis_angle` normalizes the axis internally; passing an unnormalized
axis does not raise an error.

```python
import math
from pybox3d import Quat, Vec3

q = Quat.from_axis_angle(Vec3(0, 1, 0), math.pi / 2)
q.rotate_vec3(Vec3(1, 0, 0))  # ~Vec3(0, 0, -1): a quarter-turn about Y
```