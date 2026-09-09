# Quat

A rotation quaternion `(x, y, z, w)`; defaults to the identity rotation.

```python
class Quat:
    x: float
    y: float
    z: float
    w: float

    def __init__(
        self, x: float = 0.0, y: float = 0.0, z: float = 0.0, w: float = 1.0
    ) -> None: ...
```

Supports `*` (quaternion composition) and `==` (component-wise).

## Methods

| Method | Returns | Description |
|---|---|---|
| `Quat.from_axis_angle(axis, angle_radians)` *(classmethod)* | `Quat` | Build a rotation of `angle_radians` around `axis`. |
| `normalized()` | `Quat` | Unit-length copy. |
| `conjugate()` | `Quat` | Conjugate `(-x, -y, -z, w)`. |
| `rotate_vec3(v)` | `Vec3` | Rotate vector `v` by this quaternion. |
| `to_mat3()` | `tuple[float, ...]` | The equivalent 3x3 rotation matrix, as a flat 9-tuple (row-major). |

`axis` and `v` accept anything `VecLike` (a `Vec3` or a 3-element
`Sequence[float]`).

## Example

```python
import math
from pybox3d import Quat, Vec3

q = Quat.from_axis_angle(Vec3(0, 1, 0), math.pi / 2)
q.rotate_vec3(Vec3(1, 0, 0))  # ~Vec3(0, 0, -1): a quarter-turn about Y
```
