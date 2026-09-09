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

## Attributes

| Attribute | Type | Description |
|---|---|---|
| `x` | `float` | First imaginary component. |
| `y` | `float` | Second imaginary component. |
| `z` | `float` | Third imaginary component. |
| `w` | `float` | Real (scalar) component. |

Supports `*` (quaternion composition) and `==` (component-wise).

## Example

```python
import math
from pybox3d import Quat, Vec3

q = Quat.from_axis_angle(Vec3(0, 1, 0), math.pi / 2)
q.rotate_vec3(Vec3(1, 0, 0))  # ~Vec3(0, 0, -1): a quarter-turn about Y
```

## Methods

See the [Quat functions](../functions/quat.md) page for the full method
reference (`from_axis_angle`, `normalized`, `conjugate`, `rotate_vec3`,
`to_mat3`).