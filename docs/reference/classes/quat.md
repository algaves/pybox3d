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

Anywhere a `Quat` is accepted, a 4-element `Sequence[float]` works too —
this is the `QuatLike` type alias in the stub. A length-4 NumPy array
(`numpy.ndarray` of any float dtype) works as well.

## NumPy interop

Like `Vec3`, `Quat` is `float32` internally and interoperates with NumPy:

- `np.asarray(q)` is a **zero-copy**, read-only view over the same memory
  (via the buffer protocol); writes are rejected.
- `to_numpy()` returns a new, writable `float32` array of shape `(4,)`,
  ordered `(x, y, z, w)` — mutating it does not affect the `Quat`.
- `from_numpy(arr)` builds a `Quat` from any array-like of four numbers.

## Example

```python
import math
import numpy as np
from pybox3d import Quat, Vec3

q = Quat.from_axis_angle(Vec3(0, 1, 0), math.pi / 2)
q.rotate_vec3(Vec3(1, 0, 0))  # ~Vec3(0, 0, -1): a quarter-turn about Y

np.asarray(q)                  # array([0., 0.70710677, 0., 0.70710677], dtype=float32)
Quat.from_numpy([0, 0, 0, 1])  # Quat(0.0, 0.0, 0.0, 1.0)
```

## Methods

See the [Quat functions](../functions/quat.md) page for the full method
reference (`from_axis_angle`, `normalized`, `conjugate`, `rotate_vec3`,
`to_mat3`, `to_numpy`, `from_numpy`).