# Vec3

A 3D vector `(x, y, z)`.

```python
class Vec3:
    x: float
    y: float
    z: float

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None: ...
```

## Attributes

| Attribute | Type | Description |
|---|---|---|
| `x` | `float` | X component. |
| `y` | `float` | Y component. |
| `z` | `float` | Z component. |

Supports `+`, `-` (both binary and unary negation), and scalar `*`/right-`*`.
Two vectors compare equal (`==`) component-wise and are hashable.

Anywhere a `Vec3` is accepted, a 3-element `Sequence[float]` (e.g. a plain
tuple or list) works too — this is the `VecLike` type alias in the stub.
A length-3 NumPy array (`numpy.ndarray` of any float dtype) works as well.

## NumPy interop

`Vec3` is `float32` internally (`b3_real`). It interoperates with NumPy in
three ways:

- `np.asarray(v)` is a **zero-copy**, read-only view over the same memory
  (via the buffer protocol). `bytes(v)`, `memoryview(v)`, and
  `array.array("f", v)` work for the same reason. Writes are rejected.
- `to_numpy()` returns a new, writable `float32` array of shape `(3,)` —
  mutating it does not affect the `Vec3`.
- `from_numpy(arr)` builds a `Vec3` from any array-like of three numbers.

## Example

```python
import numpy as np
from pybox3d import Vec3

a = Vec3(1, 0, 0)
b = Vec3(0, 1, 0)
a.cross(b)                  # Vec3(0, 0, 1)
(a + b).length()            # 1.4142135623730951
a.dot((1, 0, 0))            # 1.0 -- plain tuples work as VecLike
a.dot(np.array([1, 0, 0]))  # 1.0 -- NumPy arrays work too

np.asarray(a)               # array([1., 0., 0.], dtype=float32), zero-copy
a.to_numpy()                # a fresh, writable float32 array
Vec3.from_numpy([4, 5, 6])  # Vec3(4.0, 5.0, 6.0)
```

## Methods

See the [Vec3 functions](../functions/vec3.md) page for the full method
reference (`dot`, `cross`, `length`, `length_squared`, `normalized`,
`to_tuple`, `to_numpy`, `from_numpy`).