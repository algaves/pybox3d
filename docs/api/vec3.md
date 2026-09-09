# Vec3

A 3D vector `(x, y, z)`.

```python
class Vec3:
    x: float
    y: float
    z: float

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None: ...
```

Supports `+`, `-` (both binary and unary negation), and scalar `*`/right-`*`.
Two vectors compare equal (`==`) component-wise and are hashable.

## Methods

| Method | Returns | Description |
|---|---|---|
| `dot(other)` | `float` | Dot product with `other`. |
| `cross(other)` | `Vec3` | Cross product with `other`. |
| `length()` | `float` | Euclidean length. |
| `length_squared()` | `float` | Squared length (avoids a `sqrt`). |
| `normalized()` | `Vec3` | Unit-length copy; the zero vector normalizes to itself. |
| `to_tuple()` | `tuple[float, float, float]` | `(x, y, z)` as a plain tuple. |

Anywhere a `Vec3` is accepted, a 3-element `Sequence[float]` (e.g. a plain
tuple or list) works too — this is the `VecLike` type alias in the stub.

## Example

```python
from pybox3d import Vec3

a = Vec3(1, 0, 0)
b = Vec3(0, 1, 0)
a.cross(b)          # Vec3(0, 0, 1)
(a + b).length()    # 1.4142135623730951
a.dot((1, 0, 0))     # 1.0 -- plain tuples work as VecLike
```
