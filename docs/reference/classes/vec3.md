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

## Example

```python
from pybox3d import Vec3

a = Vec3(1, 0, 0)
b = Vec3(0, 1, 0)
a.cross(b)          # Vec3(0, 0, 1)
(a + b).length()    # 1.4142135623730951
a.dot((1, 0, 0))     # 1.0 -- plain tuples work as VecLike
```

## Methods

See the [Vec3 functions](../functions/vec3.md) page for the full method
reference (`dot`, `cross`, `length`, `length_squared`, `normalized`,
`to_tuple`).