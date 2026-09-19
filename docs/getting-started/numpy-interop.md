# NumPy interop

`pybox3d` depends on NumPy (`>=1.24`), and `Vec3`/`Quat` convert to and
from it. Conversion is all `pybox3d` offers — there are no batch or
array-valued `World`/`RigidBody` APIs; you convert at the boundary and
keep the simulation itself scalar.

## Passing arrays in

`Vec3` and `Quat` are `float32` internally (`b3_real`). Anywhere the API
accepts a `VecLike` or `QuatLike`, a NumPy array works directly — no
conversion call needed:

```python
import numpy as np
from pybox3d import Quat, RigidBody, Vec3, World

world = World(gravity=(0, -9.81, 0))
box = RigidBody(
    np.array([0, 5, 0], dtype=np.float32),
    np.array([0.5, 0.5, 0.5]),
    mass=1.0,
)
body = world.add_body(box)

body.apply_impulse(np.array([1, 0, 0]))       # VecLike
Quat.from_axis_angle(np.array([0, 1, 0]), 1.0)  # axis is VecLike
```

`Vec3.dot()`/`.cross()` and `Quat.rotate_vec3()` accept arrays too:

```python
Vec3(1, 0, 0).dot(np.array([0, 1, 0]))  # 0.0
```

## Zero-copy reads

Both types expose their own memory through the buffer protocol, so
`np.asarray()` is a **zero-copy**, read-only view — no allocation:

```python
v = Vec3(1, 2, 3)
arr = np.asarray(v)          # shape (3,), dtype float32
np.shares_memory(arr, v)     # True
arr.flags.writeable          # False
```

Writes are rejected, because `Vec3.x`/`.y`/`.z` route through the
non-finite guard that rejects NaN/Inf — a writable NumPy view would
bypass it. `bytes(v)`, `memoryview(v)`, and `array.array("f", v)` work
through the same protocol.

`Quat` behaves identically, with shape `(4,)` ordered `(x, y, z, w)`.

## Copying out

To get a normal, writable array of your own, use `to_numpy()`. It always
copies, so mutating the result never affects the source object:

```python
v = Vec3(1, 2, 3)
arr = v.to_numpy()           # float32, shape (3,), writable
arr[0] = 99.0
v.x                          # still 1.0
```

## Building from an array

`from_numpy()` is the explicit constructor counterpart, for symmetry and
discoverability (accepting arrays directly already works):

```python
Vec3.from_numpy([4, 5, 6])              # Vec3(4.0, 5.0, 6.0)
Quat.from_numpy(np.array([0, 0, 0, 1]))  # identity
```

## Precision

Everything is `float32`, matching `b3_real` exactly. Mixing with your own
`float64` arrays is fine: NumPy promotes automatically on the operations
it performs, and `pybox3d` reads the values as Python floats during
conversion.

See the [Vec3](../reference/classes/vec3.md#numpy-interop) and
[Quat](../reference/classes/quat.md#numpy-interop) reference pages for the
per-method signatures.
