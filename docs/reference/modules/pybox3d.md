# Module: `pybox3d`

The `pybox3d` package re-exports the compiled `pybox3d._pybox3d`
extension's public surface. Everything is available here:

```python
import pybox3d

pybox3d.Vec3, pybox3d.Quat, pybox3d.Box3D, pybox3d.RigidBody
pybox3d.DistanceJoint, pybox3d.World
pybox3d.ContactInfo, pybox3d.RayHit
```

## Module attributes

| Name | Type | Description |
|---|---|---|
| `__version__` | `str` | Version string of the compiled extension. |

## Constants

| Name | Type | Description |
|---|---|---|
| `BOX_KIND_AABB` | `int` | `Box3D.kind` value for an axis-aligned box (no orientation). |
| `BOX_KIND_OBB` | `int` | `Box3D.kind` value for an oriented box (orientation supplied). |

## Type aliases

| Name | Meaning |
|---|---|
| `VecLike` | `Vec3` or any 3-element `Sequence[float]` (tuple, list, ...). |
| `QuatLike` | `Quat` or any 4-element `Sequence[float]`. |

Anywhere the API accepts a vector, `VecLike` works; anywhere it accepts a
quaternion, `QuatLike` works.

## Exceptions

| Exception | Base | Raised when |
|---|---|---|
| `Box3DError` | `RuntimeError` | Any library-level error surfaced to Python. |
| `CapacityError` | `Box3DError` | A `World`'s storage is full (`add_body`/`add_joint`). |

## `__all__`

```python
["BOX_KIND_AABB", "BOX_KIND_OBB", "Box3D", "Box3DError",
 "CapacityError", "ContactInfo", "DistanceJoint", "Quat",
 "RayHit", "RigidBody", "Vec3", "World", "__version__"]
```

## Classes

- [Vec3](../classes/vec3.md)
- [Quat](../classes/quat.md)
- [Box3D](../classes/box3d.md)
- [RigidBody](../classes/rigidbody.md)
- [DistanceJoint](../classes/distancejoint.md)
- [World](../classes/world.md)
- [ContactInfo](../classes/contactinfo.md)
- [RayHit](../classes/rayhit.md)