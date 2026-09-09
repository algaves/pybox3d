# RigidBody methods

See the [RigidBody class](../classes/rigidbody.md) for attributes, the
constructor, and the world-backed handle explanation.

## Instance methods

| Method | Signature | Returns | Description |
|---|---|---|---|
| `apply_force` | `apply_force(force: VecLike, point: VecLike \| None = None) -> None` | `None` | Accumulate a force. |
| `apply_impulse` | `apply_impulse(impulse: VecLike, point: VecLike \| None = None) -> None` | `None` | Accumulate an impulse (applied on the next `World.step`). |
| `clear_accumulators` | `clear_accumulators() -> None` | `None` | Reset accumulated force/impulse to zero. |

## `apply_force` and `apply_impulse`

Both take an optional `point` argument (the world-space point where the
force/impulse is applied). When `point` is `None` (the default), the
force acts on the body's position directly -- i.e. with zero torque
lever arm.

```python
from pybox3d import RigidBody, Vec3

box = RigidBody(Vec3(0, 5, 0), Vec3(0.5, 0.5, 0.5), mass=1.0)

# Push the body forward at its center (no torque)
box.apply_impulse(Vec3(0, 0, 2))

# Push at the top corner (produces torque)
box.apply_impulse(Vec3(0, 0, 2), point=Vec3(0, 5.5, 0))
```

## Accumulator behaviour

Forces and impulses are accumulated and only take effect on the next call
to [`World.step`](../functions/world.md). `clear_accumulators()` resets
both to zero without applying them.