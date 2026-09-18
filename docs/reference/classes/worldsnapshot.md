# WorldSnapshot

An opaque, point-in-time recording of every body's transform/velocity in
a [World](world.md), created via `World.snapshot()` and consumed by
`World.restore()`.

```python
class WorldSnapshot:
    def __len__(self) -> int: ...
```

`WorldSnapshot` cannot be constructed directly -- it only exists as the
result of `World.snapshot()`. It owns a plain copy of the recorded data,
independent of the `World` it was taken from (unlike
[`RigidBody`](rigidbody.md)/[`DistanceJoint`](distancejoint.md)/
[`Joint`](joint.md) handles, it is never "world-backed"): the `World` can
keep changing, bodies can be added or removed, and the snapshot still
reflects exactly what it recorded.

`len(snapshot)` is how many bodies it recorded.

## Simple state recording / replay

```python
from pybox3d import RigidBody, Vec3, World

world = World(gravity=(0, -9.81, 0))
ground = world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(5, 0.5, 5), mass=0.0))
box = world.add_body(RigidBody(Vec3(0, 5, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))

history = []
for _ in range(300):
    world.step(1 / 60)
    history.append(world.snapshot())  # one snapshot per step you want to rewind to

# rewind to an earlier step
world.restore(history[100])
print(box.position.y)  # back to wherever it was 200 steps ago
```

## Notes

- `World.restore()` silently skips any body named in the snapshot that
  has since been removed from the `World` -- it doesn't re-create it.
- Bodies added to the `World` after the snapshot was taken are left
  untouched by `restore()` (not removed).
- `restore()` wakes every restored body (see [World](world.md)'s
  sleeping section) and does not itself advance the simulation -- call
  `World.step()` afterwards as usual to resume from the restored state.
