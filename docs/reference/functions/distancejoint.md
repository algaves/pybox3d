# DistanceJoint properties

See the [DistanceJoint class](../classes/distancejoint.md) for the
constructor and the v1 scope cut.

`DistanceJoint` has no callable methods. The two properties below are
**read/write** via the world-backed handle and give you live views of
the connected bodies:

## Properties

| Property | Type | Returns | Description |
|---|---|---|---|
| `body_a` | `property` | `RigidBody` | World-backed handle for the first connected body. |
| `body_b` | `property` | `RigidBody` | World-backed handle for the second connected body. |

Writing to `body_a.position` or `body_b.linear_velocity` writes the
body's live state inside the [`World`](world.md) directly -- just like
`RigidBody` handles from `World.get_body()`.

```python
from pybox3d import RigidBody, Vec3, World

world = World(gravity=(0, -9.81, 0))
anchor = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
bob = world.add_body(RigidBody(Vec3(2, 0, 0), Vec3(0.2, 0.2, 0.2), mass=1.0))
joint = world.add_joint(anchor, bob, rest_length=2.0)

joint.body_b.position  # live Vec3, updated every World.step
```

The remaining joint-management API (`add_joint`, `get_joint`,
`remove_joint`, `joint_count`) lives on the
[World class](../classes/world.md).