# Joint properties

See the [Joint class](../classes/joint.md) for the full attribute list
per kind, the constructor (one of `World`'s `add_*_joint()` methods per
kind), and the v1 scope cut.

`Joint` has no callable methods. `body_a`/`body_b` are read-only
properties giving you live views of the connected bodies, exactly like
[DistanceJoint](distancejoint.md)'s:

## Properties

| Property | Type | Returns | Description |
|---|---|---|---|
| `kind` | `property` | `str` | Which joint kind this is: `"spherical"`, `"revolute"`, `"prismatic"`, `"weld"`, `"motor"`, `"wheel"`, `"filter"`, or `"parallel"`. |
| `body_a` | `property` | `RigidBody` | World-backed handle for the first connected body. |
| `body_b` | `property` | `RigidBody` | World-backed handle for the second connected body. |

Every other attribute (`anchor_a`, `axis`, `enable_motor`, ...) is a
plain read/write field, but only the ones that apply to this handle's
`kind` -- see the [Joint class](../classes/joint.md) page for which
attributes go with which kind. Reading or writing one that doesn't apply
raises `AttributeError`:

```python
joint = world.add_spherical_joint(a, b)
joint.axis  # AttributeError: 'axis' is not available on a spherical joint
```

The remaining joint-management API (`add_spherical_joint`,
`add_revolute_joint`, `add_prismatic_joint`, `add_weld_joint`,
`add_motor_joint`, `add_wheel_joint`, `add_filter_joint`,
`add_parallel_joint`, `get_joint`, `remove_joint`, `joint_count`) lives
on the [World class](../classes/world.md).
