# Joint

Every joint kind other than [DistanceJoint](distancejoint.md), created via
one of `World`'s `add_*_joint()` methods. `kind` says which one; only the
attributes that kind actually uses are settable -- accessing one that
doesn't apply raises `AttributeError`.

```python
class Joint:
    kind: str  # read-only

    anchor_a: Vec3
    anchor_b: Vec3

    axis: Vec3                    # Revolute/Prismatic
    suspension_axis: Vec3         # Wheel
    axle_axis: Vec3               # Wheel

    has_limits: bool              # Prismatic/Wheel
    min_translation: float        # Prismatic/Wheel
    max_translation: float        # Prismatic/Wheel

    enable_motor: bool            # Revolute/Prismatic
    motor_speed: float            # Revolute/Prismatic
    max_motor_effort: float       # Revolute (torque)/Prismatic (force)

    suspension_stiffness: float   # Wheel
    suspension_damping: float     # Wheel

    linear_offset: Vec3           # Motor
    angular_offset: Quat          # Motor
    linear_stiffness: float       # Motor
    linear_damping: float         # Motor
    angular_stiffness: float      # Motor
    angular_damping: float        # Motor

    @property
    def body_a(self) -> RigidBody: ...
    @property
    def body_b(self) -> RigidBody: ...
```

## Kinds

All eight are constructed via a `World` method, not directly (`Joint()`
raises `TypeError`, matching [DistanceJoint](distancejoint.md)).
`anchor_a`/`anchor_b` are body-local offsets from each body's center
(default the center itself); every axis (`axis`, `suspension_axis`,
`axle_axis`) is **body-A-local**, evaluated in body A's current
orientation each step.

- **Spherical** (`World.add_spherical_joint(body_a, body_b, anchor_a=..., anchor_b=...)`) --
  a ball-and-socket: the two world-space anchors are held coincident (a
  3-axis point constraint), rotation is completely free.
- **Revolute** (`World.add_revolute_joint(body_a, body_b, axis, anchor_a=..., anchor_b=..., enable_motor=False, motor_speed=0.0, max_motor_effort=0.0)`) --
  a hinge: the same point constraint as Spherical, plus a lock on every
  rotation axis except `axis`. With `enable_motor` set, drives relative
  angular velocity around `axis` toward `motor_speed` (rad/s), clamped by
  `max_motor_effort` as a per-step torque (N·m) limit.
- **Prismatic** (`World.add_prismatic_joint(body_a, body_b, axis, anchor_a=..., anchor_b=..., min_translation=None, max_translation=None, enable_motor=False, motor_speed=0.0, max_motor_effort=0.0)`) --
  a slider: locks every DOF except translation along `axis`, hard-capped
  at `min_translation`/`max_translation` if either is given (mirroring
  [DistanceJoint](distancejoint.md)'s `min_length`/`max_length`). With
  `enable_motor` set, drives relative linear velocity along `axis` toward
  `motor_speed` (m/s), clamped by `max_motor_effort` as a per-step force
  (N) limit.
- **Weld** (`World.add_weld_joint(body_a, body_b, anchor_a=..., anchor_b=...)`) --
  full 6-DOF lock: the point constraint plus a lock on every rotation
  axis (no motor, no free axis).
- **Motor** (`World.add_motor_joint(body_a, body_b, linear_offset=(0,0,0), angular_offset=(0,0,0,1), linear_stiffness=0.0, linear_damping=0.0, angular_stiffness=0.0, angular_damping=0.0)`) --
  soft, not a hard constraint: independently springs body B's position
  toward `body_a.position + rotate(body_a.orientation, linear_offset)`
  and body B's orientation toward `body_a.orientation * angular_offset`,
  each only while its own stiffness is `> 0`. Useful for a kinematic
  "driven" body you want to nudge toward a moving target rather than
  rigidly attach.
- **Wheel** (`World.add_wheel_joint(body_a, body_b, suspension_axis, axle_axis, anchor_a=..., anchor_b=..., min_translation=None, max_translation=None, suspension_stiffness=0.0, suspension_damping=0.0)`) --
  a Prismatic-style translation freedom along `suspension_axis`, combined
  with free rotation around the separate `axle_axis`. Translation along
  `suspension_axis` is hard-capped if `min_translation`/`max_translation`
  are given, otherwise springs toward zero offset if
  `suspension_stiffness > 0`, otherwise free-slides with no resistance at
  all (there's no default spring -- an un-configured Wheel joint is a
  frictionless piston, not a real suspension).
- **Filter** (`World.add_filter_joint(body_a, body_b)`) -- not a real
  constraint: makes `World.step` skip collision detection/resolution
  between `body_a`/`body_b` entirely, for as long as the joint exists.
- **Parallel** (`World.add_parallel_joint(body_a, body_b)`) --
  angular-only Weld: locks relative orientation (same rotation lock as
  Weld), translation is completely unconstrained.

## Example

```python
from pybox3d import RigidBody, Vec3, World

world = World(gravity=(0, -9.81, 0))

hinge = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.05, 1, 0.05), mass=0.0))
door = world.add_body(RigidBody(Vec3(1, 0, 0), Vec3(1, 1, 0.05), mass=1.0))
joint = world.add_revolute_joint(hinge, door, axis=Vec3(0, 1, 0), anchor_b=Vec3(-1, 0, 0))

for _ in range(300):
    world.step(1 / 120)

print(joint.kind)  # "revolute"
```

## v1 scope cut

Every rotation-locking part of Revolute/Prismatic/Weld/Wheel/Parallel is
velocity-only (no positional/Baumgarte correction for orientation drift),
Revolute has no angle limits, and Motor's linear/angular springs are
fully decoupled -- see [Known limitations](../../limitations.md) for why.

See `World.get_joint()`/`World.remove_joint()`/`World.joint_count`/
`World.solver_iterations` in the [World reference](world.md) for the rest
of the joint-management API.
