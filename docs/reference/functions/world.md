# World methods

See the [World class](../classes/world.md) for attributes, the
constructor, and the example.

## Body management

| Method | Signature | Returns | Description |
|---|---|---|---|
| `add_body` | `add_body(body: RigidBody) -> RigidBody` | `RigidBody` | Deep-copies `body` in; returns a new world-backed handle. |
| `get_body` | `get_body(index: int) -> RigidBody` | `RigidBody` | A fresh world-backed handle for the body currently at `index`. |
| `remove_body` | `remove_body(index: int) -> None` | `None` | Swap-remove a body by index; also drops any joint that referenced it. |

## Joint management

| Method | Signature | Returns | Description |
|---|---|---|---|
| `add_joint` | `add_joint(body_a: RigidBody, body_b: RigidBody, rest_length: float \| None = None, anchor_a: VecLike = (0,0,0), anchor_b: VecLike = (0,0,0), min_length: float \| None = None, max_length: float \| None = None, stiffness: float = 0.0, damping: float = 0.0) -> DistanceJoint` | `DistanceJoint` | Connects two bodies with a distance constraint. |
| `add_spherical_joint` | `add_spherical_joint(body_a, body_b, anchor_a=(0,0,0), anchor_b=(0,0,0)) -> Joint` | `Joint` | Ball-and-socket point constraint, rotation free. |
| `add_revolute_joint` | `add_revolute_joint(body_a, body_b, axis, anchor_a=(0,0,0), anchor_b=(0,0,0), enable_motor=False, motor_speed=0.0, max_motor_effort=0.0) -> Joint` | `Joint` | A hinge around `axis` (body-A-local), optionally motorized. |
| `add_prismatic_joint` | `add_prismatic_joint(body_a, body_b, axis, anchor_a=(0,0,0), anchor_b=(0,0,0), min_translation=None, max_translation=None, enable_motor=False, motor_speed=0.0, max_motor_effort=0.0) -> Joint` | `Joint` | A slider along `axis` (body-A-local), optionally limited/motorized. |
| `add_weld_joint` | `add_weld_joint(body_a, body_b, anchor_a=(0,0,0), anchor_b=(0,0,0)) -> Joint` | `Joint` | Full 6-DOF rigid lock. |
| `add_motor_joint` | `add_motor_joint(body_a, body_b, linear_offset=(0,0,0), angular_offset=(0,0,0,1), linear_stiffness=0.0, linear_damping=0.0, angular_stiffness=0.0, angular_damping=0.0) -> Joint` | `Joint` | Soft spring-driven pose target, not a hard constraint. |
| `add_wheel_joint` | `add_wheel_joint(body_a, body_b, suspension_axis, axle_axis, anchor_a=(0,0,0), anchor_b=(0,0,0), min_translation=None, max_translation=None, suspension_stiffness=0.0, suspension_damping=0.0) -> Joint` | `Joint` | Suspension-axis slide + free spin around a separate axle axis. |
| `add_filter_joint` | `add_filter_joint(body_a, body_b) -> Joint` | `Joint` | Not a constraint: disables collision between the pair. |
| `add_parallel_joint` | `add_parallel_joint(body_a, body_b) -> Joint` | `Joint` | Angular-only Weld: orientation locked, translation free. |
| `get_joint` | `get_joint(index: int) -> DistanceJoint \| Joint` | `DistanceJoint \| Joint` | A fresh world-backed handle for the joint currently at `index` (the type depends on its kind). |
| `remove_joint` | `remove_joint(index: int) -> None` | `None` | Swap-remove a joint by index. |

## Queries

| Method | Signature | Returns | Description |
|---|---|---|---|
| `query_aabb` | `query_aabb(min: VecLike, max: VecLike) -> list[RigidBody]` | `list[RigidBody]` | Every body whose AABB overlaps the given box. |
| `raycast_all` | `raycast_all(origin: VecLike, direction: VecLike, max_distance: float = inf) -> list[tuple[RigidBody, RayHit]]` | `list[tuple[RigidBody, RayHit]]` | Every body the ray hits, unsorted. |

## State recording / replay

| Method | Signature | Returns | Description |
|---|---|---|---|
| `snapshot` | `snapshot() -> WorldSnapshot` | `WorldSnapshot` | Records every body's current transform/velocity. |
| `restore` | `restore(snapshot: WorldSnapshot) -> None` | `None` | Resets every body named in `snapshot` to its recorded state and wakes it. |

See [WorldSnapshot](../classes/worldsnapshot.md) for the full semantics.

## Debug Draw

| Method | Signature | Returns | Description |
|---|---|---|---|
| `debug_contacts` | `debug_contacts() -> list[tuple[Vec3, Vec3]]` | `list[tuple[Vec3, Vec3]]` | `(point, normal)` for every currently-overlapping body pair, from a fresh scan -- not filtered by Filter joints. |
| `debug_joint_anchors` | `debug_joint_anchors() -> list[tuple[Vec3, Vec3]]` | `list[tuple[Vec3, Vec3]]` | `(anchor_a, anchor_b)` in world space for every joint, in `get_joint` index order. |

Pure data, no rendering -- see
[RigidBody.debug_lines()](rigidbody.md) for the equivalent per-body
wireframe query, and [Known limitations](../../limitations.md) for both.

## Simulation

| Method | Signature | Returns | Description |
|---|---|---|---|
| `step` | `step(dt: float) -> None` | `None` | Advance the simulation by `dt` seconds. |

## Error cases

`add_body`/every `add_*_joint` can raise `MemoryError` (a `Box3DError`)
if the world's backing storage can't be grown (extremely rare -- storage
is a growable heap array, not a fixed-capacity one; `initial_capacity` is
only a starting hint). Every `add_*_joint` raises `TypeError`/`ValueError`
if `body_a`/`body_b` aren't `RigidBody` instances already belonging to
this same `World` as world-backed handles, or if a numeric argument is
given an invalid value (e.g. `max_length`/`max_translation` below its
`min_*` counterpart, or a negative stiffness/damping/motor-effort).

## `add_joint`

Connects two bodies already in the world. `rest_length` defaults to the
current distance between the two anchor points (`anchor_a`/`anchor_b`,
each a body-local offset defaulting to that body's center) if omitted:

```python
joint = world.add_joint(anchor, bob)                   # uses current distance
joint = world.add_joint(anchor, bob, rest_length=2.0)   # explicit rest_length

# anchored off-center, not at the bodies' centers
joint = world.add_joint(anchor, bob, anchor_b=Vec3(0.5, 0, 0))

# a spring instead of a rigid rod
joint = world.add_joint(anchor, bob, rest_length=2.0, stiffness=30.0, damping=5.0)

# a rope with slack, hard-capped at both ends
joint = world.add_joint(anchor, bob, min_length=1.0, max_length=3.0)
```

See [DistanceJoint](../classes/distancejoint.md) for the full anchor/
limits/spring semantics.

## The other `add_*_joint` methods

Each connects two bodies already in the world with a different kind of
constraint -- see [Joint](../classes/joint.md) for the full semantics of
each:

```python
world.add_spherical_joint(anchor, bob, anchor_b=Vec3(0, 1, 0))
world.add_revolute_joint(hinge, door, axis=Vec3(0, 1, 0), anchor_b=Vec3(-1, 0, 0))
world.add_prismatic_joint(rail, cart, axis=Vec3(1, 0, 0), min_translation=0.0, max_translation=2.0)
world.add_weld_joint(anchor, plate, anchor_b=Vec3(0, 0.5, 0))
world.add_motor_joint(anchor, drone, linear_offset=Vec3(0, 3, 0), linear_stiffness=10.0)
world.add_wheel_joint(chassis, wheel, suspension_axis=Vec3(0, 1, 0), axle_axis=Vec3(0, 0, 1))
world.add_filter_joint(left_leg, right_leg)  # let a ragdoll's limbs overlap
world.add_parallel_joint(a, b)
```

## `step`

`World.step` does the following each tick:

1. Integrate velocities and positions (semi-implicit Euler) for every
   dynamic body.
2. Solve every joint: `solver_iterations` Gauss-Seidel positional-
   correction passes (translation constraints only -- see
   [Known limitations](../../limitations.md) for why rotation locks are
   velocity-only), then `solver_iterations` velocity passes (sequential
   impulse, applied at each joint's world-space anchors -- see
   [DistanceJoint](../classes/distancejoint.md)/
   [Joint](../classes/joint.md)).
3. Detect overlapping body pairs: a sort-and-sweep broad phase (bodies'
   AABBs sorted and swept along X) narrows the O(n²) body-pair space down
   to candidates whose AABBs actually overlap, then each candidate goes
   through exact narrow phase (exact SAT for box-vs-box, generic GJK/EPA
   for every other shape pair, per-triangle GJK/EPA for
   `TriangleMesh`/`HeightField`) -- skipping any pair connected by a
   Filter joint, or where both bodies are static/asleep, entirely.
4. Resolve each detected contact: one positional-correction pass, then
   `solver_iterations` velocity passes (full 6-DOF friction, linear-only
   restitution -- see [Known limitations](../../limitations.md)), using
   each pair's own `RigidBody.restitution`/`.friction` combined via
   standard mixing rules (max, geometric mean).
5. Update `contacts_began`/`contacts_ended` (this step's contact-pair set
   diffed against last step's) and each dynamic body's sleep timer (see
   [Known limitations](../../limitations.md) for both).

`dt` is passed in seconds, so a typical 60 Hz game loop uses
`world.step(1 / 60)`. `solver_iterations` (default 4, settable via the
constructor or the attribute) controls how many passes steps 2 and 4 run
-- see [Known limitations](../../limitations.md) for what it does and
doesn't fix.

## On `remove_body` / `remove_joint`

Both swap-remove: the last item in storage moves into the freed slot. A
*raw index* obtained before the removal may now point at a different
body/joint, but a **handle** (whatever `add_body`/`get_body`/any
`add_*_joint`/`get_joint` returned) tracks a stable, generation-checked
id internally and keeps resolving to the *same* entity regardless --
accessing it after its own entity was removed raises `ValueError` instead
of silently reading the wrong one. `remove_body` also removes (and
invalidates the handles of) every joint that referenced the removed body,
regardless of its kind.
