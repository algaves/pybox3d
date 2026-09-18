# TODO

Also published as [Roadmap](docs/roadmap.md) in the MkDocs site -- keep
both in sync when either changes.

Missing elements for the `pybox3d` / `libbox3d` bindings, tracked against
the topics a mature 3D physics library's docs cover:
<https://box2d.org/documentation3d/topics.html>

Checked items are implemented (possibly in a reduced "v1" form -- see
[Known limitations](docs/limitations.md)); unchecked items don't exist
yet. See [CHANGELOG.md](CHANGELOG.md) for what's landed so far.

## Shapes

- [x] Box (AABB/OBB)
- [x] Sphere
- [x] Capsule
- [x] Convex Hull
- [x] Triangle Mesh
- [x] Height Field
- [x] Compound shape (multiple shapes per body)

## Joints

- [x] Joint infrastructure: a tagged-union `b3_Joint` (mirroring
      `b3_Shape`'s pattern for shapes) backs every kind in one array/
      slot-map, via `World.add_joint`/`add_spherical_joint`/
      `add_revolute_joint`/`add_prismatic_joint`/`add_weld_joint`/
      `add_motor_joint`/`add_wheel_joint`/`add_filter_joint`/
      `add_parallel_joint`/`get_joint`/`remove_joint`, world-backed
      handles, growable storage
- [x] Distance Joint (rigid, center-to-center only)
- [x] Per-body local anchor points on joints (`anchor_a`/`anchor_b`,
      body-local offsets; the world-space anchors are what's held at
      `rest_length`/limits, and the constraint impulse now torques each
      body via its lever arm to the anchor)
- [x] Joint limits (min/max distance) and spring softness/damping
      (`min_length`/`max_length`/`has_limits`, `stiffness`/`damping` on
      `World.add_joint()`/`DistanceJoint`; angle limits don't apply yet --
      no angular joint types exist, see below)
- [x] Spherical (ball-and-socket) Joint (`World.add_spherical_joint()`):
      a 3-axis point constraint holding `anchor_a`/`anchor_b`'s world
      positions coincident, rotation free
- [x] Revolute (hinge) Joint (`World.add_revolute_joint()`): a point
      constraint plus a velocity-only lock on every rotation axis except
      the hinge `axis`, with an optional torque motor around it
      (`enable_motor`/`motor_speed`/`max_motor_effort`); no angle limits
      yet (would need swing-twist angle tracking -- see
      [Known limitations](docs/limitations.md))
- [x] Prismatic (slider) Joint (`World.add_prismatic_joint()`): locks
      every DOF except translation along `axis`, with optional
      `min_translation`/`max_translation` limits and a force motor
- [x] Weld Joint (`World.add_weld_joint()`): full 6-DOF lock (point
      constraint plus a velocity-only lock on every rotation axis)
- [x] Motor Joint (`World.add_motor_joint()`): soft, spring-driven --
      independently springs body B's position toward a target offset
      from body A's frame and its orientation toward a target relative
      rotation, each only while its own stiffness is set
- [x] Wheel Joint (`World.add_wheel_joint()`): a Prismatic-style
      suspension-axis translation freedom (hard-capped or spring-loaded)
      combined with free rotation around a separate axle axis
- [x] Filter Joint (`World.add_filter_joint()`): not a real constraint --
      makes `World.step` skip collision detection/resolution between the
      two named bodies entirely
- [x] Parallel Joint (`World.add_parallel_joint()`): angular-only Weld
      (locks relative orientation, translation stays completely free)
- [x] Joint cleanup on `World.remove_body` (removing a body now also
      removes/invalidates any joint that referenced it)
- [x] Multiple solver iterations per step for joints and contacts
      (`World.solver_iterations`, default 4 -- see
      `examples/joint_chain_demo.py`, whose chain sag is now
      substantially reduced, and [Known limitations](docs/limitations.md)
      for the remaining residual sag)

## World / Simulation

- [x] World (gravity, fixed-`dt` step)
- [x] Contact/touch events: `World.contacts_began`/`.contacts_ended`
      (polled after each `World.step()`, not a registered callback -- a v1
      design choice; see [Known limitations](docs/limitations.md))
- [x] Query API: `World.query_aabb()`/`World.raycast_all()`
- [x] Sort-and-sweep broad-phase (bodies' AABBs sorted and swept along X,
      replacing the original naive O(n²) body-pair scan) -- not the
      dynamic BVH tree this item originally named; see
      [Known limitations](docs/limitations.md) for why a simpler
      algorithm was chosen instead
- [x] Sleeping bodies (`World.sleeping_enabled`/`.sleep_linear_threshold`/
      `.sleep_angular_threshold`/`.sleep_time_threshold`,
      `RigidBody.is_sleeping`/`.wake()`) -- per-body, not full
      connected-component islands; see
      [Known limitations](docs/limitations.md)
- [x] World state recording & replay: `World.snapshot()`/`.restore()`

## Body

- [x] RigidBody (position, orientation, velocities, mass/inertia,
      forces/impulses)
- [x] Per-body restitution/friction actually used in contact resolution
      (combined per-pair via standard mixing rules -- max for
      restitution, geometric mean for friction; `World.default_restitution`/
      `default_friction` removed, no longer needed)
- [x] Angular (torque) contribution to contact impulses (friction is
      full 6-DOF; the separating/restitution impulse stays linear-only
      for solver stability -- see [Known limitations](docs/limitations.md)).
      Joint impulses are now full 6-DOF too (applied at the world-space
      anchor, torquing both bodies) -- see the Joints section above.
- [x] Stable body handles across `World.remove_body` (handles now track
      a generation-checked id, not a raw index, so they can no longer be
      silently misdirected)

## Collision

- [x] Box-box overlap (15-axis SAT) and single-box raycast
- [x] Sphere/capsule/convex-hull/compound collision pairs (generic
      GJK/EPA core, see Shapes)
- [x] Mesh/height-field collision pairs (static-only, per-triangle
      GJK/EPA against the other shape; mesh-vs-mesh and
      mesh-vs-height-field pairs aren't supported)
- [x] Exact edge-edge contact normals (box-box SAT now reports the
      exact edge-edge axis/closest-segment-points whenever it's the true
      minimum-penetration axis, not just a face-based approximation)

## Utilities

- [x] Character Mover (`CharacterMover`): a kinematic move-and-slide
      controller, driven directly each step via `.move(world,
      displacement)` and tested against a `World`'s bodies -- discrete
      push-out + slide, not continuous/swept collision (a fast-moving
      character or thin obstacle can still tunnel through in one large
      step); not a `RigidBody`, never added to a `World`. See
      [Known limitations](docs/limitations.md)
- [x] Debug Draw: `RigidBody.debug_lines()` (a wireframe approximation of
      a body's current shape), `World.debug_contacts()`/
      `.debug_joint_anchors()` -- pure data (lists of `Vec3` pairs), no
      rendering backend
- [x] Ids (opaque handles, e.g. to survive across `remove_body`/
      `remove_joint` swap-removes -- see Body/Joint items above)
