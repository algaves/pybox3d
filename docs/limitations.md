# Known v1 limitations

These are deliberate scope cuts for a "basic" first version, not bugs:

- **Single-point contacts, no persistent manifold**: every `overlaps()`
  result (`ContactInfo`) carries exactly one representative contact
  point (each shape's support point along the normal, averaged), not a
  full multi-point manifold. There's also no warm-starting between
  simulation steps.
- **Approximate normal/penetration for round shapes**: any pair involving
  a `Sphere` or `Capsule` (i.e. anything other than box-vs-box, which
  keeps its exact SAT) is resolved by a generic GJK/EPA core. On
  perfectly round shapes (e.g. sphere-vs-sphere) the polytope EPA
  refines only approximates the true curved surface, so the reported
  normal/penetration can be off by a few percent -- fine for iterative
  contact response, but `overlaps()` shouldn't be treated as an exact
  geometric query for round shapes. This imprecision scales with the
  *absolute size* of the shapes involved (float32's relative precision
  gets coarser for larger numbers): a sphere resting on a very large
  static box (e.g. `half_extents` in the tens of meters) can pick up a
  small but persistent rolling drift over hundreds of steps from a
  contact point that isn't quite centered, even with zero initial
  horizontal velocity -- keep large "ground plane"-style boxes to a
  scale proportional to what's resting on them, not orders of magnitude
  bigger, if this matters for your scene.
- **`ConvexHull` accepts only a pre-computed convex vertex set**: v1 has
  no incremental/quickhull construction from an arbitrary point cloud --
  the caller must already supply a convex hull's vertices (at most
  `HULL_MAX_VERTICES`, currently 32, since shapes have no heap
  allocation -- see [`RigidBody`](reference/classes/rigidbody.md)'s
  no-internal-heap-pointers design). `ConvexHull.raycast()` is also only
  approximated via the hull's AABB rather than its exact surface, since
  that needs face data this v1 doesn't compute.
- **`ConvexHull`/`Compound` inertia is approximate**: a hull's mass/
  inertia is computed as if it were a solid box of the hull's own local
  AABB (exact convex inertia needs a tetrahedron decomposition of the
  hull's volume); a compound's is the sum of each child's own inertia
  (mass split equally across children) offset by the parallel-axis
  theorem using only the child's position, not its local orientation.
- **`Compound` doesn't support nesting**: a compound's children can be
  `Box3D`/`Sphere`/`Capsule`/`ConvexHull`, but not another `Compound` --
  see `box3d/compound.h` for why (the no-heap-pointers constraint above
  makes a shape's size depend on itself for a self-nesting union).
- **`TriangleMesh`/`HeightField` are always static**: `RigidBody.mesh()`/
  `.heightfield()` take no `mass` parameter, and setting `.mass` on one
  afterwards raises `ValueError` -- these shapes are only meant as fixed
  level geometry. `contains_point()` always returns `False` (no
  well-defined "inside" for an open, possibly inconsistently-wound
  triangle soup without validating a closed manifold, which v1 doesn't
  do), and `raycast()` is approximated via the shape's AABB rather than
  its exact triangles, same as `ConvexHull.raycast()`. Both have a fixed
  size cap (`MESH_MAX_TRIANGLES` = 64; `HEIGHTFIELD_MAX_ROWS`/`_COLS` =
  16), same no-heap-allocation rationale as `ConvexHull`/`Compound`.
  Collision only supports mesh/height-field vs. a convex shape
  (`Box3D`/`Sphere`/`Capsule`/`ConvexHull`/`Compound`) -- mesh-vs-mesh,
  mesh-vs-height-field, and height-field-vs-height-field pairs always
  report no overlap (both sides being static-only, this never comes up
  in `World.step()`, only if called directly via `.overlaps()`).
- **Contact restitution is linear-only; friction is full 6-DOF**: a
  contact's separating/restitution impulse doesn't torque either body,
  but its friction impulse does (e.g. a sliding ball correctly picks up
  rolling spin). This asymmetry is deliberate: a fully coupled 6-DOF
  normal-constraint solve (closing velocity measured *at* the contact
  point, torquing both bodies) is the textbook-correct approach and was
  tried, but this project's single-pass, non-iterative, non-warm-started
  solver isn't stable with it for a body resting on an offset contact
  point (e.g. an upright capsule balanced on its round cap) -- any
  existing rotation feeds back into the next step's closing-velocity
  reading, and with no iteration to damp it, small errors compound into
  runaway spin over hundreds of steps instead of settling. Restitution
  below a small closing-speed threshold (matching Box2D's
  `b2_velocityThreshold` technique) is also suppressed outright, to stop
  a resting contact's frame-to-frame normal/point noise (see above) from
  reading as a repeated tiny "bounce."
- **Angular-locking joints (Revolute/Prismatic/Weld/Wheel/Parallel) are
  velocity-only, no positional (Baumgarte) drift correction**: every
  translation-constraining part of a joint (Distance, or the point/
  point-on-line part of Spherical/Revolute/Prismatic/Weld/Wheel) gets
  both a positional cleanup pass and a velocity constraint, matching
  contact resolution's own two-phase pattern. The *rotation*-locking part
  of Revolute (non-hinge axes)/Prismatic (all 3 axes)/Weld (all 3 axes)/
  Wheel (non-axle axes)/Parallel (all 3 axes) only ever cancels relative
  *angular velocity* along the locked axes -- it actively resists further
  drift under load, but there's no corrective pull back to alignment if
  the two bodies are already misaligned (e.g. from accumulated
  floating-point error over many steps, or a hard external impulse). See
  [DistanceJoint](reference/classes/distancejoint.md)/[Joint](reference/classes/joint.md).
- **Revolute has no angle limits**: `enable_motor`/`motor_speed`/
  `max_motor_effort` drive relative angular velocity around the hinge
  axis, but there's no `min_angle`/`max_angle` the way Prismatic/Wheel
  have `min_translation`/`max_translation` -- tracking a hinge's
  accumulated rotation angle needs swing-twist quaternion decomposition,
  not implemented in v1. See `TODO.md`.
- **Motor Joint's linear and angular springs are fully decoupled**: each
  applies a pure linear-only or pure angular-only impulse (no torque arm
  coupling the two), deliberately mirroring how contact resolution keeps
  its own restitution impulse linear-only for solver stability (see
  above) -- a single coupled 6-DOF spring wasn't attempted given that
  precedent.
- **`DistanceJoint`'s positional correction is still center-only**: the
  velocity-phase constraint impulse is applied at the actual world-space
  anchor points (torquing both bodies, closing the old "zero torque arm"
  gap), but the separate Baumgarte-style positional-drift cleanup pass
  still just translates each body's whole center along the
  anchor-to-anchor direction rather than accounting for the anchor's own
  offset -- a v1 simplification, consistent with contact resolution's own
  linear-only position pass (see above).
- **Joint/contact solver iteration count is fixed per `World`, not
  adaptive**: `World.solver_iterations` (default 4) runs that many
  velocity-resolution passes over every joint and contact each step, and
  (for joints only) that many *positional* Gauss-Seidel passes too --
  this is what actually fixes most of the old joint-chain-sag problem,
  since sag was predominantly positional, not a velocity-constraint
  issue. It substantially reduces, but doesn't eliminate, sag: an 8-link
  chain that stretched to ~215% of its total `rest_length` at 1 iteration
  settles to ~108% at the default of 4, and ~102% at 8 -- there's no
  free per-step convergence check, so a short/lightly-loaded chain pays
  the same iteration count as a long/heavy one. Contacts are only
  velocity-iterated, not position-iterated (their fixed penetration
  reading from detection time would make a repeated position pass
  over-correct -- see the `b3_world_step` comment in `world.c`), so
  stacking behavior doesn't benefit from `solver_iterations` the way
  joint chains do.
- **Contact events are polled, not a registered callback**:
  `World.contacts_began`/`.contacts_ended` are read after each
  `World.step()` call, rather than the library invoking a
  Python-supplied listener function mid-step. Deliberate: `libbox3d`
  (the C core `World.step()` calls into) has zero Python dependency by
  design, so it can only ever expose queryable event *lists*, not call
  back into Python itself -- a callback-based API would have to live
  entirely in the Python binding layer anyway, and polling avoids
  reentrancy concerns (a listener callback firing partway through a
  step, while the solver is still mutating bodies) for no real loss of
  expressiveness.
- **`contacts_ended` also fires when a pair falls asleep together, not
  just on genuine separation**: once both bodies in a touching pair are
  static/asleep, that pair is excluded from collision entirely (the
  performance `World.sleeping_enabled` exists for) -- so it "ends" from
  the event tracking's perspective even though the bodies are still
  physically touching, and "begins" again the moment either one wakes.
- **Broad phase is sort-and-sweep, not a dynamic BVH tree**: bodies'
  AABBs are sorted and swept along the X axis fresh every step (O(n log
  n)) to find candidate pairs, rather than an incrementally-maintained
  bounding-volume tree with insert/remove/refit and a rotation
  heuristic. This still turns the original naive O(n²) body-pair scan
  into a real broad phase for the common case of a handful of nearby
  bodies among many -- just a simpler algorithm than a from-scratch
  dynamic tree, which is a substantially larger amount of machinery
  (Box2D's incremental dynamic AABB tree is on the order of 500 lines by
  itself) for a library whose collision detection was already documented
  as targeting "the small body counts this basic library targets."
- **Sleeping is per-body, not full connected-component islands**: each
  dynamic body tracks its own sleep timer independently (see
  `RigidBody.is_sleeping`/`World.sleep_time_threshold`), rather than
  b3_world_step computing connected components over the contact/joint
  graph and sleeping/waking a whole group of touching bodies together.
  In practice this means one body in a resting stack or a jointed chain
  can fall asleep slightly before or after its neighbors, rather than
  the whole group settling in lockstep -- a minor visual inconsistency
  in some scenes, not a correctness issue (a joint/contact between an
  awake and a sleeping body is still fully resolved each step, and
  naturally wakes the sleeping side if that resolution gives it an
  above-threshold velocity -- see the `b3_world_step` comment in
  `world.c`).
- **`CharacterMover` is discrete, not continuous/swept collision**: like
  everything else in this library (see the round-shape/GJK-EPA notes
  above), `.move()` resolves overlaps via discrete push-out + slide
  (test after the fact, then correct), not a true continuous/swept
  sweep-test-before-you-move. A character moving fast enough relative to
  a thin obstacle in a single call can still tunnel straight through it
  without ever registering an overlap, the same caveat `b3_World`'s own
  contacts already have. Keep `displacement` per call small relative to
  the thinnest obstacle in the scene if this matters.
- **`CharacterMover.move()`'s push-out only ever resolves the single
  worst-penetrating overlap per iteration**: with `max_slide_iterations`
  (default 4) passes, this converges fine for the common case of one or
  two obstacles, but a character wedged into a tight corner against
  several overlapping shapes at once may need more iterations than the
  default to fully resolve in a single `move()` call (it'll finish
  resolving over the next few calls instead, which is usually
  imperceptible at typical frame rates, but is worth knowing about if you
  see a one-frame partial-penetration flicker in a cluttered scene).
- **Debug Draw is wireframe line segments only, no rendering backend**:
  `RigidBody.debug_lines()`/`World.debug_contacts()`/
  `.debug_joint_anchors()` return plain `Vec3` pairs -- turning them into
  pixels (or anything else) is entirely up to the caller, by design (this
  project has no GUI/rendering dependency at all, in v1 or otherwise).
  `ConvexHull.debug_lines()` draws its AABB, not an exact wireframe (v1
  has no computed edge topology for an arbitrary vertex set -- see
  `ConvexHull`'s own limitation above); `Sphere`/`Capsule` circles are
  `B3_DEBUG_CIRCLE_SEGMENTS`-sided (16) polygon approximations, not exact
  curves. `World.debug_contacts()` re-scans every body pair fresh
  (independent of the sort-and-sweep broad phase `World.step()` itself
  uses, and *not* filtered by Filter joints -- see its own docstring),
  so it's meant for occasional visualization calls, not every step of a
  performance-critical simulation.
