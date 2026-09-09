# Known v1 limitations

These are deliberate scope cuts for a "basic" first version, not bugs:

- **Global contact materials**: [`World.step`](api/world.md) uses
  `World.default_restitution` / `World.default_friction` for every
  contact rather than per-body mixing rules. Each
  [`RigidBody`](api/rigidbody.md) still carries its own
  `restitution`/`friction` fields for forward compatibility, but v1
  doesn't consult them.
- **No edge-edge contact normals**: [`Box3D.overlaps()`](api/box3d.md)
  runs a full 15-axis SAT test to decide *whether* two boxes overlap, but
  only derives the reported contact normal/penetration from face axes.
  Edge-edge contact configurations report a face-based approximation
  rather than an exact edge-edge normal.
- **Linear-only contact impulses**: collision response applies impulses
  without an angular (torque) contribution, which is enough for
  axis-aligned stacking/resting scenarios but is a simplification for
  tumbling contacts.
- **`World.remove_body` swap-removes**: removing a body moves the last
  body into the freed slot. Any `RigidBody` handle still holding the old
  index for that displaced body will silently resolve to a different body
  (or raise `ValueError` if the slot is now out of range).
  [`World.get_body`](api/world.md) always returns a fresh handle object,
  so `is` comparisons don't identify a body across two calls.
- **[`DistanceJoint`](api/joint.md) only, anchored at body centers**: the
  only joint type in v1 is a rigid center-to-center distance constraint --
  no per-body local anchor offset, no spring softness/limits, and (like
  contact resolution) no angular/torque contribution. `World.remove_body`
  also doesn't clean up joints that reference the removed (or swapped)
  index, so removing a body that has joints attached can leave a joint
  pointing at the wrong body or an out-of-range index. See `TODO.md` for
  the rest of the planned joint types (revolute, prismatic, spherical,
  weld, motor, wheel, ...).
