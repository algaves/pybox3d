# TODO

Also published as [Roadmap](docs/roadmap.md) in the MkDocs site -- keep
both in sync when either changes.

Missing elements for the `pybox3d` / `libbox3d` wrapper, tracked against
the topics a mature 3D physics library's docs cover:
<https://box2d.org/documentation3d/topics.html>

Checked items are implemented (possibly in a reduced "v1" form -- see
[Known limitations](docs/limitations.md)); unchecked items don't exist
yet. See [CHANGELOG.md](CHANGELOG.md) for what's landed so far.

## Shapes

- [x] Box (AABB/OBB)
- [ ] Sphere
- [ ] Capsule
- [ ] Convex Hull
- [ ] Triangle Mesh
- [ ] Height Field
- [ ] Compound shape (multiple shapes per body)

## Joints

- [x] Joint infrastructure (`World.add_joint`/`get_joint`/`remove_joint`,
      world-backed handles, growable storage)
- [x] Distance Joint (rigid, center-to-center only)
- [ ] Per-body local anchor points on joints (currently center-to-center
      only -- no attaching to a corner/offset)
- [ ] Joint limits (min/max distance or angle) and spring softness/damping
- [ ] Spherical (ball-and-socket) Joint
- [ ] Revolute (hinge) Joint
- [ ] Prismatic (slider) Joint
- [ ] Weld Joint
- [ ] Motor Joint
- [ ] Wheel Joint
- [ ] Filter Joint (disable collision between specific body pairs)
- [ ] Parallel Joint
- [ ] Joint cleanup on `World.remove_body` (removing a body currently
      leaves any joint referencing its index dangling/misdirected)
- [ ] Multiple solver iterations per step for joints (currently one
      sequential pass, so longer joint chains sag well past
      `rest_length` -- see `examples/joint_chain_demo.py` and
      [Known limitations](docs/limitations.md))

## World / Simulation

- [x] World (gravity, fixed-`dt` step)
- [ ] Contact/touch events (begin/end callbacks)
- [ ] Query API (overlap/shape queries beyond a single `Box3D.raycast`)
- [ ] Dynamic BVH tree for broad-phase (currently naive O(n²))
- [ ] Sleeping bodies / islands
- [ ] World state recording & replay (for debugging)

## Body

- [x] RigidBody (position, orientation, velocities, mass/inertia,
      forces/impulses)
- [ ] Per-body restitution/friction actually used in contact resolution
      (currently global-only via `World.default_restitution`/
      `default_friction`)
- [ ] Angular (torque) contribution to contact and joint impulses
      (currently linear-only)
- [ ] Stable body handles across `World.remove_body` (currently
      swap-remove, which can silently misdirect a stale handle)

## Collision

- [x] Box-box overlap (15-axis SAT) and single-box raycast
- [ ] Sphere/capsule/mesh/height-field collision pairs (blocked on the
      shapes above)
- [ ] Exact edge-edge contact normals (currently a face-based
      approximation)

## Utilities

- [ ] Character Mover
- [ ] Debug Draw
- [ ] Ids (opaque handles, e.g. to survive across `remove_body`/
      `remove_joint` swap-removes -- see Body/Joint items above)
