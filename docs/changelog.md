# Changelog

Mirrors
[`CHANGELOG.md`](https://github.com/algaves/pybox3d/blob/main/CHANGELOG.md)
at the repo root -- keep both in sync when either changes.

All notable changes to `pybox3d` are documented here. Loosely follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); no version has
been tagged/released yet, so everything so far lives under Unreleased.
See [Roadmap](roadmap.md) for what's still missing.

## [Unreleased]

### Added

- Utilities: [CharacterMover](reference/classes/charactermover.md) (a
  kinematic move-and-slide character controller -- construct with a
  position and any of the seven shape types, then call `.move(world,
  displacement)` each step to move it and resolve collisions against
  `world`'s bodies via discrete push-out + slide; not a `RigidBody`,
  never added to a `World`, so gravity/input are entirely up to the
  caller) and Debug Draw (`RigidBody.debug_lines()` -- a wireframe
  approximation of a body's current shape; `World.debug_contacts()`/
  `.debug_joint_anchors()` -- pure data, no rendering backend of any
  kind, by design). This closes the last two items on
  [Roadmap](roadmap.md) -- every roadmap item is now checked off.
- World/Simulation systems: [`World.query_aabb()`/`.raycast_all()`](reference/functions/world.md)
  (AABB and ray queries over every body); `World.contacts_began`/
  `.contacts_ended` (this step's newly-formed/newly-separated contact
  pairs, polled after `World.step()` rather than a registered callback
  -- see [Known limitations](limitations.md) for why); per-body sleeping
  (`World.sleeping_enabled` and its `sleep_linear_threshold`/
  `sleep_angular_threshold`/`sleep_time_threshold` knobs,
  `RigidBody.is_sleeping`/`.wake()` -- a sleeping body's integration is
  skipped until a joint/contact impulse or an explicit `wake()`/attribute
  write gives it an above-threshold velocity again); and
  `World.snapshot()`/`.restore()` (a new
  [WorldSnapshot](reference/classes/worldsnapshot.md) type) for simple
  state recording/replay. The narrow phase's broad-phase pass is now a
  sort-and-sweep (bodies' AABBs sorted and swept along X) instead of the
  original naive O(n²) body-pair scan -- not the dynamic BVH tree
  originally planned; see [Known limitations](limitations.md) for why a
  simpler algorithm was chosen. Also skips any pair where both bodies
  are static/asleep.
- Eight new joint kinds beyond `DistanceJoint`, all under a single new
  [Joint](reference/classes/joint.md) class -- `World.add_spherical_joint()`
  (ball-and-socket point constraint, free rotation), `add_revolute_joint()`
  (a hinge: point constraint plus a lock on every rotation axis except
  the given one, with an optional torque motor), `add_prismatic_joint()`
  (a slider: locks everything but translation along an axis, with
  optional limits and a force motor), `add_weld_joint()` (full 6-DOF
  rigid lock), `add_motor_joint()` (soft, spring-driven -- decoupled
  linear/angular springs toward a target pose, not a hard constraint),
  `add_wheel_joint()` (a Prismatic-style suspension axis combined with
  free rotation around a separate axle axis), `add_filter_joint()` (not
  a real constraint -- disables collision between the named pair), and
  `add_parallel_joint()` (angular-only Weld: orientation locked,
  translation free). `Joint.kind` says which one a handle is; only the
  attributes that kind actually uses are settable, others raise
  `AttributeError`. See [Known limitations](limitations.md) for each new
  kind's v1 scope cuts.
- [DistanceJoint](reference/classes/distancejoint.md) per-body local
  anchors (`anchor_a`/`anchor_b`), distance limits (`min_length`/
  `max_length`/`has_limits`), and spring softness/damping (`stiffness`/
  `damping`), settable via `World.add_joint()`'s new keyword arguments
  or directly on the returned handle. The joint's velocity-phase
  constraint impulse is now applied at the actual world-space anchor
  points, so it torques each body when an anchor isn't at the body's
  center -- closing the "linear-only, center-only" v1 gap for joints.
  Positional-drift cleanup still only translates each body's center, a
  documented v1 simplification -- see [Known limitations](limitations.md).
- `World.solver_iterations` (default 4): distance joints and contacts
  now resolve their velocity constraint that many times per step over
  the same detected set, and joints additionally run that many
  positional passes. This substantially reduces (but doesn't eliminate)
  the documented joint-chain-sag limitation: an 8-link chain that
  stretched to ~215% of its total `rest_length` at a single iteration
  settles to ~108% at the new default of 4.
- Per-body contact materials: `World.step` now combines each contact
  pair's own [`RigidBody`](reference/classes/rigidbody.md)
  `restitution`/`friction` (max and geometric mean respectively) instead
  of a world-global default. `World.default_restitution`/
  `.default_friction` are removed (superseded).
- Full 6-DOF friction impulses: a sliding body now correctly picks up
  rolling spin from contact friction (previously linear-only). The
  separating/restitution impulse stays linear-only -- see
  [Known limitations](limitations.md) for why. Restitution is also now
  suppressed below a small closing-speed threshold (Box2D's technique)
  to stop resting-contact noise reading as a repeated tiny bounce.
- [Sphere](reference/classes/sphere.md),
  [Capsule](reference/classes/capsule.md),
  [ConvexHull](reference/classes/convexhull.md), and
  [Compound](reference/classes/compound.md) shapes, plus
  `RigidBody.sphere()`/`.capsule()`/`.hull()`/`.compound()`
  constructors. `RigidBody.shape` now returns whichever of the five
  matches the body's kind. See [Known limitations](limitations.md) for
  `ConvexHull`/`Compound`'s v1 scope cuts (no hull construction from a
  raw point cloud, approximate hull raycast/inertia, approximate
  compound inertia, no nested compounds).
- [TriangleMesh](reference/classes/trianglemesh.md) and
  [HeightField](reference/classes/heightfield.md) shapes, plus
  `RigidBody.mesh()`/`.heightfield()` constructors (both always-static --
  no `mass` parameter, and setting `.mass` on one raises `ValueError`).
  Collision against a convex shape is per-triangle GJK/EPA with an AABB
  quick-reject; see [Known limitations](limitations.md) for the v1
  scope cuts (`contains_point()` always `False`, AABB-approximated
  `raycast()`, fixed size caps, no mesh/height-field pair support).
- Exact edge-edge contact normals for box-box: the 15-axis SAT now
  compares all 15 candidate axes (previously only the 6 face axes) to
  find the true least-penetrating one, and when an edge-edge axis wins,
  computes the contact point as the exact closest points between the
  two specific box edges instead of falling back to a face-based
  approximation.
- A generic GJK/EPA collision core backing every shape-pair
  `overlaps()`/`World.step()` collision other than box-vs-box, which
  keeps its existing exact SAT test. `Box3D.overlaps()` (and the new
  shape types' `overlaps()`) now accept any of the seven shape types.
- [ContactInfo](reference/classes/contactinfo.md) gained a third field,
  `point`: one representative world-space contact point (v1
  simplification, not a full manifold -- see
  [Known limitations](limitations.md)).
- Stable, generation-checked ids (`b3_BodyId`/`b3_JointId`, via a new
  `b3_SlotMap` shared by `b3_World`'s body and joint storage) backing
  every `RigidBody`/`DistanceJoint` handle returned by `World.add_body`/
  `get_body`/`add_joint`/`get_joint`. A handle now keeps resolving to the
  *same* entity even if unrelated `remove_body`/`remove_joint` calls
  relocate it via swap-remove, and reliably raises `ValueError` once the
  entity it names has actually been removed. `World.remove_body` also
  now removes (and invalidates the handles of) every joint that
  referenced the removed body, instead of leaving it dangling.
- `DistanceJoint`: a rigid center-to-center distance constraint between
  two bodies already in a `World`, via `World.add_joint()`/`get_joint()`/
  `remove_joint()`/`joint_count`. See [DistanceJoint](reference/classes/distancejoint.md) and
  the pendulum demo in [Examples](examples/index.md).
- [Roadmap](roadmap.md), tracking missing features against
  <https://box2d.org/documentation3d/topics.html>.
- This changelog.
- MkDocs documentation site (`docs/`, `mkdocs.yml`, `uv run mkdocs
  serve`).
- PyPI packaging metadata (`classifiers`, `keywords`, `project.urls`,
  `license`/`license-files`) and a GitHub Actions release workflow
  (`../.github/workflows/publish.yml`).
- `LICENSE.md` (LGPL-2.1-or-later).
- README: Installation (`pip install`), Usage, Documentation, and
  Publishing sections.
- `examples/joint_chain_demo.py` (a short chain of `DistanceJoint`s
  hanging from a static anchor) and `examples/dumbbell_demo.py` (two
  free dynamic bodies joined by a `DistanceJoint`, no static anchor) --
  see [Examples](examples/index.md).
- This Roadmap and Changelog, published as pages in the docs site
  (mirroring `TODO.md`/`CHANGELOG.md` at the repo root).

### Changed

- Version bumped to `2026a2` for the second PyPI release (tagged
  `v2026a2`).
- Version bumped to `2026a1` for the first PyPI release (tagged
  `v2026a1`).
- Release workflow (`../.github/workflows/publish.yml`) now builds a
  full wheel matrix with `cibuildwheel` -- Windows (x86_64), Linux
  (x86_64 + ARM64 via QEMU, `manylinux`), and macOS (x86_64 + ARM64) for
  Python 3.10-3.14 -- runs the test suite against every wheel, and
  publishes to PyPI with trusted publishing. The sdist is built
  separately and trimmed to sources only (no `docs/`, `tests/`,
  `examples/`, `.github/`, or `uv.lock`).
- CI split into `tests.yaml` (cross-platform pytest matrix on every
  push/PR) and `check.yaml` (ruff lint + mypy typecheck) workflows.
- Installation docs now advertise the prebuilt wheels ([Installation](getting-started/install.md));
  source installs remain documented as the fallback for unsupported
  platforms.
- MkDocs theme switched to
  [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/)
  with a light blue palette (after a brief detour to the Nature theme),
  restoring Material's visual style and enhanced search. This reverts
  the earlier Material-to-Nature switch.
- Documentation site restructured into guided sections: `Getting
  started` (install / quickstart / documentation workflow), `Tutorials`
  (step-by-step walkthroughs of the falling box, pendulum, joint chain,
  and dumbbell demos), `Examples` (one page per runnable demo),
  `Reference` (module, class, and function sub-pages), and `Project`
  (limitations / roadmap / changelog). `docs/api/` was replaced by
  `docs/reference/{modules,classes,functions}/`, with method content
  split across the new class/function pages and `ContactInfo`/`RayHit`
  promoted to their own class pages.

### Fixed

- Broad-phase sort-and-sweep pair ordering is now deterministic: equal
  AABB `min_x` values break ties by body index. An unstable `qsort` tie
  previously made a coincident-body prismatic-motor test fail only on
  Windows (MSVC reorders equal elements differently than glibc/clang);
  that test also no longer starts with the cart fully inside the rail.
- `docs/getting-started.md` and `README.md` still referenced the
  Nature theme/`mkdocs-nature` package after the switch back above;
  updated both to `mkdocs-material`.
- The PyPI publish workflow ignores platform-tag problems: the
  `pypi-publish` job runs on `workflow_dispatch` as well as on release
  (both via trusted publishing on the `pypi` environment), and the README
  documents that locally-built Linux wheels carry an unsupported
  `linux_x86_64` tag which PyPI rejects -- repair them with
  `auditwheel repair` or publish through the CI workflow.
- [`index.md`](index.md) never mentioned `DistanceJoint` after it was
  added.
- Task-list checkboxes (`- [x]`/`- [ ]`) on this page and
  [Roadmap](roadmap.md) rendered as literal `[x]`/`[ ]` text: the
  Material-to-Nature theme switch dropped `pymdown-extensions` (a
  transitive Material dependency), which `pymdownx.tasklist` needs.
  Added it back as a direct `docs` dependency and enabled
  `pymdownx.tasklist` in `mkdocs.yml`.
- The built wheel no longer bundles the C extension's `.c`/`.h` sources
  (`wheel.exclude` in `pyproject.toml`) -- dead weight not needed at
  runtime, left over from `wheel.packages` copying the whole
  `src/pybox3d` tree.
- [RigidBody functions](reference/functions/rigidbody.md) described
  `apply_impulse()` as accumulated and only taking effect on the next
  `World.step()`, like `apply_force()` -- it actually changes
  `linear_velocity`/`angular_velocity` immediately, no accumulator
  involved.
- `_pybox3d.pyi` still declared the removed `World.default_restitution`/
  `.default_friction` attributes after per-body contact materials
  dropped them from the actual extension (see Added, above) -- a real
  stub/runtime mismatch that `mypy` couldn't catch because the stub, not
  the compiled module, is what it type-checks against. Removed them from
  the stub and added `solver_iterations`/the new `DistanceJoint`/
  `add_joint` fields from this cycle.
- `World.remove_body`/`remove_joint`'s swap-remove no longer silently
  misdirects a stale `RigidBody`/`DistanceJoint` handle to the wrong
  entity, and no longer leaves a joint dangling after the body it
  referenced is removed (see Added, above). Removed the now-obsolete
  "swap-remove" and "no joint cleanup" bullets from
  [Known limitations](limitations.md)/`README.md`.
- The GJK/EPA collision core (introduced this cycle, see Added) no
  longer produces a degenerate zero-length normal for pairs whose
  centers happen to lie on a common axis (e.g. two spheres stacked
  along X) -- a routine, not edge-case, configuration. Fixed by seeding
  the search off-axis, retrying with alternate seeds and keeping the
  largest-penetration non-degenerate result when one still converges
  poorly, and (for `ConvexHull`) averaging tied support vertices instead
  of arbitrarily picking one.

### Documented

- Newly-discovered limitation: joints solve once per step with no inner
  iteration loop, so a joint chain longer than ~2-3 links sags well past
  its `rest_length` (found while writing `joint_chain_demo.py`; tracked
  in [Roadmap](roadmap.md)/[Known limitations](limitations.md), not
  fixed yet).

## Initial implementation

- `Vec3` / `Quat`: 3D vector and rotation-quaternion math.
- `Box3D`: an axis-aligned or oriented box (AABB/OBB) with
  point-containment, SAT-based overlap, and ray-cast queries.
- `RigidBody` / `World`: box-scoped rigid-body dynamics -- mass/inertia,
  semi-implicit Euler integration, naive O(n²) collision detection, and
  impulse-based contact resolution.
