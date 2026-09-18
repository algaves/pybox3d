# pybox3d

![GitHub Last Commit](https://img.shields.io/github/last-commit/algaves/pybox3d.svg?style=flat-square)
![Python - Version](https://img.shields.io/badge/python-%3E%3D3.10-brightgreen?style=flat-square)
![PyPI - Version](https://img.shields.io/pypi/v/pybox3d?style=flat-square)
![Python - Implementation](https://img.shields.io/pypi/implementation/pybox3d?style=flat-square)
![PyPI - Wheel](https://img.shields.io/pypi/wheel/pybox3d?style=flat-square)
![Docs](https://img.shields.io/badge/docs-mkdocs-blue?style=flat-square)
![License](https://img.shields.io/badge/license-LGPL--2.1-green?style=flat-square)


Python bindings, written directly against the **CPython C API** (no
ctypes/cffi/pybind11/nanobind), for `libbox3d` -- a small, from-scratch
C library providing:

- **`Box3D`** / **`Sphere`** / **`Capsule`** / **`ConvexHull`** /
  **`Compound`** / **`TriangleMesh`** / **`HeightField`**: 3D shapes with
  point-containment, overlap (exact SAT for box-vs-box, a generic
  GJK/EPA core for anything else), and ray-cast queries. The last two
  are always-static level geometry.
- **`RigidBody`** / **`World`**: rigid-body dynamics -- mass/inertia,
  semi-implicit Euler integration, sort-and-sweep broad phase +
  impulse-based resolution, per-body sleeping, contact begin/end events,
  AABB/raycast queries, and simple state snapshot/restore
  (`WorldSnapshot`).
- **`DistanceJoint`**: a distance constraint for connecting two bodies
  (e.g. a pendulum, a chain link), anchored at a per-body local offset
  (default each body's center), either rigid at `rest_length` or a soft
  spring (`stiffness`/`damping`) optionally bounded by `min_length`/
  `max_length`.
- **`Joint`**: every other joint kind -- Spherical (ball-and-socket),
  Revolute (hinge, with an optional motor), Prismatic (slider, with
  optional limits/motor), Weld (full 6-DOF lock), Motor (a soft
  spring-driven pose target), Wheel (suspension + free spin), Filter
  (disables collision between a pair), and Parallel (angular-only Weld).
- **`CharacterMover`**: a kinematic move-and-slide character controller,
  plus Debug Draw (`RigidBody.debug_lines()`, `World.debug_contacts()`/
  `.debug_joint_anchors()`) -- pure data, no rendering backend.

## Requirements

- Python 3.10+
- For source installs: a C compiler and CMake (needed at install time --
  pip/uv compile the extension for you via the `scikit-build-core` build
  backend)

Prebuilt wheels are published for Python 3.10-3.14 on Windows (x86_64),
Linux (x86_64 and ARM64), and macOS (x86_64 and ARM64) -- those need no
C toolchain.

## Installation

```sh
pip install pybox3d                                        # prebuilt wheel
pip install .                                              # from a local checkout
pip install git+https://github.com/algaves/pybox3d.git     # straight from GitHub
```

Source installs build `libbox3d` and the `pybox3d._pybox3d` C extension
as part of the install -- there's nothing else to run afterwards.

## Usage

```python
from pybox3d import RigidBody, Vec3, World

world = World(gravity=(0, -9.81, 0))

ground = RigidBody(Vec3(0, -0.5, 0), Vec3(5, 0.5, 5), mass=0.0)  # mass=0 -> static
world.add_body(ground)

box = RigidBody(Vec3(0, 5, 0), Vec3(0.5, 0.5, 0.5), mass=1.0)
handle = world.add_body(box)  # add_body() returns a world-backed handle

for _ in range(300):  # 5 seconds at 60Hz
    world.step(1 / 60)

print(handle.position.y)  # settles near 0.5, resting on the ground
```

The library has fifteen public types:

- **`Vec3`** / **`Quat`** -- a 3D vector and rotation quaternion, with the
  usual arithmetic (`+`, `-`, `*`, dot/cross product, normalization,
  axis-angle construction, rotating a `Vec3` by a `Quat`, ...). Anywhere a
  `Vec3`/`Quat` is expected, a plain 3- or 4-element tuple/list works too.
- **`Box3D`** / **`Sphere`** / **`Capsule`** / **`ConvexHull`** /
  **`Compound`** / **`TriangleMesh`** / **`HeightField`** -- standalone
  shapes, each with `contains_point()`, `overlaps()` (accepts any of the
  seven shape types, returns a `ContactInfo` normal/penetration/point or
  `None`), and `raycast()` (returns a `RayHit` or `None`). `ConvexHull`
  takes a pre-computed convex vertex set; `Compound` rigidly attaches
  multiple leaf shapes (not nested `Compound`s) at local offsets;
  `TriangleMesh`/`HeightField` are always-static level geometry.
- **`RigidBody`** -- a rigid body with position, orientation, velocities,
  mass, and `apply_force()`/`apply_impulse()`. The default constructor
  makes a box-shaped body; `RigidBody.sphere()`/`.capsule()`/`.hull()`/
  `.compound()`/`.mesh()`/`.heightfield()` are the other shape kinds'
  equivalents (the last two always static). A handle returned by
  `World.add_body()`/`World.get_body()` is "world-backed": reading/
  writing it reads/writes the body's live state inside the `World`.
- **`World`** -- owns a set of bodies and steps the simulation
  (`step(dt)`), with a sort-and-sweep broad phase and impulse-based
  resolution, resolved over `solver_iterations` passes per step (default
  4). Also has `query_aabb()`/`raycast_all()` queries, `contacts_began`/
  `contacts_ended` event lists (read after each `step()` call), per-body
  sleeping (`sleeping_enabled` and its thresholds, default on), and
  `snapshot()`/`restore()` for simple state recording/replay.
- **`WorldSnapshot`** -- an opaque, point-in-time recording of every
  body's transform/velocity in a `World`, returned by `World.snapshot()`
  and consumed by `World.restore()`.
- **`DistanceJoint`** -- connects two bodies already in a `World` via
  `World.add_joint(body_a, body_b, rest_length=None, anchor_a=(0,0,0),
  anchor_b=(0,0,0), min_length=None, max_length=None, stiffness=0.0,
  damping=0.0)`, holding the distance between the two world-space anchors
  at `rest_length` (defaults to their current distance apart) -- rigidly
  by default, or as a spring when `stiffness > 0`, optionally hard-capped
  at `min_length`/`max_length`.
- **`Joint`** -- every other joint kind, each via its own `World`
  method: `add_spherical_joint()`, `add_revolute_joint()`,
  `add_prismatic_joint()`, `add_weld_joint()`, `add_motor_joint()`,
  `add_wheel_joint()`, `add_filter_joint()`, `add_parallel_joint()`.
  `Joint.kind` says which one; only the attributes that kind uses are
  settable.
- **`CharacterMover`** -- a kinematic move-and-slide character
  controller: `CharacterMover(position, shape)`, then
  `.move(world, displacement)` each step to move it and resolve
  collisions against `world`'s bodies (discrete push-out + slide, not
  continuous/swept -- see [Known v1 limitations](#known-v1-limitations)).
  Not a `RigidBody`; never added to a `World`.

Debug Draw is pure data, not a type: `RigidBody.debug_lines()` (a
wireframe approximation of a body's current shape) and
`World.debug_contacts()`/`.debug_joint_anchors()` all return lists of
`Vec3` pairs -- turning them into pixels is up to you, with whatever
rendering setup you already have.

See [`docs/`](docs/) for the full API reference (build it locally with
`uv run mkdocs serve`, see below) and `examples/falling_box_demo.py` for a
complete runnable version of the snippet above. `examples/` also has three
more `DistanceJoint` demos -- `pendulum_joint_demo.py`,
`joint_chain_demo.py`, and `dumbbell_demo.py` -- see the docs site's
Examples page for all of them.

## Development

```sh
uv sync              # builds the C extension and installs dev dependencies
uv run pytest -v     # run the test suite
uv run ruff check .  # lint
uv run ruff format . # format
uv run mypy          # type-check
uv run python examples/falling_box_demo.py
```

## Documentation

Full docs (getting started, API reference, examples) are built with
[MkDocs](https://www.mkdocs.org/):

```sh
uv sync --group docs   # install mkdocs + mkdocs-material
uv run mkdocs serve    # live-reloading preview at http://127.0.0.1:8000
```

## Publishing a release

`.github/workflows/publish.yml` builds the sdist and all wheels
with [`cibuildwheel`](https://cibuildwheel.pypa.io/): Linux
(x86_64 + ARM64), Windows (x86_64), and macOS (x86_64 + ARM64) for
Python 3.10-3.14, running the test suite against every wheel, then
publishes them to PyPI via [trusted
publishing](https://docs.pypi.org/trusted-publishers/) -- no API token
needed. The `pypi-publish` job runs when the workflow is triggered from
*Actions / Run workflow* (`workflow_dispatch`) or whenever a GitHub
release is published, and deploys to the `pypi` environment.

One-time setup:

1. On PyPI, add a *Trusted Publishers* entry for this repository
   matching the `publish.yml` workflow and the `pypi` environment.
2. In the repo settings, create a `pypi` GitHub environment.

To publish a new version:

1. Bump the version in `pyproject.toml` and add a `CHANGELOG.md` entry.
2. Run the `Upload Python Package` workflow (*Actions -> Run workflow*),
   or tag the commit and create a GitHub release.

### Quick local build (single platform)

```sh
uv build     # produces dist/pybox3d-<version>.tar.gz and a wheel
uv publish   # set UV_PUBLISH_TOKEN with a PyPI API token (or twine upload dist/*)
```

Linux caveat: a local `uv build` produces a `linux_x86_64`-tagged wheel,
which PyPI rejects. Repair it first --
`uvx --from auditwheel auditwheel repair dist/*.whl -w dist/` -- which
re-tags it with the compatible `manylinux*` tags. To avoid this
altogether, publish through the CI workflow above.

## Layout

- `libbox3d/` -- the pure-C library (zero Python dependency).
- `src/pybox3d/_ext/` -- the CPython C-API glue, compiled into
  `pybox3d._pybox3d`.
- `src/pybox3d/` -- the Python package (`__init__.py` re-exports the
  compiled extension's public API; `_pybox3d.pyi` is its type stub, since
  mypy cannot see into the compiled `.so`).
- `tests/` -- pytest suite.
- `examples/` -- runnable demos.
- `docs/` -- MkDocs source for the full documentation site.
- [`TODO.md`](TODO.md) -- missing features, tracked against a mature 3D
  physics library's docs.
- [`CHANGELOG.md`](CHANGELOG.md) -- notable changes.

## Known v1 limitations

These are deliberate scope cuts for a "basic" first version, not bugs:

- **Single-point contacts, no persistent manifold**: every `overlaps()`
  result (`ContactInfo`) carries exactly one representative contact
  point, not a full multi-point manifold, and there's no warm-starting
  between simulation steps.
- **Approximate normal/penetration for round shapes**: any pair
  involving a `Sphere`/`Capsule` goes through a generic GJK/EPA core
  (box-vs-box keeps its exact SAT). On perfectly round shapes (e.g.
  sphere-vs-sphere) the polytope EPA refines only approximates the true
  curved surface, so the reported normal/penetration can be off by a
  few percent.
- **`ConvexHull` accepts only a pre-computed convex vertex set**: no
  quickhull/incremental construction from a raw point cloud in v1, at
  most `HULL_MAX_VERTICES` (32) vertices (shapes have no heap
  allocation). Its `raycast()` is approximated via the hull's AABB
  rather than an exact surface test.
- **`ConvexHull`/`Compound` inertia is approximate**: a hull is treated
  as a solid box of its own local AABB; a compound sums each child's
  own inertia (mass split equally) via the parallel-axis theorem,
  ignoring each child's local orientation.
- **`Compound` doesn't support nesting**: children can be
  `Box3D`/`Sphere`/`Capsule`/`ConvexHull`, not another `Compound`.
- **`TriangleMesh`/`HeightField` are always static**: no `mass` parameter
  on `RigidBody.mesh()`/`.heightfield()`, and setting `.mass` afterwards
  raises `ValueError`. `contains_point()` always returns `False`;
  `raycast()` is approximated via the shape's AABB. Fixed size caps
  (`MESH_MAX_TRIANGLES`=64, `HEIGHTFIELD_MAX_ROWS`/`_COLS`=16). Collision
  only supports mesh/height-field vs. a convex shape -- mesh/height-field
  vs. mesh/height-field pairs always report no overlap.
- **Contact restitution is linear-only; friction is full 6-DOF**: the
  separating/restitution impulse doesn't torque either body, but
  friction does (e.g. a sliding ball correctly picks up rolling spin).
  Deliberate: a fully coupled normal-constraint solve was tried but
  isn't stable with this project's single-pass, non-warm-started solver
  for a body resting on an offset contact (e.g. an upright capsule on
  its round cap) -- see [docs/limitations.md](docs/limitations.md) for
  why. Restitution is also suppressed below a small closing-speed
  threshold (matching Box2D's technique) to stop resting-contact noise
  from reading as a repeated tiny bounce.
- **Angular-locking joints are velocity-only**: Revolute/Prismatic/Weld/
  Wheel/Parallel's rotation-locking DOF only ever cancel relative angular
  *velocity* -- there's no positional correction pulling misaligned
  bodies back into alignment (translation constraints, e.g. Distance or
  the point part of a Spherical/Revolute/Weld, do get one). Revolute also
  has no angle limits (would need swing-twist decomposition), and Motor's
  linear/angular springs are fully decoupled (no torque-arm coupling
  between them). See [docs/limitations.md](docs/limitations.md).
- **Joint/contact solver iteration count is fixed, not adaptive**:
  `World.solver_iterations` (default 4) runs that many velocity passes
  per step over every joint/contact, plus (joints only) that many
  positional Gauss-Seidel passes -- this substantially reduces the old
  joint-chain-sag problem (an 8-link chain went from ~215% of its total
  `rest_length` at 1 iteration to ~108% at the default 4), but doesn't
  eliminate it, and a short/lightly-loaded chain pays the same iteration
  cost as a long/heavy one. See
  [docs/limitations.md](docs/limitations.md) for why contacts only
  benefit from the velocity passes, not the positional ones.
- **Contact events are polled, not a registered callback**, and
  `contacts_ended` also fires when a pair falls asleep together (not
  just on genuine separation), since sleeping pairs skip collision
  entirely. Broad phase is sort-and-sweep, not a dynamic BVH tree.
  Sleeping is per-body, not full connected-component islands, so one
  body in a resting stack/jointed chain can sleep slightly before or
  after its neighbors. See [docs/limitations.md](docs/limitations.md)
  for the reasoning behind each.
- **`CharacterMover` is discrete, not continuous/swept collision**, like
  every other collision test in this library: `.move()` resolves
  overlaps via push-out + slide after the fact, not a sweep-test-before-
  you-move, so a character moving fast enough relative to a thin
  obstacle in one call can tunnel straight through it. Debug Draw is
  wireframe line segments only (`Sphere`/`Capsule` circles are
  16-segment polygon approximations, `ConvexHull` draws its AABB) with no
  rendering backend of its own. See
  [docs/limitations.md](docs/limitations.md) for both.


## License

This project is licensed under the GNU Lesser General Public License v2.1 - see the [LICENSE](LICENSE.md) file for details.

---
Copyright (C) 2026 [Algaves](https://github.com/algaves).
