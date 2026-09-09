# pybox3d

A Python wrapper, written directly against the **CPython C API** (no
ctypes/cffi/pybind11/nanobind), around `libbox3d` -- a small, from-scratch
C library providing:

- **`Box3D`**: an axis-aligned or oriented 3D box (AABB/OBB), with
  point-containment, SAT-based overlap, and ray-cast queries.
- **`RigidBody`** / **`World`**: basic rigid-body dynamics scoped to
  boxes -- mass/inertia, semi-implicit Euler integration, naive O(n²)
  collision detection, and impulse-based resolution.

## Requirements

- Python 3.10+
- A C compiler and CMake (needed at install time, since `pybox3d` ships
  no prebuilt wheels yet -- pip/uv compile the extension for you via the
  `scikit-build-core` build backend)

## Installation

```sh
pip install .                                          # from a local checkout
pip install git+https://github.com/algaves/pybox3d.git # straight from GitHub
```

Both build `libbox3d` and the `pybox3d._pybox3d` C extension from source
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

The library has five public types:

- **`Vec3`** / **`Quat`** -- a 3D vector and rotation quaternion, with the
  usual arithmetic (`+`, `-`, `*`, dot/cross product, normalization,
  axis-angle construction, rotating a `Vec3` by a `Quat`, ...). Anywhere a
  `Vec3`/`Quat` is expected, a plain 3- or 4-element tuple/list works too.
- **`Box3D`** -- a standalone box shape: `contains_point()`,
  `overlaps()` (SAT-based, returns a `ContactInfo` normal/penetration or
  `None`), and `raycast()` (returns a `RayHit` or `None`).
- **`RigidBody`** -- a box-shaped body with position, orientation,
  velocities, mass, and `apply_force()`/`apply_impulse()`. A handle
  returned by `World.add_body()`/`World.get_body()` is "world-backed":
  reading/writing it reads/writes the body's live state inside the
  `World`.
- **`World`** -- owns a set of bodies and steps the simulation
  (`step(dt)`), with naive O(n²) collision detection and impulse-based
  resolution.

See [`docs/`](docs/) for the full API reference (build it locally with
`uv run mkdocs serve`, see below) and `examples/falling_box_demo.py` for a
complete runnable version of the snippet above.

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

```sh
uv build              # produces dist/pybox3d-<version>.tar.gz and a wheel
uv publish            # or: twine upload dist/*
```

`uv build` only produces a wheel for the platform/Python you run it on
(e.g. `cp311-cp311-linux_x86_64`), since `pybox3d` is a compiled C
extension. A real PyPI release covering multiple platforms/Python
versions would need to build one wheel per target -- typically via
[`cibuildwheel`](https://cibuildwheel.pypa.io/) in CI.
`.github/workflows/python-publish.yml` already wires up a single-wheel
release-on-GitHub-Release flow via PyPI trusted publishing (no API token
needed) -- fill in its commented-out `url:` line with the real PyPI
project URL, and note it still only builds one wheel per run.

## License

LGPL-2.1-or-later -- see [`LICENSE.md`](LICENSE.md).

Copyright (C) 2026 [Algaves](https://github.com/algaves).

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

## Known v1 limitations

These are deliberate scope cuts for a "basic" first version, not bugs:

- **Global contact materials**: `World.step` uses `World.default_restitution`
  / `World.default_friction` for every contact rather than per-body mixing
  rules. Each `RigidBody` still carries its own `restitution`/`friction`
  fields for forward compatibility, but v1 doesn't consult them.
- **No edge-edge contact normals**: `Box3D.overlaps()` runs a full 15-axis
  SAT test to decide *whether* two boxes overlap, but only derives the
  reported contact normal/penetration from face axes. Edge-edge contact
  configurations report a face-based approximation rather than an exact
  edge-edge normal.
- **Linear-only contact impulses**: collision response applies impulses
  without an angular (torque) contribution, which is enough for axis-aligned
  stacking/resting scenarios but is a simplification for tumbling contacts.
- **`World.remove_body` swap-removes**: removing a body moves the last
  body into the freed slot. Any `RigidBody` handle still holding the old
  index for that displaced body will silently resolve to a different body
  (or raise `ValueError` if the slot is now out of range). `World.get_body`
  always returns a fresh handle object, so `is` comparisons don't
  identify a body across two calls.
