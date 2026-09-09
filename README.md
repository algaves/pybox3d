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
- [`uv`](https://docs.astral.sh/uv/)
- A C compiler and CMake (used by the `scikit-build-core` build backend)

## Getting started

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

## Layout

- `libbox3d/` -- the pure-C library (zero Python dependency).
- `src/pybox3d/_ext/` -- the CPython C-API glue, compiled into
  `pybox3d._pybox3d`.
- `src/pybox3d/` -- the Python package (`__init__.py` re-exports the
  compiled extension's public API; `_pybox3d.pyi` is its type stub, since
  mypy cannot see into the compiled `.so`).
- `tests/` -- pytest suite.
- `examples/` -- runnable demos.

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
