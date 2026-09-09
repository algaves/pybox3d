# pybox3d

A Python wrapper, written directly against the **CPython C API** (no
ctypes/cffi/pybind11/nanobind), around `libbox3d` — a small, from-scratch
C library providing:

- **`Box3D`** — an axis-aligned or oriented 3D box (AABB/OBB), with
  point-containment, SAT-based overlap, and ray-cast queries.
- **`RigidBody`** / **`World`** — basic rigid-body dynamics scoped to
  boxes: mass/inertia, semi-implicit Euler integration, naive O(n²)
  collision detection, and impulse-based resolution.
- **`DistanceJoint`** — a rigid center-to-center distance constraint for
  connecting two bodies (pendulums, chains, ...).

## Layout

- `libbox3d/` — the pure-C library (zero Python dependency).
- `src/pybox3d/_ext/` — the CPython C-API glue, compiled into
  `pybox3d._pybox3d`.
- `src/pybox3d/` — the Python package (`__init__.py` re-exports the
  compiled extension's public API; `_pybox3d.pyi` is its type stub, since
  mypy cannot see into the compiled `.so`).
- `tests/` — pytest suite.
- `examples/` — runnable demos.

See [Getting started](getting-started.md) to build the extension and run
the tests, the API reference for `Vec3`, `Quat`, `Box3D`, `RigidBody`,
`World`, and `DistanceJoint`, [Known limitations](limitations.md) for
deliberate v1 scope cuts, and [Roadmap](roadmap.md) for what's planned
next.
