---
icon: material/cube
---

# pybox3d

A Python wrapper, written directly against the **CPython C API** (no
ctypes/cffi/pybind11/nanobind), around `libbox3d` — a small, from-scratch
C library providing:

- **`Box3D`** / **`Sphere`** / **`Capsule`** / **`ConvexHull`** /
  **`Compound`** / **`TriangleMesh`** / **`HeightField`** — 3D shapes
  with point-containment, overlap (exact SAT for box-vs-box, a generic
  GJK/EPA core otherwise), and ray-cast queries. The last two are
  always-static level geometry.
- **`RigidBody`** / **`World`** — basic rigid-body dynamics: mass/
  inertia, semi-implicit Euler integration, naive O(n²) collision
  detection, and impulse-based resolution.
- **`DistanceJoint`** / **`Joint`** — nine joint kinds for connecting two
  bodies: a rigid-or-spring distance constraint (pendulums, chains, ...),
  plus Spherical, Revolute, Prismatic, Weld, Motor, Wheel, Filter, and
  Parallel.

## Layout

- `libbox3d/` — the pure-C library (zero Python dependency).
- `src/pybox3d/_ext/` — the CPython C-API glue, compiled into
  `pybox3d._pybox3d`.
- `src/pybox3d/` — the Python package (`__init__.py` re-exports the
  compiled extension's public API; `_pybox3d.pyi` is its type stub, since
  mypy cannot see into the compiled `.so`).
- `tests/` — pytest suite.
- `examples/` — runnable demos.

See [Getting started](getting-started/quickstart.md) to build the
extension and run the tests, the [Reference](reference/classes/vec3.md)
for `Vec3`, `Quat`, `Box3D`, `Sphere`, `Capsule`, `ConvexHull`,
`Compound`, `TriangleMesh`, `HeightField`, `RigidBody`, `World`,
`DistanceJoint`, and `Joint`, [Known limitations](limitations.md) for
deliberate v1 scope cuts, and [Roadmap](roadmap.md) for what's planned
next.
