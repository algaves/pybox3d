# pybox3d

A 3D rigid-body physics library with a NumPy-native binding, via the pure CPython C API.

![GitHub Last Commit](https://img.shields.io/github/last-commit/algaves/pybox3d.svg?style=flat-square)
![Python - Version](https://img.shields.io/badge/python-%3E%3D3.10-brightgreen?style=flat-square)
![PyPI - Version](https://img.shields.io/pypi/v/pybox3d?style=flat-square)
![Python - Implementation](https://img.shields.io/pypi/implementation/pybox3d?style=flat-square)
![PyPI - Wheel](https://img.shields.io/pypi/wheel/pybox3d?style=flat-square)
![Docs](https://img.shields.io/badge/docs-mkdocs-blue?style=flat-square)
![License](https://img.shields.io/badge/license-0BSD-green?style=flat-square)

Pure binding, no magic: `pybox3d` talks to `libbox3d`, a small
from-scratch C library, through hand-written **CPython C API** code -- no
ctypes/cffi/pybind11/nanobind, and `libbox3d` itself has zero Python
dependency.

## Features

| Shapes | Dynamics | Joints & Extras |
|---|---|---|
| `Box3D`, `Sphere`, `Capsule`, `ConvexHull`, `Compound`, `TriangleMesh`, `HeightField` -- containment, overlap (exact SAT + GJK/EPA), and raycast queries | `RigidBody` + `World` -- impulse-based rigid-body dynamics, sort-and-sweep broad phase, per-body sleeping, contact begin/end events, AABB/raycast queries, snapshot/restore | Spherical, Revolute, Prismatic, Weld, Motor, Wheel, Filter, and Parallel joints -- plus `DistanceJoint`, `CharacterMover` (kinematic move-and-slide), and Debug Draw |

## Installation

```sh
pip install pybox3d                                        # prebuilt wheel
pip install .                                              # from a local checkout
pip install git+https://github.com/algaves/pybox3d.git     # straight from GitHub
```

Prebuilt wheels ship for Python 3.10-3.14 on:

- **Windows**: x86_64, ARM64
- **Linux** (manylinux): x86_64, ARM64, ARMv7, PPC64LE, RISC-V (rv64gc)
- **Linux** (musllinux, e.g. Alpine): x86_64, ARM64, ARMv7, PPC64LE
- **macOS**: x86_64, ARM64

ARMv6 and BSD (FreeBSD, OpenBSD, etc.) have no precompiled-wheel platform
tag to target and are source-install-only; `pip install pybox3d` on those
platforms compiles `libbox3d` and the C extension locally (the code is
portable ANSI C11 with no platform-specific paths, so this works out of
the box). Source installs build `libbox3d` and the C extension at install
time and need nothing else.

## Quick Start

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

Complete runnable versions live in [`examples/`](examples/) -- see
`examples/falling_box_demo.py` for this one.

## Documentation & Examples

Full docs -- getting started, API reference, examples -- are built with
[MkDocs](https://www.mkdocs.org/):

```sh
uv run mkdocs serve    # live-reloading preview at http://127.0.0.1:8000
```

## Development

```sh
uv sync                  # builds the C extension, installs dev dependencies
uv run pytest -v         # run the test suite
uv run ruff check .      # lint
uv run ruff format .     # format
uv run mypy              # type-check
```

## Layout

| Path | Contents |
|---|---|
| `libbox3d/` | the pure-C library (zero Python dependency) |
| `src/pybox3d/` | Python package + type stub; `_ext/` is the CPython C-API glue |
| `tests/` | pytest suite |
| `examples/` | runnable demos |
| `docs/` | MkDocs source for the documentation site |

## Links

- [Documentation](https://algaves.github.io/pybox3d/) -- full API reference
- [Examples](examples/) -- runnable demos
- [Changelog](CHANGELOG.md)
- [TODO](TODO.md) -- missing features, tracked against Box2D 3D
- [Known v1 limitations](docs/limitations.md) -- documented scope cuts

## License

BSD Zero Clause License (0BSD) -- see [LICENSE](LICENSE.md). Free and
permissive, with no obligations.

---

Copyright (C) 2026 [Algaves](https://github.com/algaves).