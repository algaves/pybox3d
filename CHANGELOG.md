# Changelog

All notable changes to `pybox3d` are documented here. Loosely follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); no version has
been tagged/released yet, so everything so far lives under Unreleased.
See [TODO.md](TODO.md) for what's still missing.

## [Unreleased]

### Added

- `DistanceJoint`: a rigid center-to-center distance constraint between
  two bodies already in a `World`, via `World.add_joint()`/`get_joint()`/
  `remove_joint()`/`joint_count`. See `docs/api/joint.md` and
  `examples/pendulum_joint_demo.py`.
- `TODO.md`, tracking missing features against
  <https://box2d.org/documentation3d/topics.html>.
- `CHANGELOG.md` (this file).
- MkDocs documentation site (`docs/`, `mkdocs.yml`, `uv run mkdocs
  serve`).
- PyPI packaging metadata (`classifiers`, `keywords`, `project.urls`,
  `license`/`license-files`) and a GitHub Actions release workflow
  (`.github/workflows/python-publish.yml`).
- `LICENSE.md` (LGPL-2.1-or-later).
- README: Installation (`pip install`), Usage, Documentation, and
  Publishing sections.

### Fixed

- The built wheel no longer bundles the C extension's `.c`/`.h` sources
  (`wheel.exclude` in `pyproject.toml`) -- dead weight not needed at
  runtime, left over from `wheel.packages` copying the whole
  `src/pybox3d` tree.

## Initial implementation

- `Vec3` / `Quat`: 3D vector and rotation-quaternion math.
- `Box3D`: an axis-aligned or oriented box (AABB/OBB) with
  point-containment, SAT-based overlap, and ray-cast queries.
- `RigidBody` / `World`: box-scoped rigid-body dynamics -- mass/inertia,
  semi-implicit Euler integration, naive O(n²) collision detection, and
  impulse-based contact resolution.
