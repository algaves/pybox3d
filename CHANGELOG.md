# Changelog

Also published as [Changelog](docs/changelog.md) in the MkDocs site --
keep both in sync when either changes.

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
- `examples/joint_chain_demo.py` (a short chain of `DistanceJoint`s
  hanging from a static anchor) and `examples/dumbbell_demo.py` (two
  free dynamic bodies joined by a `DistanceJoint`, no static anchor).
- MkDocs [Roadmap](docs/roadmap.md) and this Changelog, published as
  pages in the docs site (mirroring `TODO.md`/`CHANGELOG.md`).

### Changed

- MkDocs theme switched from Material to
  [Nature](https://github.com/pkeilbach/mkdocs-nature).

### Fixed

- `docs/getting-started.md` and `README.md` still referenced the old
  Material theme/`mkdocs-material` package after the switch above;
  updated both to `mkdocs-nature`.
- `docs/index.md` never mentioned `DistanceJoint` after it was added.
- Task-list checkboxes (`- [x]`/`- [ ]`) in the Roadmap/Changelog pages
  rendered as literal `[x]`/`[ ]` text: the Material-to-Nature theme
  switch dropped `pymdown-extensions` (a transitive Material dependency),
  which `pymdownx.tasklist` needs. Added it back as a direct `docs`
  dependency and enabled `pymdownx.tasklist` in `mkdocs.yml`.
- The built wheel no longer bundles the C extension's `.c`/`.h` sources
  (`wheel.exclude` in `pyproject.toml`) -- dead weight not needed at
  runtime, left over from `wheel.packages` copying the whole
  `src/pybox3d` tree.

### Documented

- Newly-discovered limitation: joints solve once per step with no inner
  iteration loop, so a joint chain longer than ~2-3 links sags well past
  its `rest_length` (found while writing `joint_chain_demo.py`; tracked
  in `TODO.md`/`docs/limitations.md`, not fixed yet).

## Initial implementation

- `Vec3` / `Quat`: 3D vector and rotation-quaternion math.
- `Box3D`: an axis-aligned or oriented box (AABB/OBB) with
  point-containment, SAT-based overlap, and ray-cast queries.
- `RigidBody` / `World`: box-scoped rigid-body dynamics -- mass/inertia,
  semi-implicit Euler integration, naive O(n²) collision detection, and
  impulse-based contact resolution.
