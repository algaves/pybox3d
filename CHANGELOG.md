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

- Release workflow (`.github/workflows/python-publish.yml`) now builds a
  full wheel matrix with `cibuildwheel` -- Windows (x86_64), Linux
  (x86_64 + ARM64 via QEMU, `manylinux`), and macOS (x86_64 + ARM64) for
  Python 3.10-3.14 -- runs the test suite against every wheel, and
  publishes to PyPI with trusted publishing. The sdist is built
  separately and trimmed to sources only (no `docs/`, `tests/`,
  `examples/`, `.github/`, or `uv.lock`).
- CI split into `tests.yaml` (cross-platform pytest matrix on every
  push/PR) and `check.yaml` (ruff lint + mypy typecheck) workflows.
- Installation docs/README now advertise the prebuilt wheels; source
  installs remain documented as the fallback for unsupported platforms.
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

- The PyPI publish workflow now ignores platform-tag problems: the
  `pypi-publish` job runs on `workflow_dispatch` as well as on release
  (both via trusted publishing on the `pypi` environment), and the README
  documents that locally-built Linux wheels carry an unsupported
  `linux_x86_64` tag which PyPI rejects -- repair them with
  `auditwheel repair` or publish through the CI workflow.
- `docs/getting-started.md` and `README.md` still referenced the
  Nature theme/`mkdocs-nature` package after the switch back above;
  updated both to `mkdocs-material`.
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
