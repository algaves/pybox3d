# Contributing to pybox3d

Thanks for considering a contribution. This project is a small, from-scratch
3D physics library: a pure-C core (`libbox3d/`) with hand-written CPython
C-API bindings (`src/pybox3d/_ext/`) -- no pybind11/nanobind/Cython, and
`libbox3d` itself has zero Python dependency. Keeping that separation clean
is one of the project's core design goals; see [Layout](README.md#layout)
in the README for the full directory breakdown.

## Getting set up

```sh
uv sync --group dev --group docs   # builds the C extension, installs dev + docs dependencies
uv run pytest -v                   # run the test suite
uv run ruff check .                # lint
uv run ruff format .               # format
uv run mypy                        # type-check
uv run mkdocs serve                # live-reloading docs preview at http://127.0.0.1:8000
```

After changing any C source under `libbox3d/` or `src/pybox3d/_ext/`,
re-run `uv sync --reinstall-package pybox3d` (or just `uv sync`) to rebuild
the extension before re-running the tests -- `uv run pytest` alone will not
pick up C-side changes on its own.

## Before opening a PR

- **New C code must compile warning-clean.** This project targets zero
  warnings under `-Wall -Wextra`; a clean local build is
  `rm -rf build && cmake -S . -B /tmp/b3build && cmake --build /tmp/b3build
  -j$(nproc)`. If a CI job configures with `-DBOX3D_WERROR=ON`, run the same
  locally (`cmake -S . -B /tmp/b3build -DBOX3D_WERROR=ON && cmake --build
  /tmp/b3build`) to catch anything before pushing.
- **`ruff check .`, `ruff format .`, and `mypy` must pass** with no new
  suppressions unless there's a genuine reason (and a comment explaining
  it). New test files need adding to `[[tool.mypy.overrides]]` in
  `pyproject.toml` (see existing entries for the pattern).
- **Keep the docs in sync.** This project treats documentation as part of
  the change, not a follow-up:
  - Flip the relevant checkbox in **both** `TODO.md` and
    `docs/roadmap.md` when a tracked item lands (they're kept identical
    except for link style -- see the top of either file).
  - Add a `CHANGELOG.md` **and** `docs/changelog.md` entry (same content,
    `docs/changelog.md` uses relative Markdown links instead of bare
    backticks -- compare existing entries for the exact convention).
  - Update `docs/limitations.md` if the change closes a documented v1
    scope cut, or introduces a new one deliberately.
  - New public types/methods need a `docs/reference/classes/*.md` +
    `docs/reference/functions/*.md` page pair and a `mkdocs.yml` nav
    entry; run `uv run mkdocs build --strict` to catch broken links/nav
    before opening the PR (then `rm -rf site`).
- **Tests**: one behavior per test, following the existing style in
  `tests/` (e.g. `tests/test_joint_kinds.py`). Use the `approx` fixture
  from `tests/conftest.py` for float comparisons -- `libbox3d` uses
  `float` (float32), not `double`, throughout, so a plain `==` or
  overly-tight tolerance will be flaky.
- **No unrelated formatting/refactor churn** in a PR that's meant to fix
  or add one thing -- keep diffs reviewable.

## Design conventions worth knowing before you dive in

- `b3_RigidBody` and its embedded shapes have **no internal heap
  pointers** (fixed-capacity arrays for things like `ConvexHull` vertices
  or `Compound` children) so a body stays safely copyable by value. New
  shape/joint data should follow the same pattern rather than reaching
  for `malloc` inside a per-body struct.
- New failure cases funnel through the existing `b3_Status` enum
  (`libbox3d/include/box3d/world.h`) and `pybox3d_status_to_exception()`
  (`src/pybox3d/_ext/py_errors.c`) on the Python-binding side, rather than
  parallel error-handling paths.
- World-backed Python types (joints, and anything only constructible via
  `World.add_*`) follow the handle pattern in `py_joint.h`/`py_world.h`:
  `tp_new` raises `TypeError` for direct construction, and a `Resolve()`
  helper re-fetches the live C pointer on every access rather than
  caching a raw pointer (the backing arrays can realloc/swap-remove).
- Known, deliberate v1 simplifications are documented in
  [`docs/limitations.md`](docs/limitations.md), not silently left
  unexplained -- if you're about to work around one of those, read that
  entry first; there's usually a reason (often documented in detail) for
  why it's scoped the way it is.

## Reporting bugs / requesting features

Use the issue templates (bug report / feature request) when opening a new
issue -- they ask for the details (repro steps, environment, proposed API)
that make a report actionable. See [SECURITY.md](SECURITY.md) instead if
you're reporting a security vulnerability.

## License

By contributing, you agree your contribution is licensed under this
project's [0BSD license](LICENSE.md).
