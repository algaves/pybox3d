# Getting started

## Requirements

- Python 3.10+
- [`uv`](https://docs.astral.sh/uv/)
- A C compiler and CMake (used by the `scikit-build-core` build backend)

## Build and test

```sh
uv sync              # builds the C extension and installs dev dependencies
uv run pytest -v     # run the test suite
uv run ruff check .  # lint
uv run ruff format . # format
uv run mypy          # type-check
uv run python examples/falling_box_demo.py
```

## Working on the docs

Docs live under `docs/` and are built with [MkDocs](https://www.mkdocs.org/)
+ the [Nature theme](https://github.com/pkeilbach/mkdocs-nature). The
`docs` dependency group installs both:

```sh
uv sync --group docs      # install mkdocs + mkdocs-nature
uv run mkdocs serve       # live-reloading local preview at http://127.0.0.1:8000
uv run mkdocs build       # build the static site into site/
```

The type stub `src/pybox3d/_pybox3d.pyi` is hand-maintained and is the
best single reference for the compiled extension's API surface — keep the
API reference pages under `docs/api/` in sync with it as the C-API
(`src/pybox3d/_ext/*.c`) changes.
