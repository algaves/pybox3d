# Working on the documentation

The docs live under `docs/` and are built with [MkDocs](https://www.mkdocs.org/)
+ [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/). The
`docs` dependency group installs both:

```sh
uv sync --group docs      # install mkdocs + mkdocs-material
uv run mkdocs serve       # live-reloading local preview at http://127.0.0.1:8000
uv run mkdocs build       # build the static site into site/
```

`mkdocs build --strict` fails on any broken link or missing page — use it
before pushing docs changes.

## Keeping the API reference in sync

The type stub `src/pybox3d/_pybox3d.pyi` is hand-maintained and is the
best single reference for the compiled extension's API surface. Keep the
reference pages under `docs/reference/` in sync with it (and with the
C-API in `src/pybox3d/_ext/*.c`) as the API evolves:

- `docs/reference/modules/` — the package module (`__version__`, the
  `BOX_KIND_*` constants, the `VecLike`/`QuatLike` type aliases, and the
  `Box3DError`/`CapacityError` exception hierarchy).
- `docs/reference/classes/` — one page per public type (attributes and
  constructor).
- `docs/reference/functions/` — one page per type's methods.

Changing any attribute, method, constant, or type alias means updating the
stub *and* the matching reference page.