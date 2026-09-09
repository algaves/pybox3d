# Installation

## Requirements

- Python 3.10+
- [`uv`](https://docs.astral.sh/uv/) (recommended) or `pip`
- A C compiler and CMake (used by the `scikit-build-core` build backend)

`pybox3d` ships no prebuilt wheels yet, so `pip`/`uv` compile the
`libbox3d` C library and the `pybox3d._pybox3d` C extension for you at
install time.

## From GitHub

```sh
pip install git+https://github.com/algaves/pybox3d.git # straight from GitHub
```

## From a local checkout

```sh
pip install .
```

Both build `libbox3d` and the `pybox3d._pybox3d` C extension from source
as part of the install — there's nothing else to run afterwards.

!!! note "PyPI publishing"
    `pybox3d` is not on PyPI yet. Once a release is published, the usual
    `pip install pybox3d` / `uv add pybox3d` will work.

## Installing with uv

```sh
uv add "pybox3d @ git+https://github.com/algaves/pybox3d.git"
```

See [Quickstart](quickstart.md) for the next steps, or dive straight into
the [Tutorials](../tutorials/index.md).