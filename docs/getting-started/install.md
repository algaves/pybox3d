# Installation

## Requirements

- Python 3.10+
- [`uv`](https://docs.astral.sh/uv/) (recommended) or `pip`
- A C compiler and CMake (used by the `scikit-build-core` build backend,
  only needed when installing from source)

## Prebuilt wheels

`pybox3d` ships prebuilt wheels for Python 3.10-3.14 on:

- **Windows** (x86_64)
- **Linux** (x86_64 and ARM64, `manylinux`)
- **macOS** (x86_64 and ARM64)

Installing from a wheel needs no C toolchain:

```sh
pip install pybox3d
```

```sh
uv add pybox3d
```

!!! note "PyPI availability"
    The wheels are built and published for every GitHub release via
    [`cibuildwheel`](https://cibuildwheel.pypa.io/) (see
    `.github/workflows/python-publish.yml`).

## From source

If no wheel matches your platform/Python combination (or you're
installing an unreleased version), `pip`/`uv` compile the `libbox3d` C
library and the `pybox3d._pybox3d` C extension for you at install time.

### From GitHub

```sh
pip install git+https://github.com/algaves/pybox3d.git # straight from GitHub
```

### From a local checkout

```sh
pip install .
```

Both build `libbox3d` and the `pybox3d._pybox3d` C extension from source
as part of the install — there's nothing else to run afterwards.

## Installing with uv

```sh
uv add "pybox3d @ git+https://github.com/algaves/pybox3d.git"
```

See [Quickstart](quickstart.md) for the next steps, or dive straight into
the [Tutorials](../tutorials/index.md).