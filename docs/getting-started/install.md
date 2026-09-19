# Installation

## Requirements

- Python 3.10+
- [`uv`](https://docs.astral.sh/uv/) (recommended) or `pip`
- A C compiler and CMake (used by the `scikit-build-core` build backend,
  only needed when installing from source)

## Prebuilt wheels

`pybox3d` ships prebuilt wheels for Python 3.10-3.14 on:

- **Windows**: x86_64 (`win_amd64`), ARM64 (`win_arm64`, build-only --
  see note below)
- **Linux, `manylinux`**: x86_64, ARM64 (`aarch64`), ARMv7 (`armv7l`),
  PPC64LE (`ppc64le`), RISC-V (`riscv64`)
- **Linux, `musllinux`** (e.g. Alpine): x86_64, ARM64 (`aarch64`), ARMv7
  (`armv7l`), PPC64LE (`ppc64le`) -- no musllinux wheel for RISC-V yet,
  since no musllinux RISC-V platform image exists
- **macOS**: x86_64, ARM64

!!! note "Windows ARM64 is build-only"
    There's no ARM64 Windows GitHub Actions runner, so the `win_arm64`
    wheel is cross-compiled but never executed in CI (its tests are
    skipped for that target specifically). It's expected to work -- the
    C code has no architecture-specific paths -- but hasn't been run
    against the test suite the way every other platform's wheel has.

!!! note "ARMv6 and BSD are source-install-only"
    ARMv6 (no `manylinux`/`musllinux` platform tag exists for it) and
    BSD variants such as FreeBSD/OpenBSD (no CI runners or wheel platform
    tag ecosystem) don't get a precompiled wheel. `pip install pybox3d`
    on these platforms falls back to a source build automatically --
    `libbox3d` is portable ANSI C11 with no platform-specific code, so
    this works without any extra steps, just a C compiler and CMake (see
    "Requirements" above).

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
    `../../.github/workflows/publish.yml`).

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