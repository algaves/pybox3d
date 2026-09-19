# Quickstart

## Requirements

- Python 3.10+
- [`uv`](https://docs.astral.sh/uv/)
- A C compiler and CMake (used by the `scikit-build-core` build backend)

See [Installation](install.md) for how to get the package into your
environment. For development against a local checkout:

```sh
uv sync              # builds the C extension and installs dev dependencies
uv run pytest -v     # run the test suite
uv run ruff check .  # lint
uv run ruff format . # format
uv run mypy          # type-check
uv run python examples/falling_box_demo.py
```

## Your first simulation

Open a Python session and drop a box onto a static "ground" box:

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

That covers the whole API surface in one breath. The
[Tutorials](../tutorials/index.md) walk through each piece in more depth,
and the [Examples](../examples/index.md) show complete runnable programs
from `examples/`.

## Working with NumPy

`Vec3` and `Quat` convert to and from NumPy, and anywhere the API takes a
`VecLike`/`QuatLike` a NumPy array works directly:

```python
import numpy as np
from pybox3d import RigidBody, Vec3, World

world = World(gravity=(0, -9.81, 0))
body = world.add_body(RigidBody(np.array([0, 5, 0]), np.array([0.5, 0.5, 0.5]), mass=1.0))

for _ in range(300):
    world.step(1 / 60)

pos = body.position.to_numpy()  # float32 ndarray, shape (3,)
np.asarray(body.linear_velocity)  # zero-copy, read-only view
```

See [NumPy interop](numpy-interop.md) for the full conversion API.

## Working on the docs

See [Documentation](documentation.md).
