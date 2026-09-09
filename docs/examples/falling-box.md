# Falling box

`examples/falling_box_demo.py` drops a dynamic box onto a static "ground"
box and prints its height/velocity as it settles, using the
`World`/`RigidBody` API end-to-end:

```python
from pybox3d import RigidBody, Vec3, World


def main() -> None:
    world = World(gravity=(0, -9.81, 0))

    ground = RigidBody(Vec3(0, -0.5, 0), Vec3(5, 0.5, 5), mass=0.0)
    world.add_body(ground)

    box = RigidBody(Vec3(0, 5, 0), Vec3(0.5, 0.5, 0.5), mass=1.0)
    handle = world.add_body(box)

    dt = 1 / 60
    for step in range(300):  # 5 seconds
        world.step(dt)

    print(f"Final resting height: {handle.position.y:.4f} (ground top is at y=0)")


if __name__ == "__main__":
    main()
```

Run it with:

```sh
uv run python examples/falling_box_demo.py
```

See the [Dropping a box](../tutorials/falling-box.md) tutorial for a
step-by-step walkthrough.