# Examples

## Falling box

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

## Pendulum joint

`examples/pendulum_joint_demo.py` hangs a dynamic body from a static
anchor via a [`DistanceJoint`](api/joint.md) and lets it swing under
gravity, printing its position and distance from the anchor over time:

```python
from pybox3d import RigidBody, Vec3, World

world = World(gravity=(0, -9.81, 0))

anchor = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
bob = world.add_body(RigidBody(Vec3(2, 0, 0), Vec3(0.2, 0.2, 0.2), mass=1.0))
world.add_joint(anchor, bob, rest_length=2.0)

for _ in range(360):
    world.step(1 / 120)
```

Run it with:

```sh
uv run python examples/pendulum_joint_demo.py
```
