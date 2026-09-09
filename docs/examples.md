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

## Joint chain

`examples/joint_chain_demo.py` links several bodies end-to-end with
[`DistanceJoint`](api/joint.md)s hanging from a static anchor, showing
`World.add_joint()` used repeatedly to build a multi-link chain rather
than a single pendulum:

```python
from pybox3d import RigidBody, Vec3, World

world = World(gravity=(0, -9.81, 0))

link_length = 0.5
anchor = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.1, 0.1, 0.1), mass=0.0))
previous = anchor
for i in range(3):
    y = -(i + 1) * link_length
    link = world.add_body(RigidBody(Vec3(0, y, 0), Vec3(0.1, 0.1, 0.1), mass=1.0))
    world.add_joint(previous, link, rest_length=link_length)
    previous = link

for _ in range(600):
    world.step(1 / 120)
```

The demo deliberately keeps the chain short (3 links). `World.step` solves
each joint once per step with no inner iteration loop, so a correction at
one joint only reaches its immediate neighbor within a given step; a
longer chain converges to a visibly saggy steady state well past
`rest_length` rather than a taut one -- see
[Known limitations](limitations.md).

Run it with:

```sh
uv run python examples/joint_chain_demo.py
```

## Dumbbell (two dynamic bodies, no anchor)

`examples/dumbbell_demo.py` joins two *free* dynamic bodies (no static
body at all) with a single `DistanceJoint`, then gives them equal and
opposite spin velocities plus a shared drift velocity. The pair rotates
around its common center of mass while that center drifts in a straight
line, with the joint holding the two bodies at a constant distance
throughout -- demonstrating that a joint works symmetrically between two
moving dynamic bodies, not just an anchor-plus-bob pendulum:

```python
from pybox3d import RigidBody, Vec3, World

world = World(gravity=(0, 0, 0))

a = world.add_body(RigidBody(Vec3(-1, 0, 0), Vec3(0.2, 0.2, 0.2), mass=1.0))
b = world.add_body(RigidBody(Vec3(1, 0, 0), Vec3(0.2, 0.2, 0.2), mass=1.0))
world.add_joint(a, b, rest_length=2.0)

drift = Vec3(0.3, 0, 0)
a.linear_velocity = Vec3(0, 1, 0) + drift
b.linear_velocity = Vec3(0, -1, 0) + drift

for _ in range(240):
    world.step(1 / 120)
```

Run it with:

```sh
uv run python examples/dumbbell_demo.py
```
