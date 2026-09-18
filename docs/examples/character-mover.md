# Character mover

`examples/character_mover_demo.py` drives a
[`CharacterMover`](../reference/classes/charactermover.md) through a
fall-and-land, then a walk-into-a-wall-and-slide, against a small
`World` scene:

```python
from pybox3d import Box3D, CharacterMover, RigidBody, Vec3, World

world = World(gravity=(0, -9.81, 0))
world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(10, 0.5, 10), mass=0.0))  # ground
world.add_body(RigidBody(Vec3(3, 1, 0), Vec3(0.5, 2, 10), mass=0.0))  # a wall

shape = Box3D(Vec3(0, 0, 0), Vec3(0.4, 0.9, 0.4))
player = CharacterMover(Vec3(0, 5, 0), shape)

dt = 1 / 60
gravity = Vec3(0, -9.81, 0)
for _ in range(90):
    player.velocity = player.velocity + gravity * dt
    player.move(world, player.velocity * dt)
```

Run it with:

```sh
uv run python examples/character_mover_demo.py
```

`CharacterMover` is not a `RigidBody` and is never added to `world` --
gravity has to be integrated by hand, as the loop above shows, which is
also what makes it possible to drive with arbitrary input instead of
physics forces. See the [A CharacterMover](../tutorials/character-mover.md)
tutorial for the step-by-step version, including the wall-slide half of
the demo.
