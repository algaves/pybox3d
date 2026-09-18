# Debug draw

`examples/debug_draw_demo.py` builds a small scene and prints what
`RigidBody.debug_lines()` and `World.debug_contacts()`/
`.debug_joint_anchors()` return -- pure data, since this project has no
rendering backend of its own:

```python
from pybox3d import RigidBody, Vec3, World

world = World(gravity=(0, -9.81, 0))
ground = world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(5, 0.5, 5), mass=0.0))
box = world.add_body(RigidBody(Vec3(0, 0.5, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))
ball = world.add_body(RigidBody.sphere(Vec3(2, 3, 0), 0.5, mass=1.0))
world.add_joint(ground, ball, rest_length=3.0)

box.debug_lines()             # -> list[tuple[Vec3, Vec3]], 12 line segments
world.debug_contacts()        # -> list[tuple[Vec3, Vec3]], (point, normal) per overlap
world.debug_joint_anchors()   # -> list[tuple[Vec3, Vec3]], (anchor_a, anchor_b) per joint
```

Run it with:

```sh
uv run python examples/debug_draw_demo.py
```

See the [Debug Draw](../tutorials/debug-draw.md) tutorial for what each
method's output means and how to feed it to a renderer of your own.
