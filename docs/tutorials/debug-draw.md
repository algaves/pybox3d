# Debug Draw

Debug Draw isn't a type or a rendering backend -- it's three small
methods that return plain data (`Vec3` pairs), meant to be handed off to
whatever rendering setup you already have. This project has no
GUI/rendering dependency at all, in v1 or otherwise.

## 1. A body's wireframe: `RigidBody.debug_lines()`

```python
from pybox3d import RigidBody, Vec3, World

world = World(gravity=(0, -9.81, 0))
box = world.add_body(RigidBody(Vec3(0, 0.5, 0), Vec3(0.5, 0.5, 0.5), mass=1.0))

for start, end in box.debug_lines():
    my_renderer.draw_line(start, end)  # start/end are Vec3
```

Each entry is a `(start, end)` `Vec3` pair -- one line segment. A box
gets its exact 12 edges; `Sphere`/`Capsule` circles are approximated with
16-segment polygons; `ConvexHull` draws its AABB rather than an exact
wireframe (v1 has no computed edge topology for an arbitrary vertex set
-- see [Known limitations](../limitations.md)). The lines always reflect
the body's *current* pose -- call `debug_lines()` again after `world.step()`
if you want an updated wireframe.

## 2. What's touching: `World.debug_contacts()`

```python
for point, normal in world.debug_contacts():
    my_renderer.draw_point(point)
    my_renderer.draw_arrow(point, normal)
```

Returns a `(point, normal)` pair for every currently-overlapping body
pair in the world, from a fresh scan -- independent of (and redundant
with) `World.step()`'s own broad phase, since this is meant for
occasional visualization calls, not every step of a performance-critical
simulation. Unlike `step()`, it does **not** respect Filter joints: it
reports every raw geometric overlap regardless of collision filtering,
which is usually more useful for debugging than a silently-hidden one.

## 3. Where the joints are: `World.debug_joint_anchors()`

```python
for anchor_a, anchor_b in world.debug_joint_anchors():
    my_renderer.draw_point(anchor_a)
    my_renderer.draw_point(anchor_b)
    my_renderer.draw_line(anchor_a, anchor_b)  # visualizes any current joint "gap"
```

One `(anchor_a, anchor_b)` pair per joint, in world space, in the same
order `World.get_joint(0..joint_count-1)` would give you. A well-settled
rigid joint has `anchor_a` and `anchor_b` nearly coincident; a visible gap
is exactly the kind of drift [Known limitations](../limitations.md)
describes for velocity-only angular locks. Filter/Motor/Parallel joints
report each body's own center for both, since their anchors are unused.

## Key ideas

- **Pure data, no rendering**: none of these three methods draw
  anything -- they hand you `Vec3` pairs and get out of the way.
- **`debug_contacts()` doesn't respect Filter joints**: a deliberate
  difference from `World.step()`'s own collision detection, so you can
  see everything that's geometrically overlapping even if it's filtered
  from physics.
- **Call these when you need them, not every step**: `debug_contacts()`
  in particular re-scans every body pair independently of `step()`'s own
  broad phase.
