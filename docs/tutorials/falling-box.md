# Dropping a box

The shortest useful thing `pybox3d` can do: let a box fall and land on
the ground. It exercises every core idea in the library -- a `World`,
two `RigidBody`s, and `World.step`.

## 1. Create the world

A [`World`](../reference/classes/world.md) owns bodies and steps the
simulation. Its single argument is gravity:

```python
from pybox3d import RigidBody, Vec3, World

world = World(gravity=(0, -9.81, 0))
```

## 2. Add a static ground

A [`RigidBody`](../reference/classes/rigidbody.md) takes a position, half
extents, and a mass. `mass=0` marks the body **static** -- it never
moves:

```python
ground = RigidBody(Vec3(0, -0.5, 0), Vec3(5, 0.5, 5), mass=0.0)
world.add_body(ground)
```

The ground's *top* surface sits at `y = -0.5 + 0.5 = 0`.

## 3. Add a dynamic box

`mass=1.0` makes the box respond to gravity:

```python
box = RigidBody(Vec3(0, 5, 0), Vec3(0.5, 0.5, 0.5), mass=1.0)
handle = world.add_body(box)
```

Note what `add_body` returns: not the `box` you passed in, but a
**world-backed handle**. `world` deep-copies your body into its internal
storage, and the handle reads/writes that live state. That's why we keep
`handle` -- `box` would stay frozen at its original values.

## 4. Step the simulation

`World.step(dt)` advances time by `dt` seconds. Sixty steps per second,
five seconds total:

```python
for _ in range(300):
    world.step(1 / 60)
```

## 5. Read the result

```python
print(handle.position.y)  # settles near 0.5, resting on the ground
```

The box falls from `y = 5`, bounces a little, and rests with its center at
`y = 0.5` (its half-height) -- exactly on top of the ground.

## Key ideas

- **Static vs dynamic**: `mass=0` bodies are immovable; anything else is
  integrated under gravity.
- **World-backed handles**: `add_body`/`get_body` return live views into
  the world, not the object you handed in.
- **`step` drives everything**: nothing moves until you call it.

Next tutorial: [A swinging pendulum](pendulum.md).