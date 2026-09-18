# A CharacterMover

[`CharacterMover`](../reference/classes/charactermover.md) is a kinematic
move-and-slide controller: unlike a `RigidBody`, it's never added to a
`World` and has no gravity/forces of its own -- you drive it directly
each step, and it only ever gets tested against a `World`'s bodies for
collision.

## 1. Build the world and the mover

```python
from pybox3d import Box3D, CharacterMover, RigidBody, Vec3, World

world = World(gravity=(0, -9.81, 0))
world.add_body(RigidBody(Vec3(0, -0.5, 0), Vec3(10, 0.5, 10), mass=0.0))  # ground
world.add_body(RigidBody(Vec3(3, 1, 0), Vec3(0.5, 2, 10), mass=0.0))  # a wall at x=3

shape = Box3D(Vec3(0, 0, 0), Vec3(0.4, 0.9, 0.4))
player = CharacterMover(Vec3(0, 5, 0), shape)
```

`world`'s gravity is only ever applied to *bodies in that world* --
`player` needs its own gravity integration, which is the point: you're
in full control of how it moves.

## 2. Fall and land

```python
dt = 1 / 60
gravity = Vec3(0, -9.81, 0)

for _ in range(90):  # 1.5 seconds
    player.velocity = player.velocity + gravity * dt
    player.move(world, player.velocity * dt)

print(player.position.y)  # settles near 0.9 (half_extents.y), resting on the ground
```

Each `move()` call does two things: moves `player` by the displacement
you pass in, then iteratively pushes it back out of anything it now
overlaps in `world` (up to `max_slide_iterations` times), removing the
into-the-surface component of `player.velocity` as it goes -- so the
next step's displacement (derived from that corrected velocity)
naturally stops sinking into the ground instead of fighting it every
frame.

## 3. Walk into a wall and slide

```python
walk_velocity = Vec3(2.0, 0, 2.0)  # diagonally toward and along the wall
for _ in range(90):  # 1.5 seconds
    player.velocity = Vec3(walk_velocity.x, player.velocity.y, walk_velocity.z)
    player.move(world, player.velocity * dt)

print(player.position.x)  # stops near 2.1 -- blocked by the wall (its face is at x=2.5)
print(player.position.z)  # keeps advancing to ~3.0 -- a slide, not a full stop
```

The X component of velocity gets zeroed out once `player` is pressed
against the wall (that's the "push-out" removing the into-the-surface
component), but Z is untouched -- exactly the "move and *slide*" a
character controller is for, versus a full stop.

## 4. Check `is_grounded`

```python
print(player.is_grounded)
```

`is_grounded` reflects only the *most recent* `move()` call -- it's not
a sticky "currently on the ground" flag. If a step resolves a contact
whose normal points up enough (`>= ground_normal_min_y`, default `0.5`),
it's `True` for that call; a step with nothing left to push out of
(already exactly resting) can read `False` again. Call `move()` every
step, even with a zero displacement, if you need this to stay current.

## Key ideas

- **`CharacterMover` is driven by you, not gravity**: there's no
  automatic integration -- fold gravity/input into `displacement`
  yourself, like the loops above do.
- **Collision is discrete, not swept**: `move()` tests overlaps *after*
  moving, the same caveat every other collision test in this library has
  -- keep `displacement` per call small relative to the thinnest
  obstacle in your scene. See [Known limitations](../limitations.md).
- **`velocity` is a convenience, not a hard requirement**: `move()`
  reads and corrects it for you, but you could just as easily compute
  `displacement` some other way each step.

Next tutorial: [Debug Draw](debug-draw.md).
