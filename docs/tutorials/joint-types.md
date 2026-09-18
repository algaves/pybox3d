# A simple robot arm

`DistanceJoint` is one constraint among nine -- this tutorial builds a
tiny robot-arm rig out of three of the others:
[Spherical](../reference/classes/joint.md) (a shoulder that rotates
freely), [Revolute](../reference/classes/joint.md) (a motorized hinge
elbow), and [Weld](../reference/classes/joint.md) (a hand rigidly
attached to the forearm).

## 1. Build the four bodies

```python
from pybox3d import RigidBody, Vec3, World

world = World(gravity=(0, 0, 0))

shoulder = world.add_body(RigidBody(Vec3(0, 0, 0), Vec3(0.15, 0.15, 0.15), mass=0.0))
upper_arm = world.add_body(RigidBody(Vec3(1, 0, 0), Vec3(1, 0.15, 0.15), mass=1.0))
forearm = world.add_body(RigidBody(Vec3(3, 0, 0), Vec3(1, 0.15, 0.15), mass=1.0))
hand = world.add_body(RigidBody(Vec3(4.3, 0, 0), Vec3(0.3, 0.3, 0.3), mass=0.5))
```

`shoulder` is static (`mass=0`); everything else is dynamic. Gravity is
off here so the elbow motor's effect is the only thing moving the rig --
with gravity on, the whole arm would swing like a pendulum from the
free-rotating shoulder, making the elbow's own rotation harder to read.

## 2. The shoulder: Spherical

```python
world.add_spherical_joint(shoulder, upper_arm, anchor_b=Vec3(-1, 0, 0))
```

A ball-and-socket point constraint: `shoulder`'s own center and
`upper_arm`'s local `(-1, 0, 0)` (its -x end, since its half-extent is 1)
are held coincident. Rotation is completely free -- there's no lock on
any rotation axis, unlike Revolute or Weld.

## 3. The elbow: Revolute, with a motor

```python
elbow = world.add_revolute_joint(
    upper_arm, forearm, axis=Vec3(0, 0, 1), anchor_a=Vec3(1, 0, 0), anchor_b=Vec3(-1, 0, 0),
    enable_motor=True, motor_speed=1.0, max_motor_effort=200.0,
)
```

A hinge: the same kind of point constraint as Spherical (anchored at
`upper_arm`'s +x end and `forearm`'s -x end this time), plus a lock on
every rotation axis *except* `axis` -- so the forearm can only swing
around Z, in the XY plane. `enable_motor` drives the *relative* angular
velocity around that axis toward `motor_speed` (1 rad/s here), clamped
by `max_motor_effort` as a per-step torque limit (200 N·m).

## 4. The hand: Weld

```python
world.add_weld_joint(forearm, hand, anchor_a=Vec3(1, 0, 0), anchor_b=Vec3(-1.3, 0, 0))
```

Full 6-DOF lock: the same point constraint again, plus a lock on
*every* rotation axis -- the hand can't move or rotate relative to the
forearm at all, once settled.

## 5. Step and check

```python
for _ in range(240):  # 2 seconds at 120 Hz
    world.step(1 / 120)

relative_speed = elbow.body_b.angular_velocity.z - elbow.body_a.angular_velocity.z
print(relative_speed)  # settles near 1.0 -- the motor's target

hand_offset = (hand.position - forearm.position).length()
print(hand_offset)  # stays near 2.3 (1 + 1.3, the two anchors' distances from their own centers)
```

Note the `elbow.body_b.angular_velocity - elbow.body_a.angular_velocity`
subtraction: each body's `angular_velocity` is its own *absolute*
rotation rate, so the elbow's *relative* speed (what the motor actually
targets) is the difference between the two.

## Key ideas

- **Every new joint kind but Filter/Motor/Parallel is anchored, like
  DistanceJoint**: `anchor_a`/`anchor_b` are body-local offsets, not
  always the body's center.
- **Point constraints compose**: Spherical, Revolute, and Weld all start
  from the same "hold these two anchors coincident" building block --
  they differ in what (if anything) they also lock about rotation.
- **A motor drives *relative* velocity, not a position target**: read it
  back the same way, as a difference between the two connected bodies'
  own `angular_velocity`/`linear_velocity`.

Next tutorial: [A CharacterMover](character-mover.md).
