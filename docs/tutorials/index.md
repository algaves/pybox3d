# Tutorials

Step-by-step walkthroughs that build up your understanding of `pybox3d`
one concept at a time. Unlike the [Examples](../examples/index.md) --
which are complete runnable scripts -- these pages explain *why* each
piece works as you go.

1. [Dropping a box](falling-box.md) — worlds, bodies, gravity, and
   `World.step`.
2. [A swinging pendulum](pendulum.md) — connecting two bodies with a
   `DistanceJoint`.
3. [A joint chain](joint-chain.md) — several joints in sequence, and a
   real v1 limitation.
4. [A drifting dumbbell](dumbbell.md) — two free dynamic bodies joined
   with no static anchor.
5. [A simple robot arm](joint-types.md) — Spherical, Revolute, and Weld
   joints combined in one rig.
6. [A CharacterMover](character-mover.md) — a kinematic move-and-slide
   controller, driven outside the `World`'s own physics.
7. [Debug Draw](debug-draw.md) — pure-data wireframes, contacts, and
   joint anchors, with no rendering backend of its own.

Each tutorial is 5-10 minutes and assumes you've finished the
[Quickstart](../getting-started/quickstart.md).