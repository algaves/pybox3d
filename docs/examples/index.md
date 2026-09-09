# Examples

Complete, runnable programs from [`examples/`](https://github.com/algaves/pybox3d/tree/main/examples).
Use them as-is, or follow along with the [Tutorials](../tutorials/index.md)
for a step-by-step explanation of the same ideas.

Each example is a single file you can run with:

```sh
uv run python examples/<name>.py
```

| Demo | File | What it shows |
|---|---|---|
| [Falling box](falling-box.md) | `examples/falling_box_demo.py` | World/body/step API end-to-end |
| [Pendulum joint](pendulum.md) | `examples/pendulum_joint_demo.py` | A `DistanceJoint` anchor+bob pendulum |
| [Joint chain](joint-chain.md) | `examples/joint_chain_demo.py` | Multiple joints in sequence |
| [Dumbbell](dumbbell.md) | `examples/dumbbell_demo.py` | Two free bodies, no static anchor |