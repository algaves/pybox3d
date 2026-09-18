# CharacterMover methods

See the [CharacterMover class](../classes/charactermover.md) for
attributes, the constructor, and the example.

## Methods

| Method | Signature | Returns | Description |
|---|---|---|---|
| `move` | `move(world: World, displacement: VecLike) -> None` | `None` | Move by `displacement`, then resolve any resulting overlaps against `world`'s bodies. |

## Error cases

`move()` raises `TypeError` if `world` isn't a [`World`](../classes/world.md).
