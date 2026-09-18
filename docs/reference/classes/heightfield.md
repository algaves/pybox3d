# HeightField

`HeightField(center, heights, cell_size=1.0, orientation=None)`

A static-only terrain grid.

```python
class HeightField:
    center: Vec3
    orientation: Quat
    cell_size: float

    @property
    def rows(self) -> int: ...
    @property
    def cols(self) -> int: ...

    def __init__(
        self,
        center: VecLike,
        heights: Sequence[Sequence[float]],
        cell_size: float = 1.0,
        orientation: QuatLike | None = None,
    ) -> None: ...
```

## Attributes

| Attribute | Type | Description |
|---|---|---|
| `center` | `Vec3` | World-space center of the grid. |
| `orientation` | `Quat` | Rotation applied to the local grid. |
| `cell_size` | `float` | Grid spacing along the local X/Z axes. |
| `rows` | `int` | Number of grid rows (read-only). |
| `cols` | `int` | Number of grid columns (read-only). |

`heights` is a sequence of equal-length rows of local Y heights (row-major,
grid vertices centered on `center`), at most `HEIGHTFIELD_MAX_ROWS` x
`HEIGHTFIELD_MAX_COLS` (16 x 16). Each cell is triangulated into two
triangles for collision/raycast, generated on demand.

## Example

```python
from pybox3d import HeightField, Vec3

flat_ground = HeightField(
    Vec3(0, 0, 0),
    [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
    cell_size=2.0,
)
```

## Methods

See the [HeightField functions](../functions/heightfield.md) page for the
full method reference. See [Known limitations](../../limitations.md) for
the static-only, `contains_point()`-always-False, and
AABB-approximated-raycast caveats, and the mesh-pair collision
restriction.
