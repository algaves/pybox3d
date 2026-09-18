# TriangleMesh

`TriangleMesh(center, triangles, orientation=None)`

A static-only triangle soup.

```python
class TriangleMesh:
    center: Vec3
    orientation: Quat

    @property
    def triangle_count(self) -> int: ...

    def __init__(
        self,
        center: VecLike,
        triangles: Sequence[Sequence[VecLike]],
        orientation: QuatLike | None = None,
    ) -> None: ...
```

## Attributes

| Attribute | Type | Description |
|---|---|---|
| `center` | `Vec3` | World-space center. |
| `orientation` | `Quat` | Rotation applied to the local triangle vertices. |
| `triangle_count` | `int` | Number of triangles (read-only). |

`triangles` is a sequence of 3-point sequences, each point in **local**
space relative to `center` (same convention as `ConvexHull.vertices`), at
most `MESH_MAX_TRIANGLES` (64).

## Example

```python
from pybox3d import TriangleMesh, Vec3

ground = TriangleMesh(
    Vec3(0, 0, 0),
    [
        (Vec3(-5, 0, -5), Vec3(5, 0, -5), Vec3(5, 0, 5)),
        (Vec3(-5, 0, -5), Vec3(5, 0, 5), Vec3(-5, 0, 5)),
    ],
)
```

## Methods

See the [TriangleMesh functions](../functions/trianglemesh.md) page for
the full method reference. See
[Known limitations](../../limitations.md) for the static-only,
`contains_point()`-always-False, and AABB-approximated-raycast caveats,
and the mesh-pair collision restriction.
