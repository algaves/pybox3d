# ConvexHull

`ConvexHull(center, vertices, orientation=None)`

A convex polyhedron from a caller-supplied, already-convex vertex set.

```python
class ConvexHull:
    center: Vec3
    orientation: Quat

    @property
    def vertex_count(self) -> int: ...
    @property
    def vertices(self) -> tuple[Vec3, ...]: ...

    def __init__(
        self, center: VecLike, vertices: Sequence[VecLike], orientation: QuatLike | None = None
    ) -> None: ...
```

## Attributes

| Attribute | Type | Description |
|---|---|---|
| `center` | `Vec3` | World-space center. |
| `orientation` | `Quat` | Rotation applied to the local vertices. |
| `vertex_count` | `int` | Number of vertices (read-only). |
| `vertices` | `tuple[Vec3, ...]` | Local (unrotated, uncentered) vertices, as passed to the constructor (read-only). |

`vertices` in the constructor are in **local** space relative to `center`
(the same convention `Box3D.half_extents` uses), not world space.

## Example

```python
from pybox3d import ConvexHull, Vec3

cube_vertices = [Vec3(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
hull = ConvexHull(Vec3(0, 0, 0), cube_vertices)
```

## Methods

See the [ConvexHull functions](../functions/convexhull.md) page for the
full method reference. See [Known limitations](../../limitations.md) for
the vertex cap, the no-quickhull-construction and approximate-raycast
caveats, and the GJK/EPA precision caveat on round-shape pairs.
