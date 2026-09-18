# TriangleMesh methods

See the [TriangleMesh class](../classes/trianglemesh.md) for attributes
and the constructor.

## Instance methods

| Method | Signature | Returns | Description |
|---|---|---|---|
| `contains_point` | `contains_point(point: VecLike) -> bool` | `bool` | Always `False` in v1 (see below). |
| `aabb` | `aabb() -> tuple[Vec3, Vec3]` | `(Vec3 min, Vec3 max)` | The mesh's world-space axis-aligned bounding box. |
| `overlaps` | `overlaps(other: ShapeLike) -> ContactInfo \| None` | `ContactInfo \| None` | Per-triangle GJK/EPA against a convex shape; `None` if disjoint or `other` is another `TriangleMesh`/`HeightField`. |
| `raycast` | `raycast(origin: VecLike, direction: VecLike, max_t: float = ...) -> RayHit \| None` | `RayHit \| None` | Approximated via the mesh's AABB (see below); `None` if no hit within `max_t`. |

## `overlaps` return value

Tests every triangle (after an AABB quick-reject) against `other`'s
support function via GJK/EPA, keeping the deepest overlap found. Only
supports a convex `other` (`Box3D`/`Sphere`/`Capsule`/`ConvexHull`/
`Compound`) -- another `TriangleMesh`/`HeightField` always returns
`None`, since both sides being static-only means this never comes up in
`World.step()`. See [Known limitations](../../limitations.md) for the
GJK/EPA precision caveat on round-shape pairs.

## `raycast` return value

Returns a [`RayHit`](../classes/rayhit.md) when the ray intersects the
mesh's **AABB** -- v1 has no acceleration structure to test individual
triangles against (see [Known limitations](../../limitations.md)), so
this is a conservative approximation, not an exact surface hit.
