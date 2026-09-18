# ConvexHull methods

See the [ConvexHull class](../classes/convexhull.md) for attributes and
the constructor.

## Instance methods

| Method | Signature | Returns | Description |
|---|---|---|---|
| `contains_point` | `contains_point(point: VecLike) -> bool` | `bool` | Whether `point` lies inside the hull (exact -- see below). |
| `aabb` | `aabb() -> tuple[Vec3, Vec3]` | `(Vec3 min, Vec3 max)` | The hull's world-space axis-aligned bounding box. |
| `overlaps` | `overlaps(other: ShapeLike) -> ContactInfo \| None` | `ContactInfo \| None` | Overlap test (GJK/EPA); `None` if disjoint. |
| `raycast` | `raycast(origin: VecLike, direction: VecLike, max_t: float = ...) -> RayHit \| None` | `RayHit \| None` | Approximated via the hull's AABB (see below); `None` if no hit within `max_t`. |

## `contains_point`

Implemented exactly, as a GJK intersection test between the hull and a
zero-radius point -- not an approximation, unlike `raycast()` below.

## `overlaps` return value

Returns a [`ContactInfo`](../classes/contactinfo.md) when the two shapes
are intersecting. See [Known limitations](../../limitations.md) for the
GJK/EPA precision caveat on round-shape pairs.

## `raycast` return value

Returns a [`RayHit`](../classes/rayhit.md) when the ray intersects the
hull's **AABB** -- v1 has no face data to test the exact hull surface
against (see [Known limitations](../../limitations.md)), so this is a
conservative approximation, not an exact surface hit.
