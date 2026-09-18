# Capsule methods

See the [Capsule class](../classes/capsule.md) for attributes and the
constructor.

## Instance methods

| Method | Signature | Returns | Description |
|---|---|---|---|
| `contains_point` | `contains_point(point: VecLike) -> bool` | `bool` | Whether `point` lies inside the capsule (cylinder body or either cap). |
| `aabb` | `aabb() -> tuple[Vec3, Vec3]` | `(Vec3 min, Vec3 max)` | The capsule's world-space axis-aligned bounding box. |
| `overlaps` | `overlaps(other: Box3D \| Sphere \| Capsule) -> ContactInfo \| None` | `ContactInfo \| None` | Overlap test (GJK/EPA); `None` if disjoint. |
| `raycast` | `raycast(origin: VecLike, direction: VecLike, max_t: float = ...) -> RayHit \| None` | `RayHit \| None` | Ray/capsule intersection (cylinder body or either cap); `None` if no hit within `max_t`. |

## `overlaps` return value

Returns a [`ContactInfo`](../classes/contactinfo.md) when the two shapes
are intersecting. See [Known limitations](../../limitations.md) for the
GJK/EPA precision caveat on perfectly round shapes.

## `raycast` return value

Returns a [`RayHit`](../classes/rayhit.md) when the ray intersects the
capsule; `t` is the ray parameter at the hit
(`point = origin + t * direction`).
