# ContactInfo

```python
class ContactInfo(NamedTuple):
    normal: Vec3
    penetration: float
    point: Vec3
```

A named tuple returned by [`Box3D`](box3d.md)/[`Sphere`](sphere.md)/
[`Capsule`](capsule.md)/[`ConvexHull`](convexhull.md)/
[`Compound`](compound.md)/[`TriangleMesh`](trianglemesh.md)/
[`HeightField`](heightfield.md) `.overlaps()` when two shapes are
intersecting.

## Attributes

| Attribute | Type | Description |
|---|---|---|
| `normal` | `Vec3` | Unit-length contact normal, pointing from the first shape towards the second. |
| `penetration` | `float` | The scalar overlap depth along the contact normal. |
| `point` | `Vec3` | One representative world-space contact point (see Known limitation, below). |

## Example

```python
from pybox3d import Box3D, Vec3

a = Box3D(Vec3(0, 0, 0), Vec3(1, 1, 1))
b = Box3D(Vec3(1.5, 0, 0), Vec3(1, 1, 1))
contact = a.overlaps(b)
if contact is not None:
    print(contact.normal, contact.penetration, contact.point)
```

`ContactInfo` is returned as a regular `NamedTuple` -- it unpacks,
indexes, and compares like any other tuple:

```python
normal, penetration, point = a.overlaps(b)
assert penetration == a.overlaps(b).penetration
```

## Known limitations

- Every pair reports exactly one representative contact `point`, never a
  full multi-point manifold (box-vs-box places it at the two closest
  edge points for an edge-edge contact, or the midpoint of each box's
  support point along `normal` for a face contact; every other pair
  places it at the midpoint of each shape's support point along
  `normal`).
- Any pair other than box-vs-box goes through a generic GJK/EPA core, so
  `normal`/`penetration` on perfectly round shapes (e.g. sphere-vs-sphere)
  is only an approximation -- see
  [Known limitations](../../limitations.md).
