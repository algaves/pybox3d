# ContactInfo

```python
class ContactInfo(NamedTuple):
    normal: Vec3
    penetration: float
```

A named tuple returned by [`Box3D.overlaps()`](box3d.md) when two boxes
are intersecting.

## Attributes

| Attribute | Type | Description |
|---|---|---|
| `normal` | `Vec3` | The face-axis contact normal (v1 limitation: no edge-edge normals). |
| `penetration` | `float` | The scalar overlap depth along the contact normal. |

## Example

```python
from pybox3d import Box3D, Vec3

a = Box3D(Vec3(0, 0, 0), Vec3(1, 1, 1))
b = Box3D(Vec3(1.5, 0, 0), Vec3(1, 1, 1))
contact = a.overlaps(b)
if contact is not None:
    print(contact.normal, contact.penetration)
```

`ContactInfo` is returned as a regular `NamedTuple` -- it unpacks,
indexes, and compares like any other tuple:

```python
normal, penetration = a.overlaps(b)
assert penetration == a.overlaps(b).penetration
```

## Known limitation

`overlaps()` runs a full 15-axis SAT test to decide *whether* two boxes
overlap, but only derives the reported `normal`/`penetration` from face
axes. Edge-edge contact configurations report a face-based approximation
rather than an exact edge-edge normal — see
[Known limitations](../../limitations.md).