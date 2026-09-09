"""pybox3d: a 3D box geometry + basic rigid-body physics library.

The heavy lifting lives in the compiled ``pybox3d._pybox3d`` extension
(hand-written against the CPython C API, wrapping the pure-C ``libbox3d``).
This module just re-exports its public surface.
"""

from pybox3d._pybox3d import (
    BOX_KIND_AABB,
    BOX_KIND_OBB,
    Box3D,
    Box3DError,
    CapacityError,
    ContactInfo,
    DistanceJoint,
    Quat,
    RayHit,
    RigidBody,
    Vec3,
    World,
    __version__,
)

__all__ = [
    "BOX_KIND_AABB",
    "BOX_KIND_OBB",
    "Box3D",
    "Box3DError",
    "CapacityError",
    "ContactInfo",
    "DistanceJoint",
    "Quat",
    "RayHit",
    "RigidBody",
    "Vec3",
    "World",
    "__version__",
]
