#ifndef PYBOX3D_PY_WORLDSNAPSHOT_H
#define PYBOX3D_PY_WORLDSNAPSHOT_H

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "box3d/world.h"

/* An opaque, point-in-time recording of every body's transform/velocity
 * in a World, created via World.snapshot() and consumed by
 * World.restore() -- see docs. Unlike RigidBody/DistanceJoint/Joint, this
 * is never "world-backed": it owns a plain copy of the data (freed on
 * dealloc), independent of the World it was taken from. */
typedef struct {
    PyObject_HEAD
    b3_BodySnapshot *snapshots; /* owned heap array, NULL iff count == 0 */
    int count;
} PyWorldSnapshotObject;

extern PyTypeObject PyWorldSnapshot_Type;

#define PyWorldSnapshot_Check(op) PyObject_TypeCheck((op), &PyWorldSnapshot_Type)

/* Copies `count` entries from `snapshots` into a new owned
 * PyWorldSnapshotObject. Returns NULL with an exception set (MemoryError
 * or whatever PyObject_New raised) on failure. */
PyObject *PyWorldSnapshot_New(const b3_BodySnapshot *snapshots, int count);

#endif /* PYBOX3D_PY_WORLDSNAPSHOT_H */
