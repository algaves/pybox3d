#ifndef PYBOX3D_PY_RIGIDBODY_H
#define PYBOX3D_PY_RIGIDBODY_H

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "box3d/rigidbody.h"

/* A RigidBody handle is either:
 *   - standalone (owns_storage=1): `owned` holds the only copy of the
 *     data, independent of any World.
 *   - World-backed (owns_storage=0): the data lives in `world`'s
 *     b3_World.bodies[world_index]. `world` is a strong reference to the
 *     owning pybox3d.World instance (kept alive for as long as any handle
 *     references it, mirroring how e.g. numpy array views hold a `base`
 *     reference). Every access re-resolves the live pointer via
 *     b3_world_get_body rather than caching it, because World.add_body
 *     can realloc (move) the array and World.remove_body does a
 *     swap-remove (see PyRigidBody_Resolve). */
typedef struct {
    PyObject_HEAD
    b3_RigidBody owned;
    int owns_storage;
    PyObject *world;
    int world_index;
} PyRigidBodyObject;

extern PyTypeObject PyRigidBody_Type;

#define PyRigidBody_Check(op) PyObject_TypeCheck((op), &PyRigidBody_Type)

/* Creates a standalone handle owning a deep copy of *body. */
PyObject *PyRigidBody_FromOwned(const b3_RigidBody *body);

/* Creates a World-backed handle referencing world->bodies[index]. Steals
 * no reference: `world` is Py_INCREF'd internally. */
PyObject *PyRigidBody_FromWorldIndex(PyObject *world, int index);

/* Resolves the live b3_RigidBody* for `self`. Returns NULL with a
 * ValueError set if this is a World-backed handle whose index is no
 * longer valid (e.g. removed via World.remove_body). */
b3_RigidBody *PyRigidBody_Resolve(PyRigidBodyObject *self);

#endif /* PYBOX3D_PY_RIGIDBODY_H */
