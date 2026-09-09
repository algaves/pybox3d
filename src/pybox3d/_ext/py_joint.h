#ifndef PYBOX3D_PY_JOINT_H
#define PYBOX3D_PY_JOINT_H

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "box3d/world.h"

/* A DistanceJoint handle is always World-backed: a joint only exists bound
 * to two specific body indices within one World, so (unlike RigidBody)
 * there is no meaningful standalone/owned variant. */
typedef struct {
    PyObject_HEAD
    PyObject *world;
    int world_index;
} PyDistanceJointObject;

extern PyTypeObject PyDistanceJoint_Type;

#define PyDistanceJoint_Check(op) PyObject_TypeCheck((op), &PyDistanceJoint_Type)

/* Creates a handle referencing world->joints[index]. `world` is
 * Py_INCREF'd internally. */
PyObject *PyDistanceJoint_FromWorldIndex(PyObject *world, int index);

/* Resolves the live b3_DistanceJoint* for `self`. Returns NULL with a
 * ValueError set if `index` is no longer valid (e.g. removed via
 * World.remove_joint, which swap-removes like World.remove_body). */
b3_DistanceJoint *PyDistanceJoint_Resolve(PyDistanceJointObject *self);

#endif /* PYBOX3D_PY_JOINT_H */
