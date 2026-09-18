#ifndef PYBOX3D_PY_JOINT_H
#define PYBOX3D_PY_JOINT_H

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "box3d/world.h"

/* A DistanceJoint handle is always World-backed: a joint only exists bound
 * to two specific bodies within one World, so (unlike RigidBody) there is
 * no meaningful standalone/owned variant. `world_id` is a generation-
 * checked stable id (see box3d/id.h), not a raw index, so this handle
 * keeps resolving to the same joint even if unrelated removals relocate
 * it in the World's dense array. */
typedef struct {
    PyObject_HEAD
    PyObject *world;
    b3_JointId world_id;
} PyDistanceJointObject;

extern PyTypeObject PyDistanceJoint_Type;

#define PyDistanceJoint_Check(op) PyObject_TypeCheck((op), &PyDistanceJoint_Type)

/* Creates a handle referencing the joint named by `id`. `world` is
 * Py_INCREF'd internally. */
PyObject *PyDistanceJoint_FromWorldId(PyObject *world, b3_JointId id);

/* Resolves the live b3_DistanceJoint* for `self`. Returns NULL with a
 * ValueError set if `id` is no longer valid (e.g. removed via
 * World.remove_joint, or as a side effect of World.remove_body). */
b3_DistanceJoint *PyDistanceJoint_Resolve(PyDistanceJointObject *self);

/* A single Python type covering every b3_JointKind other than Distance
 * (which keeps its own dedicated PyDistanceJoint type/class, unchanged
 * from before this cycle). `kind` (read-only) tells you which one a
 * given handle is; only the attributes that kind's b3_Joint::params
 * member actually has are settable -- accessing an attribute that
 * doesn't apply to this handle's kind raises AttributeError. Mirrors
 * PyDistanceJointObject's world-backed-handle pattern exactly. */
typedef struct {
    PyObject_HEAD
    PyObject *world;
    b3_JointId world_id;
} PyJointObject;

extern PyTypeObject PyJoint_Type;

#define PyJoint_Check(op) PyObject_TypeCheck((op), &PyJoint_Type)

/* Creates a handle referencing the joint named by `id`. `world` is
 * Py_INCREF'd internally. */
PyObject *PyJoint_FromWorldId(PyObject *world, b3_JointId id);

/* Resolves the live b3_Joint* for `self`. Returns NULL with a ValueError
 * set if `id` is no longer valid, mirroring PyDistanceJoint_Resolve. */
b3_Joint *PyJoint_Resolve(PyJointObject *self);

#endif /* PYBOX3D_PY_JOINT_H */
