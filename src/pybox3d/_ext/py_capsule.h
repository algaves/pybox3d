#ifndef PYBOX3D_PY_CAPSULE_H
#define PYBOX3D_PY_CAPSULE_H

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "box3d/capsule.h"

typedef struct {
    PyObject_HEAD
    b3_Capsule value;
} PyCapsuleObject;

extern PyTypeObject PyCapsule3D_Type;

#define PyCapsule3D_Check(op) PyObject_TypeCheck((op), &PyCapsule3D_Type)

PyObject *PyCapsule3D_FromCapsule(b3_Capsule capsule);

#endif /* PYBOX3D_PY_CAPSULE_H */
