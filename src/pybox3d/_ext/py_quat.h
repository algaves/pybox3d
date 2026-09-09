#ifndef PYBOX3D_PY_QUAT_H
#define PYBOX3D_PY_QUAT_H

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "box3d/quat.h"

typedef struct {
    PyObject_HEAD
    b3_Quat value;
} PyQuatObject;

extern PyTypeObject PyQuat_Type;

#define PyQuat_Check(op) PyObject_TypeCheck((op), &PyQuat_Type)

PyObject *PyQuat_FromQuat(b3_Quat q);

/* Accepts a pybox3d.Quat instance or a sequence of 4 numbers (x, y, z, w).
 * Returns 0 on success, -1 with an exception set on failure. */
int PyQuat_Parse(PyObject *obj, b3_Quat *out);

#endif /* PYBOX3D_PY_QUAT_H */
