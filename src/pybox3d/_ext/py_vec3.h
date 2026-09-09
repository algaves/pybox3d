#ifndef PYBOX3D_PY_VEC3_H
#define PYBOX3D_PY_VEC3_H

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "box3d/vec3.h"

typedef struct {
    PyObject_HEAD
    b3_Vec3 value;
} PyVec3Object;

extern PyTypeObject PyVec3_Type;

#define PyVec3_Check(op) PyObject_TypeCheck((op), &PyVec3_Type)

/* Returns a new reference to a Vec3 wrapping `v`, or NULL with an
 * exception set on allocation failure. */
PyObject *PyVec3_FromVec3(b3_Vec3 v);

/* Accepts either a pybox3d.Vec3 instance or any sequence of 3 numbers
 * (tuple, list, ...) and writes the result to *out. Returns 0 on success,
 * -1 with an exception set on failure. */
int PyVec3_Parse(PyObject *obj, b3_Vec3 *out);

#endif /* PYBOX3D_PY_VEC3_H */
