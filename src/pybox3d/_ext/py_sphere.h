#ifndef PYBOX3D_PY_SPHERE_H
#define PYBOX3D_PY_SPHERE_H

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "box3d/sphere.h"

typedef struct {
    PyObject_HEAD
    b3_Sphere value;
} PySphereObject;

extern PyTypeObject PySphere_Type;

#define PySphere_Check(op) PyObject_TypeCheck((op), &PySphere_Type)

PyObject *PySphere_FromSphere(b3_Sphere sphere);

#endif /* PYBOX3D_PY_SPHERE_H */
