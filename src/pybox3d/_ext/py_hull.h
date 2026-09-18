#ifndef PYBOX3D_PY_HULL_H
#define PYBOX3D_PY_HULL_H

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "box3d/hull.h"

typedef struct {
    PyObject_HEAD
    b3_ConvexHull value;
} PyConvexHullObject;

extern PyTypeObject PyConvexHull_Type;

#define PyConvexHull_Check(op) PyObject_TypeCheck((op), &PyConvexHull_Type)

PyObject *PyConvexHull_FromHull(b3_ConvexHull hull);

#endif /* PYBOX3D_PY_HULL_H */
