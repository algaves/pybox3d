#ifndef PYBOX3D_PY_HEIGHTFIELD_H
#define PYBOX3D_PY_HEIGHTFIELD_H

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "box3d/heightfield.h"

typedef struct {
    PyObject_HEAD
    b3_HeightField value;
} PyHeightFieldObject;

extern PyTypeObject PyHeightField_Type;

#define PyHeightField_Check(op) PyObject_TypeCheck((op), &PyHeightField_Type)

PyObject *PyHeightField_FromHeightField(b3_HeightField hf);

#endif /* PYBOX3D_PY_HEIGHTFIELD_H */
