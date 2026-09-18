#ifndef PYBOX3D_PY_COMPOUND_H
#define PYBOX3D_PY_COMPOUND_H

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "box3d/compound.h"

typedef struct {
    PyObject_HEAD
    b3_Compound value;
} PyCompoundObject;

extern PyTypeObject PyCompound_Type;

#define PyCompound_Check(op) PyObject_TypeCheck((op), &PyCompound_Type)

PyObject *PyCompound_FromCompound(b3_Compound compound);

/* Parses a Python sequence of child entries -- each a (local_position,
 * shape) or (local_position, local_orientation, shape) sequence, `shape`
 * being any registered leaf shape object -- into `out` (capacity
 * B3_COMPOUND_MAX_CHILDREN), writing the count to *out_count. Returns 0
 * on success, -1 with an exception set (ValueError if too many, TypeError
 * for a nested Compound or an unrecognized shape type) otherwise. */
int PyCompound_ParseChildren(
    PyObject *children_obj, b3_CompoundChild out[B3_COMPOUND_MAX_CHILDREN], int *out_count
);

#endif /* PYBOX3D_PY_COMPOUND_H */
