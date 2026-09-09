#ifndef PYBOX3D_PY_WORLD_H
#define PYBOX3D_PY_WORLD_H

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "box3d/world.h"

typedef struct {
    PyObject_HEAD
    b3_World world;
} PyWorldObject;

extern PyTypeObject PyWorld_Type;

#define PyWorld_Check(op) PyObject_TypeCheck((op), &PyWorld_Type)

#endif /* PYBOX3D_PY_WORLD_H */
