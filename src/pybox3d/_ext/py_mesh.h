#ifndef PYBOX3D_PY_MESH_H
#define PYBOX3D_PY_MESH_H

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "box3d/mesh.h"

typedef struct {
    PyObject_HEAD
    b3_TriangleMesh value;
} PyTriangleMeshObject;

extern PyTypeObject PyTriangleMesh_Type;

#define PyTriangleMesh_Check(op) PyObject_TypeCheck((op), &PyTriangleMesh_Type)

PyObject *PyTriangleMesh_FromMesh(b3_TriangleMesh mesh);

#endif /* PYBOX3D_PY_MESH_H */
