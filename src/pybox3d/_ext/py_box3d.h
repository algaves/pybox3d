#ifndef PYBOX3D_PY_BOX3D_H
#define PYBOX3D_PY_BOX3D_H

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "box3d/box3d.h"

typedef struct {
    PyObject_HEAD
    b3_Box3D value;
} PyBox3DObject;

extern PyTypeObject PyBox3D_Type;

/* Heap types created via PyStructSequence_NewType at module init time
 * (see pybox3d_box3d_module_init). ContactInfo fields: (normal, penetration).
 * RayHit fields: (t, point, normal). Both are returned as immutable
 * result records -- None represents "no overlap" / "no hit" rather than
 * exposing the C layer's separate `hit`/boolean field. */
extern PyTypeObject *PyContactInfo_Type;
extern PyTypeObject *PyRayHit_Type;

#define PyBox3D_Check(op) PyObject_TypeCheck((op), &PyBox3D_Type)

PyObject *PyBox3D_FromBox3D(b3_Box3D box);

/* Readies PyBox3D_Type, creates the ContactInfo/RayHit struct-sequence
 * types, and adds Box3D/ContactInfo/RayHit plus the BOX_KIND_AABB /
 * BOX_KIND_OBB int constants to `module`. Returns 0 on success, -1 with
 * an exception set on failure. */
int pybox3d_box3d_module_init(PyObject *module);

#endif /* PYBOX3D_PY_BOX3D_H */
