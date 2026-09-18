#ifndef PYBOX3D_PY_SHAPE_H
#define PYBOX3D_PY_SHAPE_H

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "box3d/shape.h"

/* Cross-shape-type helpers: Box3D/Sphere/Capsule are separate Python
 * types, but b3_shape_overlap() (and RigidBody.shape) need to move
 * between any of them and the generic b3_Shape tagged union. */

/* Parses any registered shape object (Box3D, Sphere, Capsule) into a
 * generic b3_Shape. Returns 0 on success, -1 with a TypeError set
 * otherwise. */
int PyShape_Parse(PyObject *obj, b3_Shape *out);

/* Wraps a copy of `shape` into a fresh Python object of the matching
 * registered type (Box3D/Sphere/Capsule). */
PyObject *PyShape_Wrap(b3_Shape shape);

#endif /* PYBOX3D_PY_SHAPE_H */
