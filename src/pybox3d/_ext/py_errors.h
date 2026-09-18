#ifndef PYBOX3D_PY_ERRORS_H
#define PYBOX3D_PY_ERRORS_H

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "box3d/world.h"

/* pybox3d.Box3DError -- base exception for all pybox3d-specific errors. */
extern PyObject *PyBox3D_Error;
/* pybox3d.CapacityError(Box3DError) -- raised for B3_ERR_CAPACITY_EXCEEDED. */
extern PyObject *PyBox3D_CapacityError;

int pybox3d_errors_init(PyObject *module);

/* Translates a non-OK b3_Status into the appropriate Python exception
 * (already set via PyErr_SetString/Format) and returns -1. Returns 0
 * without touching the error indicator if status == B3_OK. Callers
 * propagate a -1 result as their own failure (e.g. `return NULL;`). */
int pybox3d_status_to_exception(b3_Status status, const char *context);

/* Rejects a non-finite (NaN/inf) scalar with a ValueError naming `what`
 * (e.g. "mass", "rest_length"). Returns 0 (no exception set) if `v` is
 * finite, -1 (ValueError set) otherwise -- same calling convention as
 * PyVec3_Parse/PyQuat_Parse. For bare-scalar setters/constructors that
 * don't go through those (RigidBody.mass, DistanceJoint.rest_length,
 * etc.); PyVec3_Parse/PyQuat_Parse do their own equivalent check inline
 * since they already loop over components. */
int pybox3d_require_finite(double v, const char *what);

#endif /* PYBOX3D_PY_ERRORS_H */
