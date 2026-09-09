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

#endif /* PYBOX3D_PY_ERRORS_H */
