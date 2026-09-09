#include "py_errors.h"

PyObject *PyBox3D_Error = NULL;
PyObject *PyBox3D_CapacityError = NULL;

int pybox3d_errors_init(PyObject *module) {
    PyBox3D_Error = PyErr_NewException("pybox3d.Box3DError", PyExc_RuntimeError, NULL);
    if (PyBox3D_Error == NULL) {
        return -1;
    }
    if (PyModule_AddObjectRef(module, "Box3DError", PyBox3D_Error) < 0) {
        return -1;
    }

    PyBox3D_CapacityError = PyErr_NewException("pybox3d.CapacityError", PyBox3D_Error, NULL);
    if (PyBox3D_CapacityError == NULL) {
        return -1;
    }
    if (PyModule_AddObjectRef(module, "CapacityError", PyBox3D_CapacityError) < 0) {
        return -1;
    }

    return 0;
}

int pybox3d_status_to_exception(b3_Status status, const char *context) {
    switch (status) {
        case B3_OK:
            return 0;
        case B3_ERR_OUT_OF_MEMORY:
            PyErr_Format(PyExc_MemoryError, "%s: out of memory", context);
            return -1;
        case B3_ERR_INVALID_ARGUMENT:
            PyErr_Format(PyExc_ValueError, "%s: invalid argument", context);
            return -1;
        case B3_ERR_CAPACITY_EXCEEDED:
            PyErr_Format(PyBox3D_CapacityError, "%s: capacity exceeded", context);
            return -1;
        case B3_ERR_INDEX_OUT_OF_RANGE:
            PyErr_Format(PyExc_IndexError, "%s: index out of range", context);
            return -1;
        default:
            PyErr_Format(PyBox3D_Error, "%s: unknown error (status=%d)", context, (int)status);
            return -1;
    }
}
