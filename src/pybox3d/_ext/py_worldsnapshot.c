#include "py_worldsnapshot.h"

#include <stdlib.h>
#include <string.h>

PyObject *PyWorldSnapshot_New(const b3_BodySnapshot *snapshots, int count) {
    PyWorldSnapshotObject *self = PyObject_New(PyWorldSnapshotObject, &PyWorldSnapshot_Type);
    if (self == NULL) {
        return NULL;
    }
    self->snapshots = NULL;
    self->count = 0;
    if (count > 0) {
        self->snapshots = (b3_BodySnapshot *)malloc((size_t)count * sizeof(b3_BodySnapshot));
        if (self->snapshots == NULL) {
            Py_DECREF(self);
            return PyErr_NoMemory();
        }
        memcpy(self->snapshots, snapshots, (size_t)count * sizeof(b3_BodySnapshot));
        self->count = count;
    }
    return (PyObject *)self;
}

static PyObject *worldsnapshot_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    (void)type;
    (void)args;
    (void)kwds;
    PyErr_SetString(
        PyExc_TypeError, "WorldSnapshot cannot be constructed directly; use World.snapshot()"
    );
    return NULL;
}

static void worldsnapshot_dealloc(PyWorldSnapshotObject *self) {
    free(self->snapshots);
    Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyObject *worldsnapshot_repr(PyWorldSnapshotObject *self) {
    return PyUnicode_FromFormat("WorldSnapshot(body_count=%d)", self->count);
}

static Py_ssize_t worldsnapshot_len(PyWorldSnapshotObject *self) {
    return (Py_ssize_t)self->count;
}

static PySequenceMethods worldsnapshot_as_sequence = {
    .sq_length = (lenfunc)worldsnapshot_len,
};

PyTypeObject PyWorldSnapshot_Type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "pybox3d.WorldSnapshot",
    .tp_basicsize = sizeof(PyWorldSnapshotObject),
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = PyDoc_STR(
        "An opaque, point-in-time recording of every body's transform/velocity in a World, "
        "created via World.snapshot() and consumed by World.restore(). len(snapshot) is how many "
        "bodies it recorded. For simple state recording/replay: call World.snapshot() once per "
        "step you want to be able to rewind to, keep the results in a list, and hand any of them "
        "back to World.restore() later."
    ),
    .tp_new = worldsnapshot_new,
    .tp_dealloc = (destructor)worldsnapshot_dealloc,
    .tp_repr = (reprfunc)worldsnapshot_repr,
    .tp_as_sequence = &worldsnapshot_as_sequence,
};
