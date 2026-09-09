#include "py_world.h"
#include "py_rigidbody.h"
#include "py_vec3.h"
#include "py_errors.h"

static PyObject *world_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"gravity", "initial_capacity", NULL};
    PyObject *gravity_obj = NULL;
    int initial_capacity = 8;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "|Oi", kwlist, &gravity_obj, &initial_capacity
        )) {
        return NULL;
    }

    b3_Vec3 gravity = b3_vec3_make(0.0f, -9.81f, 0.0f);
    if (gravity_obj != NULL && PyVec3_Parse(gravity_obj, &gravity) < 0) {
        return NULL;
    }

    PyWorldObject *self = (PyWorldObject *)type->tp_alloc(type, 0);
    if (self == NULL) {
        return NULL;
    }
    b3_Status status = b3_world_init(&self->world, gravity, initial_capacity);
    if (status != B3_OK) {
        pybox3d_status_to_exception(status, "World()");
        Py_TYPE(self)->tp_free((PyObject *)self);
        return NULL;
    }
    return (PyObject *)self;
}

static void world_dealloc(PyWorldObject *self) {
    b3_world_destroy(&self->world);
    Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyObject *world_repr(PyWorldObject *self) {
    return PyUnicode_FromFormat("World(body_count=%d)", self->world.body_count);
}

static PyObject *world_get_gravity(PyWorldObject *self, void *closure) {
    (void)closure;
    return PyVec3_FromVec3(self->world.gravity);
}
static int world_set_gravity(PyWorldObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete gravity");
        return -1;
    }
    return PyVec3_Parse(value, &self->world.gravity);
}

static PyObject *world_get_body_count(PyWorldObject *self, void *closure) {
    (void)closure;
    return PyLong_FromLong(self->world.body_count);
}

static PyObject *world_get_default_restitution(PyWorldObject *self, void *closure) {
    (void)closure;
    return PyFloat_FromDouble((double)self->world.default_restitution);
}
static int world_set_default_restitution(PyWorldObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete default_restitution");
        return -1;
    }
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    self->world.default_restitution = (b3_real)v;
    return 0;
}

static PyObject *world_get_default_friction(PyWorldObject *self, void *closure) {
    (void)closure;
    return PyFloat_FromDouble((double)self->world.default_friction);
}
static int world_set_default_friction(PyWorldObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete default_friction");
        return -1;
    }
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    self->world.default_friction = (b3_real)v;
    return 0;
}

static PyGetSetDef world_getset[] = {
    {"gravity", (getter)world_get_gravity, (setter)world_set_gravity, "gravitational acceleration (Vec3)", NULL},
    {"body_count", (getter)world_get_body_count, NULL, "number of bodies currently in the world (read-only)", NULL},
    {"default_restitution", (getter)world_get_default_restitution, (setter)world_set_default_restitution,
     "global restitution used for every contact in v1 (see RigidBody.restitution docs)", NULL},
    {"default_friction", (getter)world_get_default_friction, (setter)world_set_default_friction,
     "global friction coefficient used for every contact in v1 (see RigidBody.friction docs)", NULL},
    {NULL},
};

static PyObject *world_add_body(PyWorldObject *self, PyObject *arg) {
    if (!PyRigidBody_Check(arg)) {
        PyErr_SetString(PyExc_TypeError, "add_body() argument must be a RigidBody");
        return NULL;
    }
    b3_RigidBody *source = PyRigidBody_Resolve((PyRigidBodyObject *)arg);
    if (source == NULL) {
        return NULL;
    }

    int index;
    b3_Status status = b3_world_add_body(&self->world, source, &index);
    if (pybox3d_status_to_exception(status, "World.add_body") < 0) {
        return NULL;
    }
    return PyRigidBody_FromWorldIndex((PyObject *)self, index);
}

static PyObject *world_get_body(PyWorldObject *self, PyObject *arg) {
    long index = PyLong_AsLong(arg);
    if (index == -1 && PyErr_Occurred()) {
        return NULL;
    }
    if (b3_world_get_body(&self->world, (int)index) == NULL) {
        PyErr_Format(PyExc_IndexError, "World.get_body: index %ld out of range", index);
        return NULL;
    }
    return PyRigidBody_FromWorldIndex((PyObject *)self, (int)index);
}

static PyObject *world_remove_body(PyWorldObject *self, PyObject *arg) {
    long index = PyLong_AsLong(arg);
    if (index == -1 && PyErr_Occurred()) {
        return NULL;
    }
    b3_Status status = b3_world_remove_body(&self->world, (int)index);
    if (pybox3d_status_to_exception(status, "World.remove_body") < 0) {
        return NULL;
    }
    Py_RETURN_NONE;
}

static PyObject *world_step(PyWorldObject *self, PyObject *arg) {
    double dt = PyFloat_AsDouble(arg);
    if (dt == -1.0 && PyErr_Occurred()) {
        return NULL;
    }
    b3_world_step(&self->world, (b3_real)dt);
    Py_RETURN_NONE;
}

static PyMethodDef world_methods[] = {
    {"add_body", (PyCFunction)world_add_body, METH_O,
     "add_body(body: RigidBody) -> RigidBody (deep-copies `body` in; returns a new world-backed handle)"},
    {"get_body", (PyCFunction)world_get_body, METH_O,
     "get_body(index: int) -> RigidBody (a fresh world-backed handle each call)"},
    {"remove_body", (PyCFunction)world_remove_body, METH_O,
     "remove_body(index: int) -> None (swap-remove; see RigidBody docs on stale handles)"},
    {"step", (PyCFunction)world_step, METH_O, "step(dt: float) -> None"},
    {NULL},
};

PyTypeObject PyWorld_Type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "pybox3d.World",
    .tp_basicsize = sizeof(PyWorldObject),
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = PyDoc_STR(
        "World(gravity=(0, -9.81, 0), initial_capacity=8)\n\n"
        "A rigid-body simulation world: naive O(n^2) box-box collision "
        "detection and simple impulse-based resolution."
    ),
    .tp_new = world_new,
    .tp_dealloc = (destructor)world_dealloc,
    .tp_repr = (reprfunc)world_repr,
    .tp_getset = world_getset,
    .tp_methods = world_methods,
};
