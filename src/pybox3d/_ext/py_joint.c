#include "py_joint.h"
#include "py_world.h"
#include "py_rigidbody.h"

PyObject *PyDistanceJoint_FromWorldIndex(PyObject *world, int index) {
    PyDistanceJointObject *self = PyObject_New(PyDistanceJointObject, &PyDistanceJoint_Type);
    if (self == NULL) {
        return NULL;
    }
    Py_INCREF(world);
    self->world = world;
    self->world_index = index;
    return (PyObject *)self;
}

b3_DistanceJoint *PyDistanceJoint_Resolve(PyDistanceJointObject *self) {
    b3_DistanceJoint *joint =
        b3_world_get_joint(&((PyWorldObject *)self->world)->world, self->world_index);
    if (joint == NULL) {
        PyErr_SetString(
            PyExc_ValueError,
            "this DistanceJoint is no longer valid: its World entry was removed "
            "(World.remove_joint uses swap-remove, which can also make a handle "
            "silently refer to a different joint)"
        );
        return NULL;
    }
    return joint;
}

static PyObject *joint_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    (void)type;
    (void)args;
    (void)kwds;
    PyErr_SetString(
        PyExc_TypeError, "DistanceJoint cannot be constructed directly; use World.add_joint()"
    );
    return NULL;
}

static void joint_dealloc(PyDistanceJointObject *self) {
    Py_XDECREF(self->world);
    Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyObject *joint_repr(PyDistanceJointObject *self) {
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) {
        PyErr_Clear();
        return PyUnicode_FromString("<DistanceJoint (stale World reference)>");
    }
    return PyUnicode_FromFormat(
        "DistanceJoint(body_a=%d, body_b=%d, rest_length=%g)", joint->body_a_index,
        joint->body_b_index, (double)joint->rest_length
    );
}

static PyObject *joint_get_rest_length(PyDistanceJointObject *self, void *closure) {
    (void)closure;
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) return NULL;
    return PyFloat_FromDouble((double)joint->rest_length);
}
static int joint_set_rest_length(PyDistanceJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete rest_length");
        return -1;
    }
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) return -1;
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    if (v < 0.0) {
        PyErr_SetString(PyExc_ValueError, "rest_length must be >= 0");
        return -1;
    }
    joint->rest_length = (b3_real)v;
    return 0;
}

static PyObject *joint_get_body_a(PyDistanceJointObject *self, void *closure) {
    (void)closure;
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) return NULL;
    return PyRigidBody_FromWorldIndex(self->world, joint->body_a_index);
}

static PyObject *joint_get_body_b(PyDistanceJointObject *self, void *closure) {
    (void)closure;
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) return NULL;
    return PyRigidBody_FromWorldIndex(self->world, joint->body_b_index);
}

static PyGetSetDef joint_getset[] = {
    {"rest_length", (getter)joint_get_rest_length, (setter)joint_set_rest_length,
     "target distance between body_a.position and body_b.position", NULL},
    {"body_a", (getter)joint_get_body_a, NULL,
     "first connected body (a fresh world-backed RigidBody handle each access, read-only)", NULL},
    {"body_b", (getter)joint_get_body_b, NULL,
     "second connected body (a fresh world-backed RigidBody handle each access, read-only)", NULL},
    {NULL},
};

PyTypeObject PyDistanceJoint_Type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "pybox3d.DistanceJoint",
    .tp_basicsize = sizeof(PyDistanceJointObject),
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = PyDoc_STR(
        "A rigid distance constraint between two bodies' centers, created via "
        "World.add_joint(). Holds the distance between body_a.position and "
        "body_b.position at rest_length (a stiff rod, not a spring). v1 scope "
        "cut: anchored at body centers only, no per-body anchor offset, no "
        "angular contribution -- see README limitations."
    ),
    .tp_new = joint_new,
    .tp_dealloc = (destructor)joint_dealloc,
    .tp_repr = (reprfunc)joint_repr,
    .tp_getset = joint_getset,
};
