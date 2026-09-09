#include "py_rigidbody.h"
#include "py_world.h"
#include "py_vec3.h"
#include "py_quat.h"
#include "py_box3d.h"

PyObject *PyRigidBody_FromOwned(const b3_RigidBody *body) {
    PyRigidBodyObject *self = PyObject_New(PyRigidBodyObject, &PyRigidBody_Type);
    if (self == NULL) {
        return NULL;
    }
    self->owned = *body;
    self->owns_storage = 1;
    self->world = NULL;
    self->world_index = -1;
    return (PyObject *)self;
}

PyObject *PyRigidBody_FromWorldIndex(PyObject *world, int index) {
    PyRigidBodyObject *self = PyObject_New(PyRigidBodyObject, &PyRigidBody_Type);
    if (self == NULL) {
        return NULL;
    }
    self->owns_storage = 0;
    Py_INCREF(world);
    self->world = world;
    self->world_index = index;
    return (PyObject *)self;
}

b3_RigidBody *PyRigidBody_Resolve(PyRigidBodyObject *self) {
    if (self->owns_storage) {
        return &self->owned;
    }
    b3_RigidBody *body = b3_world_get_body(&((PyWorldObject *)self->world)->world, self->world_index);
    if (body == NULL) {
        PyErr_SetString(
            PyExc_ValueError,
            "this RigidBody is no longer valid: its World entry was removed "
            "(World.remove_body uses swap-remove, which can also make a "
            "handle silently refer to a different body -- see docs)"
        );
        return NULL;
    }
    return body;
}

static PyObject *rigidbody_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"position", "half_extents", "mass", NULL};
    PyObject *position_obj, *half_extents_obj;
    double mass = 0.0;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "OO|d", kwlist, &position_obj, &half_extents_obj, &mass
        )) {
        return NULL;
    }
    b3_Vec3 position, half_extents;
    if (PyVec3_Parse(position_obj, &position) < 0) {
        return NULL;
    }
    if (PyVec3_Parse(half_extents_obj, &half_extents) < 0) {
        return NULL;
    }

    PyRigidBodyObject *self = (PyRigidBodyObject *)type->tp_alloc(type, 0);
    if (self == NULL) {
        return NULL;
    }
    b3_rigidbody_init_box(&self->owned, position, half_extents, (b3_real)mass);
    self->owns_storage = 1;
    self->world = NULL;
    self->world_index = -1;
    return (PyObject *)self;
}

static void rigidbody_dealloc(PyRigidBodyObject *self) {
    Py_XDECREF(self->world);
    Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyObject *rigidbody_repr(PyRigidBodyObject *self) {
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) {
        PyErr_Clear();
        return PyUnicode_FromString("<RigidBody (stale World reference)>");
    }
    return PyUnicode_FromFormat(
        "RigidBody(position=(%g, %g, %g), mass=%g%s)",
        (double)body->position.x, (double)body->position.y, (double)body->position.z,
        (double)body->mass,
        self->owns_storage ? "" : ", world-backed"
    );
}

static PyObject *rigidbody_get_position(PyRigidBodyObject *self, void *closure) {
    (void)closure;
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return NULL;
    return PyVec3_FromVec3(body->position);
}
static int rigidbody_set_position(PyRigidBodyObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete position");
        return -1;
    }
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return -1;
    b3_Vec3 v;
    if (PyVec3_Parse(value, &v) < 0) return -1;
    body->position = v;
    b3_rigidbody_sync_shape(body);
    return 0;
}

static PyObject *rigidbody_get_orientation(PyRigidBodyObject *self, void *closure) {
    (void)closure;
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return NULL;
    return PyQuat_FromQuat(body->orientation);
}
static int rigidbody_set_orientation(PyRigidBodyObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete orientation");
        return -1;
    }
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return -1;
    b3_Quat q;
    if (PyQuat_Parse(value, &q) < 0) return -1;
    body->orientation = b3_quat_normalize(q);
    b3_rigidbody_sync_shape(body);
    return 0;
}

static PyObject *rigidbody_get_linear_velocity(PyRigidBodyObject *self, void *closure) {
    (void)closure;
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return NULL;
    return PyVec3_FromVec3(body->linear_velocity);
}
static int rigidbody_set_linear_velocity(PyRigidBodyObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete linear_velocity");
        return -1;
    }
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return -1;
    return PyVec3_Parse(value, &body->linear_velocity);
}

static PyObject *rigidbody_get_angular_velocity(PyRigidBodyObject *self, void *closure) {
    (void)closure;
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return NULL;
    return PyVec3_FromVec3(body->angular_velocity);
}
static int rigidbody_set_angular_velocity(PyRigidBodyObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete angular_velocity");
        return -1;
    }
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return -1;
    return PyVec3_Parse(value, &body->angular_velocity);
}

static PyObject *rigidbody_get_mass(PyRigidBodyObject *self, void *closure) {
    (void)closure;
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return NULL;
    return PyFloat_FromDouble((double)body->mass);
}
static int rigidbody_set_mass(PyRigidBodyObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete mass");
        return -1;
    }
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return -1;
    double mass = PyFloat_AsDouble(value);
    if (mass == -1.0 && PyErr_Occurred()) return -1;
    if (mass < 0.0) {
        PyErr_SetString(PyExc_ValueError, "mass must be >= 0 (0 means static)");
        return -1;
    }
    b3_rigidbody_set_mass(body, (b3_real)mass);
    return 0;
}

static PyObject *rigidbody_get_inv_mass(PyRigidBodyObject *self, void *closure) {
    (void)closure;
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return NULL;
    return PyFloat_FromDouble((double)body->inv_mass);
}

static PyObject *rigidbody_get_restitution(PyRigidBodyObject *self, void *closure) {
    (void)closure;
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return NULL;
    return PyFloat_FromDouble((double)body->restitution);
}
static int rigidbody_set_restitution(PyRigidBodyObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete restitution");
        return -1;
    }
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return -1;
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    body->restitution = (b3_real)v;
    return 0;
}

static PyObject *rigidbody_get_friction(PyRigidBodyObject *self, void *closure) {
    (void)closure;
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return NULL;
    return PyFloat_FromDouble((double)body->friction);
}
static int rigidbody_set_friction(PyRigidBodyObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete friction");
        return -1;
    }
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return -1;
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    body->friction = (b3_real)v;
    return 0;
}

static PyObject *rigidbody_get_shape(PyRigidBodyObject *self, void *closure) {
    (void)closure;
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return NULL;
    return PyBox3D_FromBox3D(body->shape);
}

static PyObject *rigidbody_get_is_static(PyRigidBodyObject *self, void *closure) {
    (void)closure;
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return NULL;
    if (body->inv_mass == 0.0f) {
        Py_RETURN_TRUE;
    }
    Py_RETURN_FALSE;
}

static PyGetSetDef rigidbody_getset[] = {
    {"position", (getter)rigidbody_get_position, (setter)rigidbody_set_position, "world-space position (Vec3)", NULL},
    {"orientation", (getter)rigidbody_get_orientation, (setter)rigidbody_set_orientation, "orientation (Quat)", NULL},
    {"linear_velocity", (getter)rigidbody_get_linear_velocity, (setter)rigidbody_set_linear_velocity, "linear velocity (Vec3)", NULL},
    {"angular_velocity", (getter)rigidbody_get_angular_velocity, (setter)rigidbody_set_angular_velocity, "angular velocity, rad/s (Vec3)", NULL},
    {"mass", (getter)rigidbody_get_mass, (setter)rigidbody_set_mass, "mass; 0 means static (infinite mass)", NULL},
    {"inv_mass", (getter)rigidbody_get_inv_mass, NULL, "1/mass, or 0 for static bodies (read-only)", NULL},
    {"restitution", (getter)rigidbody_get_restitution, (setter)rigidbody_set_restitution, "bounciness coefficient (informational in v1; World uses global defaults, see World.default_restitution)", NULL},
    {"friction", (getter)rigidbody_get_friction, (setter)rigidbody_set_friction, "friction coefficient (informational in v1; see World.default_friction)", NULL},
    {"shape", (getter)rigidbody_get_shape, NULL, "current collision shape as a Box3D snapshot (read-only)", NULL},
    {"is_static", (getter)rigidbody_get_is_static, NULL, "True if mass == 0", NULL},
    {NULL},
};

static PyObject *rigidbody_apply_force(PyRigidBodyObject *self, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"force", "point", NULL};
    PyObject *force_obj;
    PyObject *point_obj = Py_None;
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "O|O", kwlist, &force_obj, &point_obj)) {
        return NULL;
    }
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return NULL;

    b3_Vec3 force;
    if (PyVec3_Parse(force_obj, &force) < 0) return NULL;
    b3_Vec3 point = body->position;
    if (point_obj != Py_None && PyVec3_Parse(point_obj, &point) < 0) return NULL;

    b3_rigidbody_apply_force(body, force, point);
    Py_RETURN_NONE;
}

static PyObject *rigidbody_apply_impulse(PyRigidBodyObject *self, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"impulse", "point", NULL};
    PyObject *impulse_obj;
    PyObject *point_obj = Py_None;
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "O|O", kwlist, &impulse_obj, &point_obj)) {
        return NULL;
    }
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return NULL;

    b3_Vec3 impulse;
    if (PyVec3_Parse(impulse_obj, &impulse) < 0) return NULL;
    b3_Vec3 point = body->position;
    if (point_obj != Py_None && PyVec3_Parse(point_obj, &point) < 0) return NULL;

    b3_rigidbody_apply_impulse(body, impulse, point);
    Py_RETURN_NONE;
}

static PyObject *rigidbody_clear_accumulators(PyRigidBodyObject *self, PyObject *Py_UNUSED(ignored)) {
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return NULL;
    b3_rigidbody_clear_accumulators(body);
    Py_RETURN_NONE;
}

static PyMethodDef rigidbody_methods[] = {
    {"apply_force", (PyCFunction)rigidbody_apply_force, METH_VARARGS | METH_KEYWORDS,
     "apply_force(force, point=None) -> None (point defaults to the body's position, i.e. no torque)"},
    {"apply_impulse", (PyCFunction)rigidbody_apply_impulse, METH_VARARGS | METH_KEYWORDS,
     "apply_impulse(impulse, point=None) -> None"},
    {"clear_accumulators", (PyCFunction)rigidbody_clear_accumulators, METH_NOARGS,
     "clear_accumulators() -> None"},
    {NULL},
};

PyTypeObject PyRigidBody_Type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "pybox3d.RigidBody",
    .tp_basicsize = sizeof(PyRigidBodyObject),
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = PyDoc_STR(
        "RigidBody(position, half_extents, mass=0.0)\n\n"
        "A box-shaped rigid body. mass=0 creates a static body. A handle "
        "returned by World.add_body()/World.get_body() is 'world-backed': "
        "it reads/writes the body stored inside that World rather than an "
        "independent copy."
    ),
    .tp_new = rigidbody_new,
    .tp_dealloc = (destructor)rigidbody_dealloc,
    .tp_repr = (reprfunc)rigidbody_repr,
    .tp_getset = rigidbody_getset,
    .tp_methods = rigidbody_methods,
};
