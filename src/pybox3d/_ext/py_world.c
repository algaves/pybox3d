#include "py_world.h"
#include "py_rigidbody.h"
#include "py_joint.h"
#include "py_worldsnapshot.h"
#include "py_vec3.h"
#include "py_quat.h"
#include "py_box3d.h"
#include "py_errors.h"
#include "box3d/debugdraw.h"

#include <float.h>
#include <stdlib.h>

static PyObject *world_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"gravity", "initial_capacity", "solver_iterations", NULL};
    PyObject *gravity_obj = NULL;
    int initial_capacity = 8;
    int solver_iterations = 4;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "|Oii", kwlist, &gravity_obj, &initial_capacity, &solver_iterations
        )) {
        return NULL;
    }
    if (solver_iterations < 1) {
        PyErr_SetString(PyExc_ValueError, "solver_iterations must be >= 1");
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
    self->world.solver_iterations = solver_iterations;
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

static PyObject *world_get_joint_count(PyWorldObject *self, void *closure) {
    (void)closure;
    return PyLong_FromLong(self->world.joint_count);
}

static PyObject *world_get_solver_iterations(PyWorldObject *self, void *closure) {
    (void)closure;
    return PyLong_FromLong(self->world.solver_iterations);
}
static int world_set_solver_iterations(PyWorldObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete solver_iterations");
        return -1;
    }
    long v = PyLong_AsLong(value);
    if (v == -1 && PyErr_Occurred()) return -1;
    if (v < 1) {
        PyErr_SetString(PyExc_ValueError, "solver_iterations must be >= 1");
        return -1;
    }
    self->world.solver_iterations = (int)v;
    return 0;
}

static PyObject *world_get_sleeping_enabled(PyWorldObject *self, void *closure) {
    (void)closure;
    return PyBool_FromLong(self->world.sleeping_enabled);
}
static int world_set_sleeping_enabled(PyWorldObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete sleeping_enabled");
        return -1;
    }
    int truth = PyObject_IsTrue(value);
    if (truth < 0) return -1;
    self->world.sleeping_enabled = truth;
    return 0;
}

static PyObject *world_get_sleep_linear_threshold(PyWorldObject *self, void *closure) {
    (void)closure;
    return PyFloat_FromDouble((double)self->world.sleep_linear_threshold);
}
static int world_set_sleep_linear_threshold(PyWorldObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete sleep_linear_threshold");
        return -1;
    }
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    if (v < 0.0) {
        PyErr_SetString(PyExc_ValueError, "sleep_linear_threshold must be >= 0");
        return -1;
    }
    self->world.sleep_linear_threshold = (b3_real)v;
    return 0;
}

static PyObject *world_get_sleep_angular_threshold(PyWorldObject *self, void *closure) {
    (void)closure;
    return PyFloat_FromDouble((double)self->world.sleep_angular_threshold);
}
static int world_set_sleep_angular_threshold(PyWorldObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete sleep_angular_threshold");
        return -1;
    }
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    if (v < 0.0) {
        PyErr_SetString(PyExc_ValueError, "sleep_angular_threshold must be >= 0");
        return -1;
    }
    self->world.sleep_angular_threshold = (b3_real)v;
    return 0;
}

static PyObject *world_get_sleep_time_threshold(PyWorldObject *self, void *closure) {
    (void)closure;
    return PyFloat_FromDouble((double)self->world.sleep_time_threshold);
}
static int world_set_sleep_time_threshold(PyWorldObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete sleep_time_threshold");
        return -1;
    }
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    if (v < 0.0) {
        PyErr_SetString(PyExc_ValueError, "sleep_time_threshold must be >= 0");
        return -1;
    }
    self->world.sleep_time_threshold = (b3_real)v;
    return 0;
}

static PyObject *world_get_contacts_began(PyWorldObject *self, void *closure) {
    (void)closure;
    int count = b3_world_contacts_began_count(&self->world);
    PyObject *list = PyList_New(count);
    if (list == NULL) return NULL;
    for (int i = 0; i < count; i++) {
        b3_ContactPairIds ids = b3_world_contacts_began(&self->world, i);
        PyObject *a = PyRigidBody_FromWorldId((PyObject *)self, ids.a);
        PyObject *b = a ? PyRigidBody_FromWorldId((PyObject *)self, ids.b) : NULL;
        PyObject *pair = b ? PyTuple_Pack(2, a, b) : NULL;
        Py_XDECREF(a);
        Py_XDECREF(b);
        if (pair == NULL) {
            Py_DECREF(list);
            return NULL;
        }
        PyList_SET_ITEM(list, i, pair);
    }
    return list;
}

static PyObject *world_get_contacts_ended(PyWorldObject *self, void *closure) {
    (void)closure;
    int count = b3_world_contacts_ended_count(&self->world);
    PyObject *list = PyList_New(count);
    if (list == NULL) return NULL;
    for (int i = 0; i < count; i++) {
        b3_ContactPairIds ids = b3_world_contacts_ended(&self->world, i);
        PyObject *a = PyRigidBody_FromWorldId((PyObject *)self, ids.a);
        PyObject *b = a ? PyRigidBody_FromWorldId((PyObject *)self, ids.b) : NULL;
        PyObject *pair = b ? PyTuple_Pack(2, a, b) : NULL;
        Py_XDECREF(a);
        Py_XDECREF(b);
        if (pair == NULL) {
            Py_DECREF(list);
            return NULL;
        }
        PyList_SET_ITEM(list, i, pair);
    }
    return list;
}

static PyGetSetDef world_getset[] = {
    {"gravity", (getter)world_get_gravity, (setter)world_set_gravity, "gravitational acceleration (Vec3)", NULL},
    {"body_count", (getter)world_get_body_count, NULL, "number of bodies currently in the world (read-only)", NULL},
    {"joint_count", (getter)world_get_joint_count, NULL, "number of joints currently in the world (read-only)", NULL},
    {"solver_iterations", (getter)world_get_solver_iterations, (setter)world_set_solver_iterations,
     "number of velocity-resolution passes per step over the same joint/contact set (>= 1, default 4)", NULL},
    {"sleeping_enabled", (getter)world_get_sleeping_enabled, (setter)world_set_sleeping_enabled,
     "whether World.step lets bodies fall asleep (default True)", NULL},
    {"sleep_linear_threshold", (getter)world_get_sleep_linear_threshold,
     (setter)world_set_sleep_linear_threshold,
     "linear speed (m/s) a dynamic body must stay below to accumulate sleep_timer (default 0.05)", NULL},
    {"sleep_angular_threshold", (getter)world_get_sleep_angular_threshold,
     (setter)world_set_sleep_angular_threshold,
     "angular speed (rad/s) a dynamic body must stay below to accumulate sleep_timer (default 0.05)", NULL},
    {"sleep_time_threshold", (getter)world_get_sleep_time_threshold, (setter)world_set_sleep_time_threshold,
     "seconds below both thresholds before a body falls asleep (default 0.5)", NULL},
    {"contacts_began", (getter)world_get_contacts_began, NULL,
     "list of (RigidBody, RigidBody) pairs that started overlapping on the last World.step() call "
     "(read-only)", NULL},
    {"contacts_ended", (getter)world_get_contacts_ended, NULL,
     "list of (RigidBody, RigidBody) pairs that stopped overlapping on the last World.step() call "
     "-- also fires when both bodies in a pair become static/asleep at the same time, even though "
     "they're still touching, since sleeping pairs are excluded from collision entirely (read-only)",
     NULL},
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
    return PyRigidBody_FromWorldId((PyObject *)self, b3_world_body_id(&self->world, index));
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
    return PyRigidBody_FromWorldId((PyObject *)self, b3_world_body_id(&self->world, (int)index));
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

/* Shared body validation for every World.add_*_joint() method: both
 * arguments must be RigidBody instances that are world-backed handles
 * already belonging to this World. On success, writes the resolved
 * PyRigidBodyObject/b3_RigidBody pointer pairs and returns 0; on
 * failure, sets a Python exception and returns -1. */
static int validate_joint_bodies(
    PyWorldObject *self, PyObject *body_a_obj, PyObject *body_b_obj, const char *func_name,
    PyRigidBodyObject **out_py_body_a, PyRigidBodyObject **out_py_body_b, b3_RigidBody **out_a,
    b3_RigidBody **out_b
) {
    if (!PyRigidBody_Check(body_a_obj) || !PyRigidBody_Check(body_b_obj)) {
        PyErr_Format(PyExc_TypeError, "%s() arguments must be RigidBody instances", func_name);
        return -1;
    }
    PyRigidBodyObject *body_a = (PyRigidBodyObject *)body_a_obj;
    PyRigidBodyObject *body_b = (PyRigidBodyObject *)body_b_obj;
    if (body_a->owns_storage || body_b->owns_storage || body_a->world != (PyObject *)self ||
        body_b->world != (PyObject *)self) {
        PyErr_Format(
            PyExc_ValueError,
            "%s() requires body_a and body_b to be world-backed handles already belonging to "
            "this World (e.g. returned by World.add_body())",
            func_name
        );
        return -1;
    }
    b3_RigidBody *a = PyRigidBody_Resolve(body_a);
    if (a == NULL) return -1;
    b3_RigidBody *b = PyRigidBody_Resolve(body_b);
    if (b == NULL) return -1;
    *out_py_body_a = body_a;
    *out_py_body_b = body_b;
    *out_a = a;
    *out_b = b;
    return 0;
}

static PyObject *world_add_joint(PyWorldObject *self, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {
        "body_a", "body_b", "rest_length", "anchor_a", "anchor_b",
        "min_length", "max_length", "stiffness", "damping", NULL
    };
    PyObject *body_a_obj, *body_b_obj;
    PyObject *rest_length_obj = Py_None;
    PyObject *anchor_a_obj = Py_None;
    PyObject *anchor_b_obj = Py_None;
    PyObject *min_length_obj = Py_None;
    PyObject *max_length_obj = Py_None;
    double stiffness = 0.0;
    double damping = 0.0;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "OO|OOOOOdd", kwlist, &body_a_obj, &body_b_obj, &rest_length_obj,
            &anchor_a_obj, &anchor_b_obj, &min_length_obj, &max_length_obj, &stiffness, &damping
        )) {
        return NULL;
    }
    PyRigidBodyObject *body_a, *body_b;
    b3_RigidBody *a, *b;
    if (validate_joint_bodies(self, body_a_obj, body_b_obj, "add_joint", &body_a, &body_b, &a, &b) < 0) {
        return NULL;
    }

    b3_Vec3 anchor_a = b3_vec3_make(0.0f, 0.0f, 0.0f);
    if (anchor_a_obj != Py_None && PyVec3_Parse(anchor_a_obj, &anchor_a) < 0) return NULL;
    b3_Vec3 anchor_b = b3_vec3_make(0.0f, 0.0f, 0.0f);
    if (anchor_b_obj != Py_None && PyVec3_Parse(anchor_b_obj, &anchor_b) < 0) return NULL;

    b3_real rest_length;
    if (rest_length_obj == Py_None) {
        b3_Vec3 world_anchor_a = b3_vec3_add(a->position, b3_quat_rotate_vec3(a->orientation, anchor_a));
        b3_Vec3 world_anchor_b = b3_vec3_add(b->position, b3_quat_rotate_vec3(b->orientation, anchor_b));
        rest_length = b3_vec3_length(b3_vec3_sub(world_anchor_b, world_anchor_a));
    } else {
        double v = PyFloat_AsDouble(rest_length_obj);
        if (v == -1.0 && PyErr_Occurred()) return NULL;
        if (v < 0.0) {
            PyErr_SetString(PyExc_ValueError, "rest_length must be >= 0");
            return NULL;
        }
        rest_length = (b3_real)v;
    }

    int has_limits = min_length_obj != Py_None || max_length_obj != Py_None;
    double min_length = 0.0;
    if (min_length_obj != Py_None) {
        min_length = PyFloat_AsDouble(min_length_obj);
        if (min_length == -1.0 && PyErr_Occurred()) return NULL;
        if (min_length < 0.0) {
            PyErr_SetString(PyExc_ValueError, "min_length must be >= 0");
            return NULL;
        }
    }
    double max_length = (double)rest_length;
    if (max_length_obj != Py_None) {
        max_length = PyFloat_AsDouble(max_length_obj);
        if (max_length == -1.0 && PyErr_Occurred()) return NULL;
    } else if (min_length_obj != Py_None) {
        max_length = min_length > (double)rest_length ? min_length : (double)rest_length;
    }
    if (has_limits && max_length < min_length) {
        PyErr_SetString(PyExc_ValueError, "max_length must be >= min_length");
        return NULL;
    }
    if (stiffness < 0.0) {
        PyErr_SetString(PyExc_ValueError, "stiffness must be >= 0");
        return NULL;
    }
    if (damping < 0.0) {
        PyErr_SetString(PyExc_ValueError, "damping must be >= 0");
        return NULL;
    }

    int index;
    b3_Status status = b3_world_add_joint(
        &self->world, body_a->world_id, body_b->world_id, rest_length, &index
    );
    if (pybox3d_status_to_exception(status, "World.add_joint") < 0) {
        return NULL;
    }
    b3_Joint *joint = b3_world_get_joint(&self->world, index);
    joint->anchor_a = anchor_a;
    joint->anchor_b = anchor_b;
    joint->params.distance.has_limits = has_limits;
    joint->params.distance.min_length = (b3_real)min_length;
    joint->params.distance.max_length = (b3_real)max_length;
    joint->params.distance.stiffness = (b3_real)stiffness;
    joint->params.distance.damping = (b3_real)damping;
    return PyDistanceJoint_FromWorldId((PyObject *)self, b3_world_joint_id(&self->world, index));
}

/* Parses anchor_a/anchor_b keyword args (both default the origin = each
 * body's center) into out_anchor_a/out_anchor_b. Shared by every
 * add_*_joint() method below that takes them. */
static int parse_anchor_kwargs(
    PyObject *anchor_a_obj, PyObject *anchor_b_obj, b3_Vec3 *out_anchor_a, b3_Vec3 *out_anchor_b
) {
    *out_anchor_a = b3_vec3_make(0.0f, 0.0f, 0.0f);
    if (anchor_a_obj != NULL && anchor_a_obj != Py_None && PyVec3_Parse(anchor_a_obj, out_anchor_a) < 0) {
        return -1;
    }
    *out_anchor_b = b3_vec3_make(0.0f, 0.0f, 0.0f);
    if (anchor_b_obj != NULL && anchor_b_obj != Py_None && PyVec3_Parse(anchor_b_obj, out_anchor_b) < 0) {
        return -1;
    }
    return 0;
}

static PyObject *world_add_spherical_joint(PyWorldObject *self, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"body_a", "body_b", "anchor_a", "anchor_b", NULL};
    PyObject *body_a_obj, *body_b_obj;
    PyObject *anchor_a_obj = Py_None;
    PyObject *anchor_b_obj = Py_None;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "OO|OO", kwlist, &body_a_obj, &body_b_obj, &anchor_a_obj, &anchor_b_obj
        )) {
        return NULL;
    }
    PyRigidBodyObject *body_a, *body_b;
    b3_RigidBody *a, *b;
    if (validate_joint_bodies(self, body_a_obj, body_b_obj, "add_spherical_joint", &body_a, &body_b, &a, &b) < 0) {
        return NULL;
    }
    b3_Vec3 anchor_a, anchor_b;
    if (parse_anchor_kwargs(anchor_a_obj, anchor_b_obj, &anchor_a, &anchor_b) < 0) {
        return NULL;
    }

    int index;
    b3_Status status = b3_world_add_spherical_joint(&self->world, body_a->world_id, body_b->world_id, &index);
    if (pybox3d_status_to_exception(status, "World.add_spherical_joint") < 0) {
        return NULL;
    }
    b3_Joint *joint = b3_world_get_joint(&self->world, index);
    joint->anchor_a = anchor_a;
    joint->anchor_b = anchor_b;
    return PyJoint_FromWorldId((PyObject *)self, b3_world_joint_id(&self->world, index));
}

static PyObject *world_add_revolute_joint(PyWorldObject *self, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {
        "body_a", "body_b", "axis", "anchor_a", "anchor_b", "enable_motor", "motor_speed",
        "max_motor_effort", NULL
    };
    PyObject *body_a_obj, *body_b_obj, *axis_obj;
    PyObject *anchor_a_obj = Py_None;
    PyObject *anchor_b_obj = Py_None;
    int enable_motor = 0;
    double motor_speed = 0.0;
    double max_motor_effort = 0.0;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "OOO|OOpdd", kwlist, &body_a_obj, &body_b_obj, &axis_obj, &anchor_a_obj,
            &anchor_b_obj, &enable_motor, &motor_speed, &max_motor_effort
        )) {
        return NULL;
    }
    PyRigidBodyObject *body_a, *body_b;
    b3_RigidBody *a, *b;
    if (validate_joint_bodies(self, body_a_obj, body_b_obj, "add_revolute_joint", &body_a, &body_b, &a, &b) < 0) {
        return NULL;
    }
    b3_Vec3 axis;
    if (PyVec3_Parse(axis_obj, &axis) < 0) return NULL;
    b3_Vec3 anchor_a, anchor_b;
    if (parse_anchor_kwargs(anchor_a_obj, anchor_b_obj, &anchor_a, &anchor_b) < 0) {
        return NULL;
    }
    if (max_motor_effort < 0.0) {
        PyErr_SetString(PyExc_ValueError, "max_motor_effort must be >= 0");
        return NULL;
    }

    int index;
    b3_Status status = b3_world_add_revolute_joint(&self->world, body_a->world_id, body_b->world_id, &index);
    if (pybox3d_status_to_exception(status, "World.add_revolute_joint") < 0) {
        return NULL;
    }
    b3_Joint *joint = b3_world_get_joint(&self->world, index);
    joint->anchor_a = anchor_a;
    joint->anchor_b = anchor_b;
    joint->params.revolute.axis_a = axis;
    joint->params.revolute.enable_motor = enable_motor;
    joint->params.revolute.motor_speed = (b3_real)motor_speed;
    joint->params.revolute.max_motor_torque = (b3_real)max_motor_effort;
    return PyJoint_FromWorldId((PyObject *)self, b3_world_joint_id(&self->world, index));
}

static PyObject *world_add_prismatic_joint(PyWorldObject *self, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {
        "body_a", "body_b", "axis", "anchor_a", "anchor_b", "min_translation", "max_translation",
        "enable_motor", "motor_speed", "max_motor_effort", NULL
    };
    PyObject *body_a_obj, *body_b_obj, *axis_obj;
    PyObject *anchor_a_obj = Py_None;
    PyObject *anchor_b_obj = Py_None;
    PyObject *min_translation_obj = Py_None;
    PyObject *max_translation_obj = Py_None;
    int enable_motor = 0;
    double motor_speed = 0.0;
    double max_motor_effort = 0.0;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "OOO|OOOOpdd", kwlist, &body_a_obj, &body_b_obj, &axis_obj, &anchor_a_obj,
            &anchor_b_obj, &min_translation_obj, &max_translation_obj, &enable_motor, &motor_speed,
            &max_motor_effort
        )) {
        return NULL;
    }
    PyRigidBodyObject *body_a, *body_b;
    b3_RigidBody *a, *b;
    if (validate_joint_bodies(self, body_a_obj, body_b_obj, "add_prismatic_joint", &body_a, &body_b, &a, &b) < 0) {
        return NULL;
    }
    b3_Vec3 axis;
    if (PyVec3_Parse(axis_obj, &axis) < 0) return NULL;
    b3_Vec3 anchor_a, anchor_b;
    if (parse_anchor_kwargs(anchor_a_obj, anchor_b_obj, &anchor_a, &anchor_b) < 0) {
        return NULL;
    }
    int has_limits = min_translation_obj != Py_None || max_translation_obj != Py_None;
    double min_translation = 0.0;
    if (min_translation_obj != Py_None) {
        min_translation = PyFloat_AsDouble(min_translation_obj);
        if (min_translation == -1.0 && PyErr_Occurred()) return NULL;
    }
    double max_translation = 0.0;
    if (max_translation_obj != Py_None) {
        max_translation = PyFloat_AsDouble(max_translation_obj);
        if (max_translation == -1.0 && PyErr_Occurred()) return NULL;
    }
    if (has_limits && max_translation < min_translation) {
        PyErr_SetString(PyExc_ValueError, "max_translation must be >= min_translation");
        return NULL;
    }
    if (max_motor_effort < 0.0) {
        PyErr_SetString(PyExc_ValueError, "max_motor_effort must be >= 0");
        return NULL;
    }

    int index;
    b3_Status status = b3_world_add_prismatic_joint(&self->world, body_a->world_id, body_b->world_id, &index);
    if (pybox3d_status_to_exception(status, "World.add_prismatic_joint") < 0) {
        return NULL;
    }
    b3_Joint *joint = b3_world_get_joint(&self->world, index);
    joint->anchor_a = anchor_a;
    joint->anchor_b = anchor_b;
    joint->params.prismatic.axis_a = axis;
    joint->params.prismatic.has_limits = has_limits;
    joint->params.prismatic.min_translation = (b3_real)min_translation;
    joint->params.prismatic.max_translation = (b3_real)max_translation;
    joint->params.prismatic.enable_motor = enable_motor;
    joint->params.prismatic.motor_speed = (b3_real)motor_speed;
    joint->params.prismatic.max_motor_force = (b3_real)max_motor_effort;
    return PyJoint_FromWorldId((PyObject *)self, b3_world_joint_id(&self->world, index));
}

static PyObject *world_add_weld_joint(PyWorldObject *self, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"body_a", "body_b", "anchor_a", "anchor_b", NULL};
    PyObject *body_a_obj, *body_b_obj;
    PyObject *anchor_a_obj = Py_None;
    PyObject *anchor_b_obj = Py_None;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "OO|OO", kwlist, &body_a_obj, &body_b_obj, &anchor_a_obj, &anchor_b_obj
        )) {
        return NULL;
    }
    PyRigidBodyObject *body_a, *body_b;
    b3_RigidBody *a, *b;
    if (validate_joint_bodies(self, body_a_obj, body_b_obj, "add_weld_joint", &body_a, &body_b, &a, &b) < 0) {
        return NULL;
    }
    b3_Vec3 anchor_a, anchor_b;
    if (parse_anchor_kwargs(anchor_a_obj, anchor_b_obj, &anchor_a, &anchor_b) < 0) {
        return NULL;
    }

    int index;
    b3_Status status = b3_world_add_weld_joint(&self->world, body_a->world_id, body_b->world_id, &index);
    if (pybox3d_status_to_exception(status, "World.add_weld_joint") < 0) {
        return NULL;
    }
    b3_Joint *joint = b3_world_get_joint(&self->world, index);
    joint->anchor_a = anchor_a;
    joint->anchor_b = anchor_b;
    return PyJoint_FromWorldId((PyObject *)self, b3_world_joint_id(&self->world, index));
}

static PyObject *world_add_motor_joint(PyWorldObject *self, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {
        "body_a", "body_b", "linear_offset", "angular_offset", "linear_stiffness",
        "linear_damping", "angular_stiffness", "angular_damping", NULL
    };
    PyObject *body_a_obj, *body_b_obj;
    PyObject *linear_offset_obj = Py_None;
    PyObject *angular_offset_obj = Py_None;
    double linear_stiffness = 0.0;
    double linear_damping = 0.0;
    double angular_stiffness = 0.0;
    double angular_damping = 0.0;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "OO|OOdddd", kwlist, &body_a_obj, &body_b_obj, &linear_offset_obj,
            &angular_offset_obj, &linear_stiffness, &linear_damping, &angular_stiffness,
            &angular_damping
        )) {
        return NULL;
    }
    PyRigidBodyObject *body_a, *body_b;
    b3_RigidBody *a, *b;
    if (validate_joint_bodies(self, body_a_obj, body_b_obj, "add_motor_joint", &body_a, &body_b, &a, &b) < 0) {
        return NULL;
    }
    b3_Vec3 linear_offset = b3_vec3_make(0.0f, 0.0f, 0.0f);
    if (linear_offset_obj != Py_None && PyVec3_Parse(linear_offset_obj, &linear_offset) < 0) return NULL;
    b3_Quat angular_offset = b3_quat_identity();
    if (angular_offset_obj != Py_None && PyQuat_Parse(angular_offset_obj, &angular_offset) < 0) return NULL;
    if (linear_stiffness < 0.0 || linear_damping < 0.0 || angular_stiffness < 0.0 || angular_damping < 0.0) {
        PyErr_SetString(PyExc_ValueError, "stiffness/damping values must be >= 0");
        return NULL;
    }

    int index;
    b3_Status status = b3_world_add_motor_joint(&self->world, body_a->world_id, body_b->world_id, &index);
    if (pybox3d_status_to_exception(status, "World.add_motor_joint") < 0) {
        return NULL;
    }
    b3_Joint *joint = b3_world_get_joint(&self->world, index);
    joint->params.motor.linear_offset = linear_offset;
    joint->params.motor.angular_offset = angular_offset;
    joint->params.motor.linear_stiffness = (b3_real)linear_stiffness;
    joint->params.motor.linear_damping = (b3_real)linear_damping;
    joint->params.motor.angular_stiffness = (b3_real)angular_stiffness;
    joint->params.motor.angular_damping = (b3_real)angular_damping;
    return PyJoint_FromWorldId((PyObject *)self, b3_world_joint_id(&self->world, index));
}

static PyObject *world_add_wheel_joint(PyWorldObject *self, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {
        "body_a", "body_b", "suspension_axis", "axle_axis", "anchor_a", "anchor_b",
        "min_translation", "max_translation", "suspension_stiffness", "suspension_damping", NULL
    };
    PyObject *body_a_obj, *body_b_obj, *suspension_axis_obj, *axle_axis_obj;
    PyObject *anchor_a_obj = Py_None;
    PyObject *anchor_b_obj = Py_None;
    PyObject *min_translation_obj = Py_None;
    PyObject *max_translation_obj = Py_None;
    double suspension_stiffness = 0.0;
    double suspension_damping = 0.0;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "OOOO|OOOOdd", kwlist, &body_a_obj, &body_b_obj, &suspension_axis_obj,
            &axle_axis_obj, &anchor_a_obj, &anchor_b_obj, &min_translation_obj, &max_translation_obj,
            &suspension_stiffness, &suspension_damping
        )) {
        return NULL;
    }
    PyRigidBodyObject *body_a, *body_b;
    b3_RigidBody *a, *b;
    if (validate_joint_bodies(self, body_a_obj, body_b_obj, "add_wheel_joint", &body_a, &body_b, &a, &b) < 0) {
        return NULL;
    }
    b3_Vec3 suspension_axis;
    if (PyVec3_Parse(suspension_axis_obj, &suspension_axis) < 0) return NULL;
    b3_Vec3 axle_axis;
    if (PyVec3_Parse(axle_axis_obj, &axle_axis) < 0) return NULL;
    b3_Vec3 anchor_a, anchor_b;
    if (parse_anchor_kwargs(anchor_a_obj, anchor_b_obj, &anchor_a, &anchor_b) < 0) {
        return NULL;
    }
    int has_limits = min_translation_obj != Py_None || max_translation_obj != Py_None;
    double min_translation = 0.0;
    if (min_translation_obj != Py_None) {
        min_translation = PyFloat_AsDouble(min_translation_obj);
        if (min_translation == -1.0 && PyErr_Occurred()) return NULL;
    }
    double max_translation = 0.0;
    if (max_translation_obj != Py_None) {
        max_translation = PyFloat_AsDouble(max_translation_obj);
        if (max_translation == -1.0 && PyErr_Occurred()) return NULL;
    }
    if (has_limits && max_translation < min_translation) {
        PyErr_SetString(PyExc_ValueError, "max_translation must be >= min_translation");
        return NULL;
    }
    if (suspension_stiffness < 0.0 || suspension_damping < 0.0) {
        PyErr_SetString(PyExc_ValueError, "suspension_stiffness/suspension_damping must be >= 0");
        return NULL;
    }

    int index;
    b3_Status status = b3_world_add_wheel_joint(&self->world, body_a->world_id, body_b->world_id, &index);
    if (pybox3d_status_to_exception(status, "World.add_wheel_joint") < 0) {
        return NULL;
    }
    b3_Joint *joint = b3_world_get_joint(&self->world, index);
    joint->anchor_a = anchor_a;
    joint->anchor_b = anchor_b;
    joint->params.wheel.suspension_axis_a = suspension_axis;
    joint->params.wheel.axle_axis_a = axle_axis;
    joint->params.wheel.has_limits = has_limits;
    joint->params.wheel.min_translation = (b3_real)min_translation;
    joint->params.wheel.max_translation = (b3_real)max_translation;
    joint->params.wheel.suspension_stiffness = (b3_real)suspension_stiffness;
    joint->params.wheel.suspension_damping = (b3_real)suspension_damping;
    return PyJoint_FromWorldId((PyObject *)self, b3_world_joint_id(&self->world, index));
}

static PyObject *world_add_filter_joint(PyWorldObject *self, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"body_a", "body_b", NULL};
    PyObject *body_a_obj, *body_b_obj;
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "OO", kwlist, &body_a_obj, &body_b_obj)) {
        return NULL;
    }
    PyRigidBodyObject *body_a, *body_b;
    b3_RigidBody *a, *b;
    if (validate_joint_bodies(self, body_a_obj, body_b_obj, "add_filter_joint", &body_a, &body_b, &a, &b) < 0) {
        return NULL;
    }

    int index;
    b3_Status status = b3_world_add_filter_joint(&self->world, body_a->world_id, body_b->world_id, &index);
    if (pybox3d_status_to_exception(status, "World.add_filter_joint") < 0) {
        return NULL;
    }
    return PyJoint_FromWorldId((PyObject *)self, b3_world_joint_id(&self->world, index));
}

static PyObject *world_add_parallel_joint(PyWorldObject *self, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"body_a", "body_b", NULL};
    PyObject *body_a_obj, *body_b_obj;
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "OO", kwlist, &body_a_obj, &body_b_obj)) {
        return NULL;
    }
    PyRigidBodyObject *body_a, *body_b;
    b3_RigidBody *a, *b;
    if (validate_joint_bodies(self, body_a_obj, body_b_obj, "add_parallel_joint", &body_a, &body_b, &a, &b) < 0) {
        return NULL;
    }

    int index;
    b3_Status status = b3_world_add_parallel_joint(&self->world, body_a->world_id, body_b->world_id, &index);
    if (pybox3d_status_to_exception(status, "World.add_parallel_joint") < 0) {
        return NULL;
    }
    return PyJoint_FromWorldId((PyObject *)self, b3_world_joint_id(&self->world, index));
}

static PyObject *world_get_joint(PyWorldObject *self, PyObject *arg) {
    long index = PyLong_AsLong(arg);
    if (index == -1 && PyErr_Occurred()) {
        return NULL;
    }
    b3_Joint *joint = b3_world_get_joint(&self->world, (int)index);
    if (joint == NULL) {
        PyErr_Format(PyExc_IndexError, "World.get_joint: index %ld out of range", index);
        return NULL;
    }
    b3_JointId id = b3_world_joint_id(&self->world, (int)index);
    if (joint->kind == B3_JOINT_DISTANCE) {
        return PyDistanceJoint_FromWorldId((PyObject *)self, id);
    }
    return PyJoint_FromWorldId((PyObject *)self, id);
}

static PyObject *world_remove_joint(PyWorldObject *self, PyObject *arg) {
    long index = PyLong_AsLong(arg);
    if (index == -1 && PyErr_Occurred()) {
        return NULL;
    }
    b3_Status status = b3_world_remove_joint(&self->world, (int)index);
    if (pybox3d_status_to_exception(status, "World.remove_joint") < 0) {
        return NULL;
    }
    Py_RETURN_NONE;
}

static PyObject *world_query_aabb(PyWorldObject *self, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"min", "max", NULL};
    PyObject *min_obj, *max_obj;
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "OO", kwlist, &min_obj, &max_obj)) {
        return NULL;
    }
    b3_Vec3 min, max;
    if (PyVec3_Parse(min_obj, &min) < 0) return NULL;
    if (PyVec3_Parse(max_obj, &max) < 0) return NULL;

    int capacity = self->world.body_count;
    b3_BodyId *ids = capacity > 0 ? (b3_BodyId *)malloc((size_t)capacity * sizeof(b3_BodyId)) : NULL;
    if (capacity > 0 && ids == NULL) {
        return PyErr_NoMemory();
    }
    /* `capacity` is body_count, the maximum any query could ever match,
     * so `matched` here can never exceed it -- no truncation to worry
     * about. */
    int matched = b3_world_query_aabb(&self->world, min, max, ids, capacity);
    PyObject *list = PyList_New(matched);
    if (list == NULL) {
        free(ids);
        return NULL;
    }
    for (int i = 0; i < matched; i++) {
        PyObject *body = PyRigidBody_FromWorldId((PyObject *)self, ids[i]);
        if (body == NULL) {
            free(ids);
            Py_DECREF(list);
            return NULL;
        }
        PyList_SET_ITEM(list, i, body);
    }
    free(ids);
    return list;
}

static PyObject *build_world_rayhit(b3_RayHit hit) {
    PyObject *result = PyStructSequence_New(PyRayHit_Type);
    if (result == NULL) {
        return NULL;
    }
    PyObject *t = PyFloat_FromDouble((double)hit.t);
    PyObject *point = t ? PyVec3_FromVec3(hit.point) : NULL;
    PyObject *normal = point ? PyVec3_FromVec3(hit.normal) : NULL;
    if (t == NULL || point == NULL || normal == NULL) {
        Py_XDECREF(t);
        Py_XDECREF(point);
        Py_XDECREF(normal);
        Py_DECREF(result);
        return NULL;
    }
    PyStructSequence_SET_ITEM(result, 0, t);
    PyStructSequence_SET_ITEM(result, 1, point);
    PyStructSequence_SET_ITEM(result, 2, normal);
    return result;
}

static PyObject *world_raycast_all(PyWorldObject *self, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"origin", "direction", "max_distance", NULL};
    PyObject *origin_obj, *direction_obj;
    double max_distance = DBL_MAX;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "OO|d", kwlist, &origin_obj, &direction_obj, &max_distance
        )) {
        return NULL;
    }
    b3_Vec3 origin, direction;
    if (PyVec3_Parse(origin_obj, &origin) < 0) return NULL;
    if (PyVec3_Parse(direction_obj, &direction) < 0) return NULL;

    int capacity = self->world.body_count;
    b3_WorldRayHit *hits =
        capacity > 0 ? (b3_WorldRayHit *)malloc((size_t)capacity * sizeof(b3_WorldRayHit)) : NULL;
    if (capacity > 0 && hits == NULL) {
        return PyErr_NoMemory();
    }
    int matched = b3_world_raycast_all(&self->world, origin, direction, (b3_real)max_distance, hits, capacity);
    PyObject *list = PyList_New(matched);
    if (list == NULL) {
        free(hits);
        return NULL;
    }
    for (int i = 0; i < matched; i++) {
        PyObject *body = PyRigidBody_FromWorldId((PyObject *)self, hits[i].body_id);
        PyObject *rayhit = body != NULL ? build_world_rayhit(hits[i].hit) : NULL;
        PyObject *pair = rayhit != NULL ? PyTuple_Pack(2, body, rayhit) : NULL;
        Py_XDECREF(body);
        Py_XDECREF(rayhit);
        if (pair == NULL) {
            free(hits);
            Py_DECREF(list);
            return NULL;
        }
        PyList_SET_ITEM(list, i, pair);
    }
    free(hits);
    return list;
}

static PyObject *world_snapshot(PyWorldObject *self, PyObject *Py_UNUSED(ignored)) {
    int count = self->world.body_count;
    b3_BodySnapshot *buffer = count > 0 ? (b3_BodySnapshot *)malloc((size_t)count * sizeof(b3_BodySnapshot)) : NULL;
    if (count > 0 && buffer == NULL) {
        return PyErr_NoMemory();
    }
    int out_count = 0;
    b3_Status status = b3_world_snapshot(&self->world, buffer, count, &out_count);
    if (pybox3d_status_to_exception(status, "World.snapshot") < 0) {
        free(buffer);
        return NULL;
    }
    PyObject *result = PyWorldSnapshot_New(buffer, out_count);
    free(buffer);
    return result;
}

static PyObject *world_restore(PyWorldObject *self, PyObject *arg) {
    if (!PyWorldSnapshot_Check(arg)) {
        PyErr_SetString(PyExc_TypeError, "restore() argument must be a WorldSnapshot");
        return NULL;
    }
    PyWorldSnapshotObject *snapshot = (PyWorldSnapshotObject *)arg;
    b3_world_restore(&self->world, snapshot->snapshots, snapshot->count);
    Py_RETURN_NONE;
}

static PyObject *world_debug_contacts(PyWorldObject *self, PyObject *Py_UNUSED(ignored)) {
    int count = b3_world_debug_contacts(&self->world, NULL, 0);
    b3_DebugContact *contacts =
        count > 0 ? (b3_DebugContact *)malloc((size_t)count * sizeof(b3_DebugContact)) : NULL;
    if (count > 0 && contacts == NULL) {
        return PyErr_NoMemory();
    }
    b3_world_debug_contacts(&self->world, contacts, count);

    PyObject *list = PyList_New(count);
    if (list == NULL) {
        free(contacts);
        return NULL;
    }
    for (int i = 0; i < count; i++) {
        PyObject *point = PyVec3_FromVec3(contacts[i].point);
        PyObject *normal = point != NULL ? PyVec3_FromVec3(contacts[i].normal) : NULL;
        PyObject *pair = normal != NULL ? PyTuple_Pack(2, point, normal) : NULL;
        Py_XDECREF(point);
        Py_XDECREF(normal);
        if (pair == NULL) {
            free(contacts);
            Py_DECREF(list);
            return NULL;
        }
        PyList_SET_ITEM(list, i, pair);
    }
    free(contacts);
    return list;
}

static PyObject *world_debug_joint_anchors(PyWorldObject *self, PyObject *Py_UNUSED(ignored)) {
    int count = b3_world_debug_joint_anchors(&self->world, NULL, 0);
    b3_DebugJointAnchor *anchors =
        count > 0 ? (b3_DebugJointAnchor *)malloc((size_t)count * sizeof(b3_DebugJointAnchor)) : NULL;
    if (count > 0 && anchors == NULL) {
        return PyErr_NoMemory();
    }
    b3_world_debug_joint_anchors(&self->world, anchors, count);

    PyObject *list = PyList_New(count);
    if (list == NULL) {
        free(anchors);
        return NULL;
    }
    for (int i = 0; i < count; i++) {
        PyObject *a = PyVec3_FromVec3(anchors[i].anchor_a);
        PyObject *b = a != NULL ? PyVec3_FromVec3(anchors[i].anchor_b) : NULL;
        PyObject *pair = b != NULL ? PyTuple_Pack(2, a, b) : NULL;
        Py_XDECREF(a);
        Py_XDECREF(b);
        if (pair == NULL) {
            free(anchors);
            Py_DECREF(list);
            return NULL;
        }
        PyList_SET_ITEM(list, i, pair);
    }
    free(anchors);
    return list;
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
    {"add_joint", (PyCFunction)world_add_joint, METH_VARARGS | METH_KEYWORDS,
     "add_joint(body_a: RigidBody, body_b: RigidBody, rest_length: float | None = None, "
     "anchor_a: VecLike = (0, 0, 0), anchor_b: VecLike = (0, 0, 0), "
     "min_length: float | None = None, max_length: float | None = None, "
     "stiffness: float = 0.0, damping: float = 0.0) -> DistanceJoint "
     "(body_a/body_b must already be world-backed handles in this World; rest_length "
     "defaults to the current distance between the two world-space anchors; "
     "min_length/max_length are only enforced if either is given; stiffness > 0 makes "
     "this a spring toward rest_length instead of a rigid constraint)"},
    {"add_spherical_joint", (PyCFunction)world_add_spherical_joint, METH_VARARGS | METH_KEYWORDS,
     "add_spherical_joint(body_a: RigidBody, body_b: RigidBody, anchor_a: VecLike = (0, 0, 0), "
     "anchor_b: VecLike = (0, 0, 0)) -> Joint (a ball-and-socket point constraint: the two "
     "world-space anchors are held coincident, rotation is free)"},
    {"add_revolute_joint", (PyCFunction)world_add_revolute_joint, METH_VARARGS | METH_KEYWORDS,
     "add_revolute_joint(body_a: RigidBody, body_b: RigidBody, axis: VecLike, "
     "anchor_a: VecLike = (0, 0, 0), anchor_b: VecLike = (0, 0, 0), enable_motor: bool = False, "
     "motor_speed: float = 0.0, max_motor_effort: float = 0.0) -> Joint (a hinge: point "
     "constraint plus a lock on every rotation axis except `axis`, body_a-local; an optional "
     "motor drives relative angular velocity around `axis` toward motor_speed, clamped by "
     "max_motor_effort as a per-step torque limit)"},
    {"add_prismatic_joint", (PyCFunction)world_add_prismatic_joint, METH_VARARGS | METH_KEYWORDS,
     "add_prismatic_joint(body_a: RigidBody, body_b: RigidBody, axis: VecLike, "
     "anchor_a: VecLike = (0, 0, 0), anchor_b: VecLike = (0, 0, 0), "
     "min_translation: float | None = None, max_translation: float | None = None, "
     "enable_motor: bool = False, motor_speed: float = 0.0, max_motor_effort: float = 0.0) "
     "-> Joint (a slider: locks every DOF except translation along `axis` (body_a-local), "
     "optionally hard-capped at min_translation/max_translation; an optional motor drives "
     "relative linear velocity along `axis` toward motor_speed, clamped by max_motor_effort "
     "as a per-step force limit)"},
    {"add_weld_joint", (PyCFunction)world_add_weld_joint, METH_VARARGS | METH_KEYWORDS,
     "add_weld_joint(body_a: RigidBody, body_b: RigidBody, anchor_a: VecLike = (0, 0, 0), "
     "anchor_b: VecLike = (0, 0, 0)) -> Joint (full 6-DOF lock: point constraint plus a lock "
     "on every rotation axis)"},
    {"add_motor_joint", (PyCFunction)world_add_motor_joint, METH_VARARGS | METH_KEYWORDS,
     "add_motor_joint(body_a: RigidBody, body_b: RigidBody, linear_offset: VecLike = (0, 0, 0), "
     "angular_offset: QuatLike = (0, 0, 0, 1), linear_stiffness: float = 0.0, "
     "linear_damping: float = 0.0, angular_stiffness: float = 0.0, angular_damping: float = 0.0) "
     "-> Joint (soft, not a hard constraint: independently springs body_b's position toward "
     "body_a.position + rotate(body_a.orientation, linear_offset), and body_b's orientation "
     "toward body_a.orientation * angular_offset -- each spring only active while its "
     "stiffness > 0)"},
    {"add_wheel_joint", (PyCFunction)world_add_wheel_joint, METH_VARARGS | METH_KEYWORDS,
     "add_wheel_joint(body_a: RigidBody, body_b: RigidBody, suspension_axis: VecLike, "
     "axle_axis: VecLike, anchor_a: VecLike = (0, 0, 0), anchor_b: VecLike = (0, 0, 0), "
     "min_translation: float | None = None, max_translation: float | None = None, "
     "suspension_stiffness: float = 0.0, suspension_damping: float = 0.0) -> Joint "
     "(a Prismatic-style suspension_axis translation freedom, both body-A-local, combined with "
     "free rotation around axle_axis; suspension_axis translation is hard-capped if "
     "min_translation/max_translation are given, otherwise springs toward zero offset if "
     "suspension_stiffness > 0, otherwise free-slides)"},
    {"add_filter_joint", (PyCFunction)world_add_filter_joint, METH_VARARGS | METH_KEYWORDS,
     "add_filter_joint(body_a: RigidBody, body_b: RigidBody) -> Joint (not a real constraint: "
     "makes World.step skip collision detection/resolution between body_a and body_b entirely)"},
    {"add_parallel_joint", (PyCFunction)world_add_parallel_joint, METH_VARARGS | METH_KEYWORDS,
     "add_parallel_joint(body_a: RigidBody, body_b: RigidBody) -> Joint (angular-only Weld: "
     "locks relative orientation, translation is completely free)"},
    {"get_joint", (PyCFunction)world_get_joint, METH_O,
     "get_joint(index: int) -> DistanceJoint | Joint (a fresh world-backed handle each call; "
     "the type depends on what kind of joint is at `index`)"},
    {"remove_joint", (PyCFunction)world_remove_joint, METH_O,
     "remove_joint(index: int) -> None (swap-remove; see DistanceJoint/Joint docs on stale handles)"},
    {"query_aabb", (PyCFunction)world_query_aabb, METH_VARARGS | METH_KEYWORDS,
     "query_aabb(min: VecLike, max: VecLike) -> list[RigidBody] (every body whose AABB overlaps "
     "the given box)"},
    {"raycast_all", (PyCFunction)world_raycast_all, METH_VARARGS | METH_KEYWORDS,
     "raycast_all(origin: VecLike, direction: VecLike, max_distance: float = inf) -> "
     "list[tuple[RigidBody, RayHit]] (every body the ray hits, unsorted; max_distance is in "
     "units of direction's own length, matching RigidBody.shape.raycast())"},
    {"snapshot", (PyCFunction)world_snapshot, METH_NOARGS,
     "snapshot() -> WorldSnapshot (records every body's current transform/velocity; pair with "
     "restore() for simple state recording/replay)"},
    {"restore", (PyCFunction)world_restore, METH_O,
     "restore(snapshot: WorldSnapshot) -> None (resets every body named in `snapshot` to its "
     "recorded state and wakes it; bodies removed since the snapshot are silently skipped, "
     "bodies added since are left untouched)"},
    {"debug_contacts", (PyCFunction)world_debug_contacts, METH_NOARGS,
     "debug_contacts() -> list[tuple[Vec3, Vec3]] (point, normal for every currently-overlapping "
     "body pair, from a fresh scan independent of the last step() -- pure data, no rendering; "
     "does not respect Filter joints, unlike step() itself)"},
    {"debug_joint_anchors", (PyCFunction)world_debug_joint_anchors, METH_NOARGS,
     "debug_joint_anchors() -> list[tuple[Vec3, Vec3]] (world-space anchor_a, anchor_b for every "
     "joint, in the same order as get_joint(0..joint_count-1); Filter/Motor/Parallel joints "
     "report each body's own center, since their anchors are unused)"},
    {"step", (PyCFunction)world_step, METH_O, "step(dt: float) -> None"},
    {NULL},
};

PyTypeObject PyWorld_Type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "pybox3d.World",
    .tp_basicsize = sizeof(PyWorldObject),
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = PyDoc_STR(
        "World(gravity=(0, -9.81, 0), initial_capacity=8, solver_iterations=4)\n\n"
        "A rigid-body simulation world: naive O(n^2) collision detection "
        "and full 6-DOF impulse-based resolution (using each pair's own "
        "RigidBody.restitution/friction, combined via standard mixing "
        "rules), plus distance joints (see add_joint()). solver_iterations "
        "velocity-resolution passes run per step over the same detected "
        "joint/contact set."
    ),
    .tp_new = world_new,
    .tp_dealloc = (destructor)world_dealloc,
    .tp_repr = (reprfunc)world_repr,
    .tp_getset = world_getset,
    .tp_methods = world_methods,
};
