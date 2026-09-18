#include "py_rigidbody.h"
#include "py_world.h"
#include "py_vec3.h"
#include "py_quat.h"
#include "py_box3d.h"
#include "py_shape.h"
#include "box3d/debugdraw.h"

#include <stdlib.h>
#include "py_hull.h"
#include "py_compound.h"
#include "py_mesh.h"
#include "py_heightfield.h"

PyObject *PyRigidBody_FromOwned(const b3_RigidBody *body) {
    PyRigidBodyObject *self = PyObject_New(PyRigidBodyObject, &PyRigidBody_Type);
    if (self == NULL) {
        return NULL;
    }
    self->owned = *body;
    self->owns_storage = 1;
    self->world = NULL;
    self->world_id = b3_id_invalid();
    return (PyObject *)self;
}

PyObject *PyRigidBody_FromWorldId(PyObject *world, b3_BodyId id) {
    PyRigidBodyObject *self = PyObject_New(PyRigidBodyObject, &PyRigidBody_Type);
    if (self == NULL) {
        return NULL;
    }
    self->owns_storage = 0;
    Py_INCREF(world);
    self->world = world;
    self->world_id = id;
    return (PyObject *)self;
}

b3_RigidBody *PyRigidBody_Resolve(PyRigidBodyObject *self) {
    if (self->owns_storage) {
        return &self->owned;
    }
    b3_RigidBody *body =
        b3_world_get_body_by_id(&((PyWorldObject *)self->world)->world, self->world_id);
    if (body == NULL) {
        PyErr_SetString(
            PyExc_ValueError,
            "this RigidBody is no longer valid: it was removed from its World "
            "(via World.remove_body, or as a side effect of removing a body "
            "one of its joints referenced)"
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
    self->world_id = b3_id_invalid();
    return (PyObject *)self;
}

static PyObject *rigidbody_sphere(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"position", "radius", "mass", NULL};
    PyObject *position_obj;
    double radius;
    double mass = 0.0;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "Od|d", kwlist, &position_obj, &radius, &mass
        )) {
        return NULL;
    }
    b3_Vec3 position;
    if (PyVec3_Parse(position_obj, &position) < 0) {
        return NULL;
    }

    PyRigidBodyObject *self = (PyRigidBodyObject *)type->tp_alloc(type, 0);
    if (self == NULL) {
        return NULL;
    }
    b3_rigidbody_init_sphere(&self->owned, position, (b3_real)radius, (b3_real)mass);
    self->owns_storage = 1;
    self->world = NULL;
    self->world_id = b3_id_invalid();
    return (PyObject *)self;
}

static PyObject *rigidbody_capsule(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"position", "radius", "half_height", "mass", NULL};
    PyObject *position_obj;
    double radius, half_height;
    double mass = 0.0;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "Odd|d", kwlist, &position_obj, &radius, &half_height, &mass
        )) {
        return NULL;
    }
    b3_Vec3 position;
    if (PyVec3_Parse(position_obj, &position) < 0) {
        return NULL;
    }

    PyRigidBodyObject *self = (PyRigidBodyObject *)type->tp_alloc(type, 0);
    if (self == NULL) {
        return NULL;
    }
    b3_rigidbody_init_capsule(&self->owned, position, (b3_real)radius, (b3_real)half_height, (b3_real)mass);
    self->owns_storage = 1;
    self->world = NULL;
    self->world_id = b3_id_invalid();
    return (PyObject *)self;
}

static PyObject *rigidbody_hull(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"position", "vertices", "mass", NULL};
    PyObject *position_obj, *vertices_obj;
    double mass = 0.0;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "OO|d", kwlist, &position_obj, &vertices_obj, &mass
        )) {
        return NULL;
    }
    b3_Vec3 position;
    if (PyVec3_Parse(position_obj, &position) < 0) {
        return NULL;
    }

    PyObject *seq = PySequence_Fast(vertices_obj, "vertices must be a sequence of Vec3-like values");
    if (seq == NULL) {
        return NULL;
    }
    Py_ssize_t n = PySequence_Fast_GET_SIZE(seq);
    if (n > B3_HULL_MAX_VERTICES) {
        Py_DECREF(seq);
        PyErr_Format(
            PyExc_ValueError, "hull supports at most %d vertices, got %zd", B3_HULL_MAX_VERTICES, n
        );
        return NULL;
    }
    b3_Vec3 local_vertices[B3_HULL_MAX_VERTICES];
    for (Py_ssize_t i = 0; i < n; i++) {
        if (PyVec3_Parse(PySequence_Fast_GET_ITEM(seq, i), &local_vertices[i]) < 0) {
            Py_DECREF(seq);
            return NULL;
        }
    }
    Py_DECREF(seq);

    PyRigidBodyObject *self = (PyRigidBodyObject *)type->tp_alloc(type, 0);
    if (self == NULL) {
        return NULL;
    }
    b3_rigidbody_init_hull(&self->owned, position, local_vertices, (int)n, (b3_real)mass);
    self->owns_storage = 1;
    self->world = NULL;
    self->world_id = b3_id_invalid();
    return (PyObject *)self;
}

static PyObject *rigidbody_compound(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"position", "children", "mass", NULL};
    PyObject *position_obj, *children_obj;
    double mass = 0.0;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "OO|d", kwlist, &position_obj, &children_obj, &mass
        )) {
        return NULL;
    }
    b3_Vec3 position;
    if (PyVec3_Parse(position_obj, &position) < 0) {
        return NULL;
    }

    b3_CompoundChild children[B3_COMPOUND_MAX_CHILDREN];
    int child_count;
    if (PyCompound_ParseChildren(children_obj, children, &child_count) < 0) {
        return NULL;
    }

    PyRigidBodyObject *self = (PyRigidBodyObject *)type->tp_alloc(type, 0);
    if (self == NULL) {
        return NULL;
    }
    b3_rigidbody_init_compound(&self->owned, position, children, child_count, (b3_real)mass);
    self->owns_storage = 1;
    self->world = NULL;
    self->world_id = b3_id_invalid();
    return (PyObject *)self;
}

static PyObject *rigidbody_mesh(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"position", "triangles", NULL};
    PyObject *position_obj, *triangles_obj;
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "OO", kwlist, &position_obj, &triangles_obj)) {
        return NULL;
    }
    b3_Vec3 position;
    if (PyVec3_Parse(position_obj, &position) < 0) {
        return NULL;
    }

    PyObject *seq = PySequence_Fast(triangles_obj, "triangles must be a sequence of 3-point sequences");
    if (seq == NULL) {
        return NULL;
    }
    Py_ssize_t n = PySequence_Fast_GET_SIZE(seq);
    if (n > B3_MESH_MAX_TRIANGLES) {
        Py_DECREF(seq);
        PyErr_Format(
            PyExc_ValueError, "mesh supports at most %d triangles, got %zd", B3_MESH_MAX_TRIANGLES, n
        );
        return NULL;
    }
    b3_Vec3 local_vertices[B3_MESH_MAX_TRIANGLES * 3];
    for (Py_ssize_t i = 0; i < n; i++) {
        PyObject *tri_seq = PySequence_Fast(
            PySequence_Fast_GET_ITEM(seq, i), "each triangle must be a 3-point sequence"
        );
        if (tri_seq == NULL || PySequence_Fast_GET_SIZE(tri_seq) != 3) {
            Py_XDECREF(tri_seq);
            Py_DECREF(seq);
            if (!PyErr_Occurred()) {
                PyErr_SetString(PyExc_ValueError, "each triangle must have exactly 3 points");
            }
            return NULL;
        }
        for (int k = 0; k < 3; k++) {
            if (PyVec3_Parse(PySequence_Fast_GET_ITEM(tri_seq, k), &local_vertices[i * 3 + k]) < 0) {
                Py_DECREF(tri_seq);
                Py_DECREF(seq);
                return NULL;
            }
        }
        Py_DECREF(tri_seq);
    }
    Py_DECREF(seq);

    PyRigidBodyObject *self = (PyRigidBodyObject *)type->tp_alloc(type, 0);
    if (self == NULL) {
        return NULL;
    }
    b3_rigidbody_init_mesh(&self->owned, position, local_vertices, (int)n);
    self->owns_storage = 1;
    self->world = NULL;
    self->world_id = b3_id_invalid();
    return (PyObject *)self;
}

static PyObject *rigidbody_heightfield(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"position", "heights", "cell_size", NULL};
    PyObject *position_obj, *heights_obj;
    double cell_size = 1.0;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "OO|d", kwlist, &position_obj, &heights_obj, &cell_size
        )) {
        return NULL;
    }
    b3_Vec3 position;
    if (PyVec3_Parse(position_obj, &position) < 0) {
        return NULL;
    }

    PyObject *rows_seq = PySequence_Fast(heights_obj, "heights must be a sequence of rows");
    if (rows_seq == NULL) {
        return NULL;
    }
    Py_ssize_t rows = PySequence_Fast_GET_SIZE(rows_seq);
    if (rows == 0 || rows > B3_HEIGHTFIELD_MAX_ROWS) {
        Py_DECREF(rows_seq);
        PyErr_Format(
            PyExc_ValueError, "heightfield supports 1-%d rows, got %zd", B3_HEIGHTFIELD_MAX_ROWS, rows
        );
        return NULL;
    }
    b3_real heights[B3_HEIGHTFIELD_MAX_ROWS * B3_HEIGHTFIELD_MAX_COLS];
    Py_ssize_t cols = -1;
    for (Py_ssize_t r = 0; r < rows; r++) {
        PyObject *row_seq = PySequence_Fast(
            PySequence_Fast_GET_ITEM(rows_seq, r), "each heights row must be a sequence of floats"
        );
        if (row_seq == NULL) {
            Py_DECREF(rows_seq);
            return NULL;
        }
        Py_ssize_t this_cols = PySequence_Fast_GET_SIZE(row_seq);
        if (cols < 0) {
            cols = this_cols;
            if (cols > B3_HEIGHTFIELD_MAX_COLS) {
                Py_DECREF(row_seq);
                Py_DECREF(rows_seq);
                PyErr_Format(
                    PyExc_ValueError, "heightfield supports at most %d columns, got %zd",
                    B3_HEIGHTFIELD_MAX_COLS, cols
                );
                return NULL;
            }
        } else if (this_cols != cols) {
            Py_DECREF(row_seq);
            Py_DECREF(rows_seq);
            PyErr_SetString(PyExc_ValueError, "every heights row must have the same length");
            return NULL;
        }
        for (Py_ssize_t c = 0; c < cols; c++) {
            double v = PyFloat_AsDouble(PySequence_Fast_GET_ITEM(row_seq, c));
            if (v == -1.0 && PyErr_Occurred()) {
                Py_DECREF(row_seq);
                Py_DECREF(rows_seq);
                return NULL;
            }
            heights[r * cols + c] = (b3_real)v;
        }
        Py_DECREF(row_seq);
    }
    Py_DECREF(rows_seq);

    PyRigidBodyObject *self = (PyRigidBodyObject *)type->tp_alloc(type, 0);
    if (self == NULL) {
        return NULL;
    }
    b3_rigidbody_init_heightfield(&self->owned, position, (b3_real)cell_size, heights, (int)rows, (int)cols);
    self->owns_storage = 1;
    self->world = NULL;
    self->world_id = b3_id_invalid();
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
    b3_rigidbody_wake(body);
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
    b3_rigidbody_wake(body);
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
    if (PyVec3_Parse(value, &body->linear_velocity) < 0) return -1;
    b3_rigidbody_wake(body);
    return 0;
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
    if (PyVec3_Parse(value, &body->angular_velocity) < 0) return -1;
    b3_rigidbody_wake(body);
    return 0;
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
    if (mass > 0.0 && (body->shape.kind == B3_SHAPE_MESH || body->shape.kind == B3_SHAPE_HEIGHTFIELD)) {
        PyErr_SetString(
            PyExc_ValueError, "TriangleMesh/HeightField bodies are always static (mass must be 0)"
        );
        return -1;
    }
    b3_rigidbody_set_mass(body, (b3_real)mass);
    b3_rigidbody_wake(body);
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
    return PyShape_Wrap(body->shape);
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

static PyObject *rigidbody_get_is_sleeping(PyRigidBodyObject *self, void *closure) {
    (void)closure;
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return NULL;
    return PyBool_FromLong(body->is_sleeping);
}

static PyGetSetDef rigidbody_getset[] = {
    {"position", (getter)rigidbody_get_position, (setter)rigidbody_set_position, "world-space position (Vec3)", NULL},
    {"orientation", (getter)rigidbody_get_orientation, (setter)rigidbody_set_orientation, "orientation (Quat)", NULL},
    {"linear_velocity", (getter)rigidbody_get_linear_velocity, (setter)rigidbody_set_linear_velocity, "linear velocity (Vec3)", NULL},
    {"angular_velocity", (getter)rigidbody_get_angular_velocity, (setter)rigidbody_set_angular_velocity, "angular velocity, rad/s (Vec3)", NULL},
    {"mass", (getter)rigidbody_get_mass, (setter)rigidbody_set_mass, "mass; 0 means static (infinite mass)", NULL},
    {"inv_mass", (getter)rigidbody_get_inv_mass, NULL, "1/mass, or 0 for static bodies (read-only)", NULL},
    {"restitution", (getter)rigidbody_get_restitution, (setter)rigidbody_set_restitution, "bounciness coefficient; combined with the other body's via max() for each contact", NULL},
    {"friction", (getter)rigidbody_get_friction, (setter)rigidbody_set_friction, "friction coefficient; combined with the other body's via sqrt(a*b) for each contact", NULL},
    {"shape", (getter)rigidbody_get_shape, NULL, "current collision shape as a snapshot, kept in sync with position/orientation (read-only)", NULL},
    {"is_static", (getter)rigidbody_get_is_static, NULL, "True if mass == 0", NULL},
    {"is_sleeping", (getter)rigidbody_get_is_sleeping, NULL,
     "True if World.step is currently skipping this body's integration (see World.sleeping_enabled); "
     "always False for a standalone body not attached to a World (read-only)", NULL},
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
    b3_rigidbody_wake(body);
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
    b3_rigidbody_wake(body);
    Py_RETURN_NONE;
}

static PyObject *rigidbody_clear_accumulators(PyRigidBodyObject *self, PyObject *Py_UNUSED(ignored)) {
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return NULL;
    b3_rigidbody_clear_accumulators(body);
    Py_RETURN_NONE;
}

static PyObject *rigidbody_wake(PyRigidBodyObject *self, PyObject *Py_UNUSED(ignored)) {
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return NULL;
    b3_rigidbody_wake(body);
    Py_RETURN_NONE;
}

static PyObject *rigidbody_debug_lines(PyRigidBodyObject *self, PyObject *Py_UNUSED(ignored)) {
    b3_RigidBody *body = PyRigidBody_Resolve(self);
    if (body == NULL) return NULL;

    int count = b3_shape_debug_lines(&body->shape, NULL, 0);
    b3_DebugLine *lines = count > 0 ? (b3_DebugLine *)malloc((size_t)count * sizeof(b3_DebugLine)) : NULL;
    if (count > 0 && lines == NULL) {
        return PyErr_NoMemory();
    }
    b3_shape_debug_lines(&body->shape, lines, count);

    PyObject *list = PyList_New(count);
    if (list == NULL) {
        free(lines);
        return NULL;
    }
    for (int i = 0; i < count; i++) {
        PyObject *a = PyVec3_FromVec3(lines[i].a);
        PyObject *b = a != NULL ? PyVec3_FromVec3(lines[i].b) : NULL;
        PyObject *pair = b != NULL ? PyTuple_Pack(2, a, b) : NULL;
        Py_XDECREF(a);
        Py_XDECREF(b);
        if (pair == NULL) {
            free(lines);
            Py_DECREF(list);
            return NULL;
        }
        PyList_SET_ITEM(list, i, pair);
    }
    free(lines);
    return list;
}

static PyMethodDef rigidbody_methods[] = {
    {"sphere", (PyCFunction)rigidbody_sphere, METH_CLASS | METH_VARARGS | METH_KEYWORDS,
     "sphere(position, radius, mass=0.0) -> RigidBody (a standalone sphere-shaped body)"},
    {"capsule", (PyCFunction)rigidbody_capsule, METH_CLASS | METH_VARARGS | METH_KEYWORDS,
     "capsule(position, radius, half_height, mass=0.0) -> RigidBody "
     "(a standalone capsule-shaped body)"},
    {"hull", (PyCFunction)rigidbody_hull, METH_CLASS | METH_VARARGS | METH_KEYWORDS,
     "hull(position, vertices, mass=0.0) -> RigidBody (a standalone convex-hull-shaped body; "
     "vertices are local to position, at most ConvexHull's vertex cap)"},
    {"compound", (PyCFunction)rigidbody_compound, METH_CLASS | METH_VARARGS | METH_KEYWORDS,
     "compound(position, children, mass=0.0) -> RigidBody (a standalone compound-shaped body; "
     "see Compound for the children format)"},
    {"mesh", (PyCFunction)rigidbody_mesh, METH_CLASS | METH_VARARGS | METH_KEYWORDS,
     "mesh(position, triangles) -> RigidBody (a standalone, always-static triangle-mesh body; "
     "see TriangleMesh)"},
    {"heightfield", (PyCFunction)rigidbody_heightfield, METH_CLASS | METH_VARARGS | METH_KEYWORDS,
     "heightfield(position, heights, cell_size=1.0) -> RigidBody (a standalone, always-static "
     "height-field body; see HeightField)"},
    {"apply_force", (PyCFunction)rigidbody_apply_force, METH_VARARGS | METH_KEYWORDS,
     "apply_force(force, point=None) -> None (point defaults to the body's position, i.e. no torque)"},
    {"apply_impulse", (PyCFunction)rigidbody_apply_impulse, METH_VARARGS | METH_KEYWORDS,
     "apply_impulse(impulse, point=None) -> None"},
    {"clear_accumulators", (PyCFunction)rigidbody_clear_accumulators, METH_NOARGS,
     "clear_accumulators() -> None"},
    {"wake", (PyCFunction)rigidbody_wake, METH_NOARGS,
     "wake() -> None (clears is_sleeping/its sleep timer; see World.sleeping_enabled -- a no-op "
     "if this body isn't attached to a World, or is already awake)"},
    {"debug_lines", (PyCFunction)rigidbody_debug_lines, METH_NOARGS,
     "debug_lines() -> list[tuple[Vec3, Vec3]] (a wireframe approximation of this body's current "
     "shape, in world space -- pure data, no rendering; see docs)"},
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
