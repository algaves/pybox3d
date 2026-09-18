#include "py_character.h"
#include "py_vec3.h"
#include "py_shape.h"
#include "py_world.h"

static PyObject *character_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"position", "shape", NULL};
    PyObject *position_obj, *shape_obj;
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "OO", kwlist, &position_obj, &shape_obj)) {
        return NULL;
    }
    b3_Vec3 position;
    if (PyVec3_Parse(position_obj, &position) < 0) return NULL;
    b3_Shape shape;
    if (PyShape_Parse(shape_obj, &shape) < 0) return NULL;

    PyCharacterMoverObject *self = (PyCharacterMoverObject *)type->tp_alloc(type, 0);
    if (self == NULL) {
        return NULL;
    }
    b3_character_init(&self->value, position, shape);
    return (PyObject *)self;
}

static PyObject *character_repr(PyCharacterMoverObject *self) {
    return PyUnicode_FromFormat(
        "CharacterMover(position=(%g, %g, %g), is_grounded=%s)", (double)self->value.position.x,
        (double)self->value.position.y, (double)self->value.position.z,
        self->value.is_grounded ? "True" : "False"
    );
}

static PyObject *character_get_position(PyCharacterMoverObject *self, void *closure) {
    (void)closure;
    return PyVec3_FromVec3(self->value.position);
}
static int character_set_position(PyCharacterMoverObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete position");
        return -1;
    }
    b3_Vec3 v;
    if (PyVec3_Parse(value, &v) < 0) return -1;
    self->value.position = v;
    b3_shape_set_pose(&self->value.shape, v, b3_quat_identity());
    return 0;
}

static PyObject *character_get_velocity(PyCharacterMoverObject *self, void *closure) {
    (void)closure;
    return PyVec3_FromVec3(self->value.velocity);
}
static int character_set_velocity(PyCharacterMoverObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete velocity");
        return -1;
    }
    return PyVec3_Parse(value, &self->value.velocity);
}

static PyObject *character_get_is_grounded(PyCharacterMoverObject *self, void *closure) {
    (void)closure;
    return PyBool_FromLong(self->value.is_grounded);
}

static PyObject *character_get_shape(PyCharacterMoverObject *self, void *closure) {
    (void)closure;
    return PyShape_Wrap(self->value.shape);
}

static PyObject *character_get_skin_width(PyCharacterMoverObject *self, void *closure) {
    (void)closure;
    return PyFloat_FromDouble((double)self->value.skin_width);
}
static int character_set_skin_width(PyCharacterMoverObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete skin_width");
        return -1;
    }
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    if (v < 0.0) {
        PyErr_SetString(PyExc_ValueError, "skin_width must be >= 0");
        return -1;
    }
    self->value.skin_width = (b3_real)v;
    return 0;
}

static PyObject *character_get_max_slide_iterations(PyCharacterMoverObject *self, void *closure) {
    (void)closure;
    return PyLong_FromLong(self->value.max_slide_iterations);
}
static int character_set_max_slide_iterations(PyCharacterMoverObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete max_slide_iterations");
        return -1;
    }
    long v = PyLong_AsLong(value);
    if (v == -1 && PyErr_Occurred()) return -1;
    if (v < 1) {
        PyErr_SetString(PyExc_ValueError, "max_slide_iterations must be >= 1");
        return -1;
    }
    self->value.max_slide_iterations = (int)v;
    return 0;
}

static PyObject *character_get_ground_normal_min_y(PyCharacterMoverObject *self, void *closure) {
    (void)closure;
    return PyFloat_FromDouble((double)self->value.ground_normal_min_y);
}
static int character_set_ground_normal_min_y(PyCharacterMoverObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete ground_normal_min_y");
        return -1;
    }
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    self->value.ground_normal_min_y = (b3_real)v;
    return 0;
}

static PyGetSetDef character_getset[] = {
    {"position", (getter)character_get_position, (setter)character_set_position,
     "world-space position (Vec3)", NULL},
    {"velocity", (getter)character_get_velocity, (setter)character_set_velocity,
     "current velocity (Vec3); move() removes any into-the-surface component it resolves", NULL},
    {"is_grounded", (getter)character_get_is_grounded, NULL,
     "whether the last move() call resolved a contact with an up-facing-enough normal "
     "(>= ground_normal_min_y); not sticky between calls (read-only)", NULL},
    {"shape", (getter)character_get_shape, NULL,
     "current collision shape as a snapshot, kept in sync with position (read-only)", NULL},
    {"skin_width", (getter)character_get_skin_width, (setter)character_set_skin_width,
     "depenetration margin beyond exact contact, added to every push-out (default 0.01)", NULL},
    {"max_slide_iterations", (getter)character_get_max_slide_iterations,
     (setter)character_set_max_slide_iterations,
     "discrete push-out passes per move() call (>= 1, default 4)", NULL},
    {"ground_normal_min_y", (getter)character_get_ground_normal_min_y,
     (setter)character_set_ground_normal_min_y,
     "dot(resolved contact normal, +Y) threshold for is_grounded (default 0.5)", NULL},
    {NULL},
};

static PyObject *character_move(PyCharacterMoverObject *self, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"world", "displacement", NULL};
    PyObject *world_obj, *displacement_obj;
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "OO", kwlist, &world_obj, &displacement_obj)) {
        return NULL;
    }
    if (!PyWorld_Check(world_obj)) {
        PyErr_SetString(PyExc_TypeError, "move() world argument must be a World");
        return NULL;
    }
    b3_Vec3 displacement;
    if (PyVec3_Parse(displacement_obj, &displacement) < 0) return NULL;

    b3_character_move(&self->value, &((PyWorldObject *)world_obj)->world, displacement);
    Py_RETURN_NONE;
}

static PyMethodDef character_methods[] = {
    {"move", (PyCFunction)character_move, METH_VARARGS | METH_KEYWORDS,
     "move(world: World, displacement: VecLike) -> None (moves by `displacement`, then resolves "
     "any resulting overlaps against `world`'s bodies via discrete push-out + slide -- not "
     "continuous/swept collision; see docs)"},
    {NULL},
};

PyTypeObject PyCharacterMover_Type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "pybox3d.CharacterMover",
    .tp_basicsize = sizeof(PyCharacterMoverObject),
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = PyDoc_STR(
        "CharacterMover(position, shape)\n\n"
        "A kinematic move-and-slide character controller, driven directly by the caller each "
        "step (via move()) and tested against a World's bodies for collision -- it is not a "
        "RigidBody and is never added to a World, so it plays no part in that World's own "
        "physics (gravity, joints, contact response all stay up to the caller)."
    ),
    .tp_new = character_new,
    .tp_repr = (reprfunc)character_repr,
    .tp_getset = character_getset,
    .tp_methods = character_methods,
};
