#include "py_vec3.h"

#include "structmember.h"

PyObject *PyVec3_FromVec3(b3_Vec3 v) {
    PyVec3Object *obj = PyObject_New(PyVec3Object, &PyVec3_Type);
    if (obj == NULL) {
        return NULL;
    }
    obj->value = v;
    return (PyObject *)obj;
}

static int parse_number(PyObject *obj, b3_real *out) {
    double d = PyFloat_AsDouble(obj);
    if (d == -1.0 && PyErr_Occurred()) {
        return -1;
    }
    *out = (b3_real)d;
    return 0;
}

int PyVec3_Parse(PyObject *obj, b3_Vec3 *out) {
    if (PyVec3_Check(obj)) {
        *out = ((PyVec3Object *)obj)->value;
        return 0;
    }

    PyObject *seq = PySequence_Fast(obj, "expected a Vec3 or a sequence of 3 numbers");
    if (seq == NULL) {
        return -1;
    }
    Py_ssize_t len = PySequence_Fast_GET_SIZE(seq);
    if (len != 3) {
        PyErr_Format(PyExc_ValueError, "expected a sequence of 3 numbers, got length %zd", len);
        Py_DECREF(seq);
        return -1;
    }

    b3_real components[3];
    for (Py_ssize_t i = 0; i < 3; i++) {
        if (parse_number(PySequence_Fast_GET_ITEM(seq, i), &components[i]) < 0) {
            Py_DECREF(seq);
            return -1;
        }
    }
    Py_DECREF(seq);
    *out = b3_vec3_make(components[0], components[1], components[2]);
    return 0;
}

static PyObject *vec3_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"x", "y", "z", NULL};
    double x = 0.0, y = 0.0, z = 0.0;
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|ddd", kwlist, &x, &y, &z)) {
        return NULL;
    }
    PyVec3Object *self = (PyVec3Object *)type->tp_alloc(type, 0);
    if (self == NULL) {
        return NULL;
    }
    self->value = b3_vec3_make((b3_real)x, (b3_real)y, (b3_real)z);
    return (PyObject *)self;
}

static void vec3_dealloc(PyVec3Object *self) {
    Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyObject *vec3_repr(PyVec3Object *self) {
    PyObject *fx = PyFloat_FromDouble(self->value.x);
    PyObject *fy = PyFloat_FromDouble(self->value.y);
    PyObject *fz = PyFloat_FromDouble(self->value.z);
    if (fx == NULL || fy == NULL || fz == NULL) {
        Py_XDECREF(fx);
        Py_XDECREF(fy);
        Py_XDECREF(fz);
        return NULL;
    }
    PyObject *result = PyUnicode_FromFormat("Vec3(%R, %R, %R)", fx, fy, fz);
    Py_DECREF(fx);
    Py_DECREF(fy);
    Py_DECREF(fz);
    return result;
}

static Py_hash_t vec3_hash(PyVec3Object *self) {
    PyObject *tuple = Py_BuildValue("(ddd)", (double)self->value.x, (double)self->value.y, (double)self->value.z);
    if (tuple == NULL) {
        return -1;
    }
    Py_hash_t h = PyObject_Hash(tuple);
    Py_DECREF(tuple);
    return h;
}

static PyObject *vec3_richcompare(PyObject *a, PyObject *b, int op) {
    if (!PyVec3_Check(a) || !PyVec3_Check(b) || (op != Py_EQ && op != Py_NE)) {
        Py_RETURN_NOTIMPLEMENTED;
    }
    int equal = b3_vec3_equal(((PyVec3Object *)a)->value, ((PyVec3Object *)b)->value);
    int result = (op == Py_EQ) ? equal : !equal;
    if (result) {
        Py_RETURN_TRUE;
    }
    Py_RETURN_FALSE;
}

/* --- number protocol --- */

static PyObject *vec3_add(PyObject *a, PyObject *b) {
    b3_Vec3 va, vb;
    if (!PyVec3_Check(a) || !PyVec3_Check(b)) {
        Py_RETURN_NOTIMPLEMENTED;
    }
    va = ((PyVec3Object *)a)->value;
    vb = ((PyVec3Object *)b)->value;
    return PyVec3_FromVec3(b3_vec3_add(va, vb));
}

static PyObject *vec3_sub(PyObject *a, PyObject *b) {
    if (!PyVec3_Check(a) || !PyVec3_Check(b)) {
        Py_RETURN_NOTIMPLEMENTED;
    }
    return PyVec3_FromVec3(b3_vec3_sub(((PyVec3Object *)a)->value, ((PyVec3Object *)b)->value));
}

static PyObject *vec3_mul(PyObject *a, PyObject *b) {
    PyVec3Object *vec_obj;
    PyObject *scalar_obj;
    if (PyVec3_Check(a)) {
        vec_obj = (PyVec3Object *)a;
        scalar_obj = b;
    } else if (PyVec3_Check(b)) {
        vec_obj = (PyVec3Object *)b;
        scalar_obj = a;
    } else {
        Py_RETURN_NOTIMPLEMENTED;
    }
    if (!PyNumber_Check(scalar_obj)) {
        Py_RETURN_NOTIMPLEMENTED;
    }
    b3_real s;
    if (parse_number(scalar_obj, &s) < 0) {
        return NULL;
    }
    return PyVec3_FromVec3(b3_vec3_scale(vec_obj->value, s));
}

static PyObject *vec3_negative(PyObject *a) {
    return PyVec3_FromVec3(b3_vec3_negate(((PyVec3Object *)a)->value));
}

static PyNumberMethods vec3_as_number = {
    .nb_add = vec3_add,
    .nb_subtract = vec3_sub,
    .nb_multiply = vec3_mul,
    .nb_negative = vec3_negative,
};

/* --- getset --- */

static PyObject *vec3_get_x(PyVec3Object *self, void *closure) {
    (void)closure;
    return PyFloat_FromDouble(self->value.x);
}
static PyObject *vec3_get_y(PyVec3Object *self, void *closure) {
    (void)closure;
    return PyFloat_FromDouble(self->value.y);
}
static PyObject *vec3_get_z(PyVec3Object *self, void *closure) {
    (void)closure;
    return PyFloat_FromDouble(self->value.z);
}

static int vec3_set_component(PyVec3Object *self, PyObject *value, void *closure) {
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete Vec3 components");
        return -1;
    }
    b3_real component;
    if (parse_number(value, &component) < 0) {
        return -1;
    }
    Py_ssize_t which = (Py_ssize_t)(intptr_t)closure;
    if (which == 0) self->value.x = component;
    else if (which == 1) self->value.y = component;
    else self->value.z = component;
    return 0;
}

static PyGetSetDef vec3_getset[] = {
    {"x", (getter)vec3_get_x, (setter)vec3_set_component, "x component", (void *)0},
    {"y", (getter)vec3_get_y, (setter)vec3_set_component, "y component", (void *)1},
    {"z", (getter)vec3_get_z, (setter)vec3_set_component, "z component", (void *)2},
    {NULL},
};

/* --- methods --- */

static PyObject *vec3_dot(PyVec3Object *self, PyObject *other) {
    b3_Vec3 o;
    if (PyVec3_Parse(other, &o) < 0) {
        return NULL;
    }
    return PyFloat_FromDouble(b3_vec3_dot(self->value, o));
}

static PyObject *vec3_cross(PyVec3Object *self, PyObject *other) {
    b3_Vec3 o;
    if (PyVec3_Parse(other, &o) < 0) {
        return NULL;
    }
    return PyVec3_FromVec3(b3_vec3_cross(self->value, o));
}

static PyObject *vec3_length(PyVec3Object *self, PyObject *Py_UNUSED(ignored)) {
    return PyFloat_FromDouble(b3_vec3_length(self->value));
}

static PyObject *vec3_length_sq(PyVec3Object *self, PyObject *Py_UNUSED(ignored)) {
    return PyFloat_FromDouble(b3_vec3_length_sq(self->value));
}

static PyObject *vec3_normalized(PyVec3Object *self, PyObject *Py_UNUSED(ignored)) {
    return PyVec3_FromVec3(b3_vec3_normalize(self->value));
}

static PyObject *vec3_to_tuple(PyVec3Object *self, PyObject *Py_UNUSED(ignored)) {
    return Py_BuildValue("(ddd)", (double)self->value.x, (double)self->value.y, (double)self->value.z);
}

static PyMethodDef vec3_methods[] = {
    {"dot", (PyCFunction)vec3_dot, METH_O, "dot(other) -> float"},
    {"cross", (PyCFunction)vec3_cross, METH_O, "cross(other) -> Vec3"},
    {"length", (PyCFunction)vec3_length, METH_NOARGS, "length() -> float"},
    {"length_squared", (PyCFunction)vec3_length_sq, METH_NOARGS, "length_squared() -> float"},
    {"normalized", (PyCFunction)vec3_normalized, METH_NOARGS, "normalized() -> Vec3"},
    {"to_tuple", (PyCFunction)vec3_to_tuple, METH_NOARGS, "to_tuple() -> tuple[float, float, float]"},
    {NULL},
};

PyTypeObject PyVec3_Type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "pybox3d.Vec3",
    .tp_basicsize = sizeof(PyVec3Object),
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = PyDoc_STR("A 3D vector (x, y, z)."),
    .tp_new = vec3_new,
    .tp_dealloc = (destructor)vec3_dealloc,
    .tp_repr = (reprfunc)vec3_repr,
    .tp_hash = (hashfunc)vec3_hash,
    .tp_richcompare = vec3_richcompare,
    .tp_as_number = &vec3_as_number,
    .tp_getset = vec3_getset,
    .tp_methods = vec3_methods,
};
