#include "py_quat.h"
#include "py_vec3.h"

#include <math.h>

PyObject *PyQuat_FromQuat(b3_Quat q) {
    PyQuatObject *obj = PyObject_New(PyQuatObject, &PyQuat_Type);
    if (obj == NULL) {
        return NULL;
    }
    obj->value = q;
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

int PyQuat_Parse(PyObject *obj, b3_Quat *out) {
    if (PyQuat_Check(obj)) {
        *out = ((PyQuatObject *)obj)->value;
        return 0;
    }

    PyObject *seq = PySequence_Fast(obj, "expected a Quat or a sequence of 4 numbers (x, y, z, w)");
    if (seq == NULL) {
        return -1;
    }
    Py_ssize_t len = PySequence_Fast_GET_SIZE(seq);
    if (len != 4) {
        PyErr_Format(PyExc_ValueError, "expected a sequence of 4 numbers, got length %zd", len);
        Py_DECREF(seq);
        return -1;
    }
    b3_real components[4];
    for (Py_ssize_t i = 0; i < 4; i++) {
        if (parse_number(PySequence_Fast_GET_ITEM(seq, i), &components[i]) < 0) {
            Py_DECREF(seq);
            return -1;
        }
    }
    Py_DECREF(seq);
    *out = b3_quat_make(components[0], components[1], components[2], components[3]);
    return 0;
}

static PyObject *quat_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"x", "y", "z", "w", NULL};
    double x = 0.0, y = 0.0, z = 0.0, w = 1.0;
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|dddd", kwlist, &x, &y, &z, &w)) {
        return NULL;
    }
    PyQuatObject *self = (PyQuatObject *)type->tp_alloc(type, 0);
    if (self == NULL) {
        return NULL;
    }
    self->value = b3_quat_make((b3_real)x, (b3_real)y, (b3_real)z, (b3_real)w);
    return (PyObject *)self;
}

static void quat_dealloc(PyQuatObject *self) {
    Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyObject *quat_repr(PyQuatObject *self) {
    PyObject *fx = PyFloat_FromDouble(self->value.x);
    PyObject *fy = PyFloat_FromDouble(self->value.y);
    PyObject *fz = PyFloat_FromDouble(self->value.z);
    PyObject *fw = PyFloat_FromDouble(self->value.w);
    if (fx == NULL || fy == NULL || fz == NULL || fw == NULL) {
        Py_XDECREF(fx);
        Py_XDECREF(fy);
        Py_XDECREF(fz);
        Py_XDECREF(fw);
        return NULL;
    }
    PyObject *result = PyUnicode_FromFormat("Quat(%R, %R, %R, %R)", fx, fy, fz, fw);
    Py_DECREF(fx);
    Py_DECREF(fy);
    Py_DECREF(fz);
    Py_DECREF(fw);
    return result;
}

static PyObject *quat_richcompare(PyObject *a, PyObject *b, int op) {
    if (!PyQuat_Check(a) || !PyQuat_Check(b) || (op != Py_EQ && op != Py_NE)) {
        Py_RETURN_NOTIMPLEMENTED;
    }
    b3_Quat qa = ((PyQuatObject *)a)->value;
    b3_Quat qb = ((PyQuatObject *)b)->value;
    const b3_real eps = 1e-6f;
    int equal = fabsf(qa.x - qb.x) < eps && fabsf(qa.y - qb.y) < eps &&
                fabsf(qa.z - qb.z) < eps && fabsf(qa.w - qb.w) < eps;
    int result = (op == Py_EQ) ? equal : !equal;
    if (result) {
        Py_RETURN_TRUE;
    }
    Py_RETURN_FALSE;
}

static PyObject *quat_mul(PyObject *a, PyObject *b) {
    if (!PyQuat_Check(a) || !PyQuat_Check(b)) {
        Py_RETURN_NOTIMPLEMENTED;
    }
    return PyQuat_FromQuat(b3_quat_mul(((PyQuatObject *)a)->value, ((PyQuatObject *)b)->value));
}

static PyNumberMethods quat_as_number = {
    .nb_multiply = quat_mul,
};

static PyObject *quat_get_x(PyQuatObject *self, void *closure) { (void)closure; return PyFloat_FromDouble(self->value.x); }
static PyObject *quat_get_y(PyQuatObject *self, void *closure) { (void)closure; return PyFloat_FromDouble(self->value.y); }
static PyObject *quat_get_z(PyQuatObject *self, void *closure) { (void)closure; return PyFloat_FromDouble(self->value.z); }
static PyObject *quat_get_w(PyQuatObject *self, void *closure) { (void)closure; return PyFloat_FromDouble(self->value.w); }

static int quat_set_component(PyQuatObject *self, PyObject *value, void *closure) {
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete Quat components");
        return -1;
    }
    b3_real component;
    if (parse_number(value, &component) < 0) {
        return -1;
    }
    Py_ssize_t which = (Py_ssize_t)(intptr_t)closure;
    if (which == 0) self->value.x = component;
    else if (which == 1) self->value.y = component;
    else if (which == 2) self->value.z = component;
    else self->value.w = component;
    return 0;
}

static PyGetSetDef quat_getset[] = {
    {"x", (getter)quat_get_x, (setter)quat_set_component, "x component", (void *)0},
    {"y", (getter)quat_get_y, (setter)quat_set_component, "y component", (void *)1},
    {"z", (getter)quat_get_z, (setter)quat_set_component, "z component", (void *)2},
    {"w", (getter)quat_get_w, (setter)quat_set_component, "w (scalar) component", (void *)3},
    {NULL},
};

static PyObject *quat_from_axis_angle(PyTypeObject *type, PyObject *args) {
    (void)type;
    PyObject *axis_obj;
    double angle;
    if (!PyArg_ParseTuple(args, "Od", &axis_obj, &angle)) {
        return NULL;
    }
    b3_Vec3 axis;
    if (PyVec3_Parse(axis_obj, &axis) < 0) {
        return NULL;
    }
    return PyQuat_FromQuat(b3_quat_from_axis_angle(axis, (b3_real)angle));
}

static PyObject *quat_normalized(PyQuatObject *self, PyObject *Py_UNUSED(ignored)) {
    return PyQuat_FromQuat(b3_quat_normalize(self->value));
}

static PyObject *quat_conjugate(PyQuatObject *self, PyObject *Py_UNUSED(ignored)) {
    return PyQuat_FromQuat(b3_quat_conjugate(self->value));
}

static PyObject *quat_rotate_vec3(PyQuatObject *self, PyObject *arg) {
    b3_Vec3 v;
    if (PyVec3_Parse(arg, &v) < 0) {
        return NULL;
    }
    return PyVec3_FromVec3(b3_quat_rotate_vec3(self->value, v));
}

static PyObject *quat_to_mat3(PyQuatObject *self, PyObject *Py_UNUSED(ignored)) {
    b3_real m[9];
    b3_quat_to_mat3(self->value, m);
    PyObject *result = PyTuple_New(9);
    if (result == NULL) {
        return NULL;
    }
    for (int i = 0; i < 9; i++) {
        PyObject *item = PyFloat_FromDouble((double)m[i]);
        if (item == NULL) {
            Py_DECREF(result);
            return NULL;
        }
        PyTuple_SET_ITEM(result, i, item);
    }
    return result;
}

static PyMethodDef quat_methods[] = {
    {"from_axis_angle", (PyCFunction)quat_from_axis_angle, METH_VARARGS | METH_CLASS,
     "from_axis_angle(axis, angle_radians) -> Quat"},
    {"normalized", (PyCFunction)quat_normalized, METH_NOARGS, "normalized() -> Quat"},
    {"conjugate", (PyCFunction)quat_conjugate, METH_NOARGS, "conjugate() -> Quat"},
    {"rotate_vec3", (PyCFunction)quat_rotate_vec3, METH_O, "rotate_vec3(v) -> Vec3"},
    {"to_mat3", (PyCFunction)quat_to_mat3, METH_NOARGS,
     "to_mat3() -> tuple[float, ...] (row-major 3x3 rotation matrix, 9 elements)"},
    {NULL},
};

PyTypeObject PyQuat_Type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "pybox3d.Quat",
    .tp_basicsize = sizeof(PyQuatObject),
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = PyDoc_STR("A rotation quaternion (x, y, z, w); defaults to identity."),
    .tp_new = quat_new,
    .tp_dealloc = (destructor)quat_dealloc,
    .tp_repr = (reprfunc)quat_repr,
    .tp_richcompare = quat_richcompare,
    .tp_as_number = &quat_as_number,
    .tp_getset = quat_getset,
    .tp_methods = quat_methods,
};
