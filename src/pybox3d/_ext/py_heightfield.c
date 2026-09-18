#include "py_heightfield.h"
#include "py_vec3.h"
#include "py_quat.h"
#include "py_box3d.h"
#include "py_shape.h"

#include <float.h>

PyObject *PyHeightField_FromHeightField(b3_HeightField hf) {
    PyHeightFieldObject *obj = PyObject_New(PyHeightFieldObject, &PyHeightField_Type);
    if (obj == NULL) {
        return NULL;
    }
    obj->value = hf;
    return (PyObject *)obj;
}

/* Parses `heights_obj` (a sequence of equal-length row sequences of
 * floats) into a flat row-major array. Returns 0 on success, -1 with an
 * exception set otherwise. */
static int parse_heights(
    PyObject *heights_obj, b3_real out[B3_HEIGHTFIELD_MAX_ROWS * B3_HEIGHTFIELD_MAX_COLS],
    int *out_rows, int *out_cols
) {
    PyObject *rows_seq = PySequence_Fast(heights_obj, "heights must be a sequence of rows");
    if (rows_seq == NULL) {
        return -1;
    }
    Py_ssize_t rows = PySequence_Fast_GET_SIZE(rows_seq);
    if (rows > B3_HEIGHTFIELD_MAX_ROWS) {
        Py_DECREF(rows_seq);
        PyErr_Format(
            PyExc_ValueError, "HeightField supports at most %d rows, got %zd",
            B3_HEIGHTFIELD_MAX_ROWS, rows
        );
        return -1;
    }
    if (rows == 0) {
        Py_DECREF(rows_seq);
        PyErr_SetString(PyExc_ValueError, "heights must have at least one row");
        return -1;
    }

    Py_ssize_t cols = -1;
    for (Py_ssize_t r = 0; r < rows; r++) {
        PyObject *row_obj = PySequence_Fast_GET_ITEM(rows_seq, r);
        PyObject *row_seq = PySequence_Fast(row_obj, "each heights row must be a sequence of floats");
        if (row_seq == NULL) {
            Py_DECREF(rows_seq);
            return -1;
        }
        Py_ssize_t this_cols = PySequence_Fast_GET_SIZE(row_seq);
        if (cols < 0) {
            cols = this_cols;
            if (cols > B3_HEIGHTFIELD_MAX_COLS) {
                Py_DECREF(row_seq);
                Py_DECREF(rows_seq);
                PyErr_Format(
                    PyExc_ValueError, "HeightField supports at most %d columns, got %zd",
                    B3_HEIGHTFIELD_MAX_COLS, cols
                );
                return -1;
            }
        } else if (this_cols != cols) {
            Py_DECREF(row_seq);
            Py_DECREF(rows_seq);
            PyErr_SetString(PyExc_ValueError, "every heights row must have the same length");
            return -1;
        }
        for (Py_ssize_t c = 0; c < cols; c++) {
            double v = PyFloat_AsDouble(PySequence_Fast_GET_ITEM(row_seq, c));
            if (v == -1.0 && PyErr_Occurred()) {
                Py_DECREF(row_seq);
                Py_DECREF(rows_seq);
                return -1;
            }
            out[r * cols + c] = (b3_real)v;
        }
        Py_DECREF(row_seq);
    }
    Py_DECREF(rows_seq);
    *out_rows = (int)rows;
    *out_cols = (int)cols;
    return 0;
}

static PyObject *heightfield_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"center", "heights", "cell_size", "orientation", NULL};
    PyObject *center_obj, *heights_obj;
    double cell_size = 1.0;
    PyObject *orientation_obj = Py_None;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "OO|dO", kwlist, &center_obj, &heights_obj, &cell_size, &orientation_obj
        )) {
        return NULL;
    }
    b3_Vec3 center;
    if (PyVec3_Parse(center_obj, &center) < 0) {
        return NULL;
    }
    b3_Quat orientation = b3_quat_identity();
    if (orientation_obj != Py_None && PyQuat_Parse(orientation_obj, &orientation) < 0) {
        return NULL;
    }

    b3_real heights[B3_HEIGHTFIELD_MAX_ROWS * B3_HEIGHTFIELD_MAX_COLS];
    int rows, cols;
    if (parse_heights(heights_obj, heights, &rows, &cols) < 0) {
        return NULL;
    }

    PyHeightFieldObject *self = (PyHeightFieldObject *)type->tp_alloc(type, 0);
    if (self == NULL) {
        return NULL;
    }
    self->value = b3_heightfield_make(center, orientation, (b3_real)cell_size, heights, rows, cols);
    return (PyObject *)self;
}

static void heightfield_dealloc(PyHeightFieldObject *self) {
    Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyObject *heightfield_repr(PyHeightFieldObject *self) {
    return PyUnicode_FromFormat(
        "HeightField(center=(%g, %g, %g), rows=%d, cols=%d)",
        (double)self->value.center.x, (double)self->value.center.y, (double)self->value.center.z,
        self->value.rows, self->value.cols
    );
}

static PyObject *heightfield_get_center(PyHeightFieldObject *self, void *closure) {
    (void)closure;
    return PyVec3_FromVec3(self->value.center);
}
static int heightfield_set_center(PyHeightFieldObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete center");
        return -1;
    }
    return PyVec3_Parse(value, &self->value.center);
}

static PyObject *heightfield_get_orientation(PyHeightFieldObject *self, void *closure) {
    (void)closure;
    return PyQuat_FromQuat(self->value.orientation);
}
static int heightfield_set_orientation(PyHeightFieldObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete orientation");
        return -1;
    }
    b3_Quat q;
    if (PyQuat_Parse(value, &q) < 0) {
        return -1;
    }
    self->value.orientation = b3_quat_normalize(q);
    return 0;
}

static PyObject *heightfield_get_cell_size(PyHeightFieldObject *self, void *closure) {
    (void)closure;
    return PyFloat_FromDouble((double)self->value.cell_size);
}
static int heightfield_set_cell_size(PyHeightFieldObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete cell_size");
        return -1;
    }
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    self->value.cell_size = (b3_real)v;
    return 0;
}

static PyObject *heightfield_get_rows(PyHeightFieldObject *self, void *closure) {
    (void)closure;
    return PyLong_FromLong(self->value.rows);
}
static PyObject *heightfield_get_cols(PyHeightFieldObject *self, void *closure) {
    (void)closure;
    return PyLong_FromLong(self->value.cols);
}

static PyGetSetDef heightfield_getset[] = {
    {"center", (getter)heightfield_get_center, (setter)heightfield_set_center, "world-space center (Vec3)", NULL},
    {"orientation", (getter)heightfield_get_orientation, (setter)heightfield_set_orientation, "orientation (Quat)", NULL},
    {"cell_size", (getter)heightfield_get_cell_size, (setter)heightfield_set_cell_size,
     "grid spacing along local X/Z", NULL},
    {"rows", (getter)heightfield_get_rows, NULL, "number of grid rows (read-only)", NULL},
    {"cols", (getter)heightfield_get_cols, NULL, "number of grid columns (read-only)", NULL},
    {NULL},
};

static PyObject *heightfield_contains_point(PyHeightFieldObject *self, PyObject *arg) {
    b3_Vec3 point;
    if (PyVec3_Parse(arg, &point) < 0) {
        return NULL;
    }
    if (b3_heightfield_contains_point(&self->value, point)) {
        Py_RETURN_TRUE;
    }
    Py_RETURN_FALSE;
}

static PyObject *heightfield_aabb(PyHeightFieldObject *self, PyObject *Py_UNUSED(ignored)) {
    b3_Vec3 lo, hi;
    b3_heightfield_compute_aabb(&self->value, &lo, &hi);
    PyObject *lo_obj = PyVec3_FromVec3(lo);
    if (lo_obj == NULL) {
        return NULL;
    }
    PyObject *hi_obj = PyVec3_FromVec3(hi);
    if (hi_obj == NULL) {
        Py_DECREF(lo_obj);
        return NULL;
    }
    PyObject *result = PyTuple_Pack(2, lo_obj, hi_obj);
    Py_DECREF(lo_obj);
    Py_DECREF(hi_obj);
    return result;
}

static PyObject *heightfield_overlaps(PyHeightFieldObject *self, PyObject *arg) {
    b3_Shape self_shape = b3_shape_from_heightfield(self->value);
    b3_Shape other_shape;
    if (PyShape_Parse(arg, &other_shape) < 0) {
        return NULL;
    }
    b3_ContactInfo contact;
    int hit = b3_shape_overlap(&self_shape, &other_shape, &contact);
    if (!hit) {
        Py_RETURN_NONE;
    }

    PyObject *result = PyStructSequence_New(PyContactInfo_Type);
    if (result == NULL) {
        return NULL;
    }
    PyObject *normal = PyVec3_FromVec3(contact.normal);
    PyObject *penetration = normal ? PyFloat_FromDouble((double)contact.penetration) : NULL;
    PyObject *point = penetration ? PyVec3_FromVec3(contact.point) : NULL;
    if (normal == NULL || penetration == NULL || point == NULL) {
        Py_XDECREF(normal);
        Py_XDECREF(penetration);
        Py_XDECREF(point);
        Py_DECREF(result);
        return NULL;
    }
    PyStructSequence_SET_ITEM(result, 0, normal);
    PyStructSequence_SET_ITEM(result, 1, penetration);
    PyStructSequence_SET_ITEM(result, 2, point);
    return result;
}

static PyObject *heightfield_raycast(PyHeightFieldObject *self, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"origin", "direction", "max_t", NULL};
    PyObject *origin_obj, *direction_obj;
    double max_t = DBL_MAX;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "OO|d", kwlist, &origin_obj, &direction_obj, &max_t
        )) {
        return NULL;
    }
    b3_Vec3 origin, direction;
    if (PyVec3_Parse(origin_obj, &origin) < 0) return NULL;
    if (PyVec3_Parse(direction_obj, &direction) < 0) return NULL;

    b3_RayHit rayhit = b3_heightfield_raycast(&self->value, origin, direction, (b3_real)max_t);
    if (!rayhit.hit) {
        Py_RETURN_NONE;
    }

    PyObject *result = PyStructSequence_New(PyRayHit_Type);
    if (result == NULL) {
        return NULL;
    }
    PyObject *t = PyFloat_FromDouble((double)rayhit.t);
    PyObject *point = t ? PyVec3_FromVec3(rayhit.point) : NULL;
    PyObject *normal = point ? PyVec3_FromVec3(rayhit.normal) : NULL;
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

static PyMethodDef heightfield_methods[] = {
    {"contains_point", (PyCFunction)heightfield_contains_point, METH_O,
     "contains_point(point) -> bool (v1: always False, see docs/limitations.md)"},
    {"aabb", (PyCFunction)heightfield_aabb, METH_NOARGS, "aabb() -> (Vec3 min, Vec3 max)"},
    {"overlaps", (PyCFunction)heightfield_overlaps, METH_O,
     "overlaps(other: Box3D | Sphere | Capsule | ConvexHull | Compound) -> ContactInfo | None"},
    {"raycast", (PyCFunction)heightfield_raycast, METH_VARARGS | METH_KEYWORDS,
     "raycast(origin, direction, max_t=inf) -> RayHit | None (v1: approximated via the field's "
     "AABB, see docs/limitations.md)"},
    {NULL},
};

PyTypeObject PyHeightField_Type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "pybox3d.HeightField",
    .tp_basicsize = sizeof(PyHeightFieldObject),
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = PyDoc_STR(
        "HeightField(center, heights, cell_size=1.0, orientation=None)\n\n"
        "A static-only terrain grid: `heights` is a sequence of equal-length "
        "rows of local Y heights, at most 16x16. Only static bodies (mass=0) "
        "can use it, see RigidBody.heightfield()."
    ),
    .tp_new = heightfield_new,
    .tp_dealloc = (destructor)heightfield_dealloc,
    .tp_repr = (reprfunc)heightfield_repr,
    .tp_getset = heightfield_getset,
    .tp_methods = heightfield_methods,
};
