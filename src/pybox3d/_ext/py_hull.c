#include "py_hull.h"
#include "py_vec3.h"
#include "py_quat.h"
#include "py_box3d.h"
#include "py_shape.h"

#include <float.h>

PyObject *PyConvexHull_FromHull(b3_ConvexHull hull) {
    PyConvexHullObject *obj = PyObject_New(PyConvexHullObject, &PyConvexHull_Type);
    if (obj == NULL) {
        return NULL;
    }
    obj->value = hull;
    return (PyObject *)obj;
}

static PyObject *hull_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"center", "vertices", "orientation", NULL};
    PyObject *center_obj, *vertices_obj;
    PyObject *orientation_obj = Py_None;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "OO|O", kwlist, &center_obj, &vertices_obj, &orientation_obj
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

    PyObject *seq = PySequence_Fast(vertices_obj, "vertices must be a sequence of Vec3-like values");
    if (seq == NULL) {
        return NULL;
    }
    Py_ssize_t n = PySequence_Fast_GET_SIZE(seq);
    if (n > B3_HULL_MAX_VERTICES) {
        Py_DECREF(seq);
        PyErr_Format(
            PyExc_ValueError, "ConvexHull supports at most %d vertices, got %zd",
            B3_HULL_MAX_VERTICES, n
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

    PyConvexHullObject *self = (PyConvexHullObject *)type->tp_alloc(type, 0);
    if (self == NULL) {
        return NULL;
    }
    self->value = b3_convexhull_make(center, orientation, local_vertices, (int)n);
    return (PyObject *)self;
}

static void hull_dealloc(PyConvexHullObject *self) {
    Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyObject *hull_repr(PyConvexHullObject *self) {
    return PyUnicode_FromFormat(
        "ConvexHull(center=(%g, %g, %g), vertex_count=%d)",
        (double)self->value.center.x, (double)self->value.center.y, (double)self->value.center.z,
        self->value.vertex_count
    );
}

static PyObject *hull_get_center(PyConvexHullObject *self, void *closure) {
    (void)closure;
    return PyVec3_FromVec3(self->value.center);
}
static int hull_set_center(PyConvexHullObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete center");
        return -1;
    }
    return PyVec3_Parse(value, &self->value.center);
}

static PyObject *hull_get_orientation(PyConvexHullObject *self, void *closure) {
    (void)closure;
    return PyQuat_FromQuat(self->value.orientation);
}
static int hull_set_orientation(PyConvexHullObject *self, PyObject *value, void *closure) {
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

static PyObject *hull_get_vertex_count(PyConvexHullObject *self, void *closure) {
    (void)closure;
    return PyLong_FromLong(self->value.vertex_count);
}

static PyObject *hull_get_vertices(PyConvexHullObject *self, void *closure) {
    (void)closure;
    PyObject *result = PyTuple_New(self->value.vertex_count);
    if (result == NULL) {
        return NULL;
    }
    for (int i = 0; i < self->value.vertex_count; i++) {
        PyObject *v = PyVec3_FromVec3(self->value.local_vertices[i]);
        if (v == NULL) {
            Py_DECREF(result);
            return NULL;
        }
        PyTuple_SET_ITEM(result, i, v);
    }
    return result;
}

static PyGetSetDef hull_getset[] = {
    {"center", (getter)hull_get_center, (setter)hull_set_center, "world-space center (Vec3)", NULL},
    {"orientation", (getter)hull_get_orientation, (setter)hull_set_orientation, "orientation (Quat)", NULL},
    {"vertex_count", (getter)hull_get_vertex_count, NULL, "number of vertices (read-only)", NULL},
    {"vertices", (getter)hull_get_vertices, NULL,
     "local (unrotated, uncentered) vertices, as passed to the constructor (read-only)", NULL},
    {NULL},
};

static PyObject *hull_contains_point(PyConvexHullObject *self, PyObject *arg) {
    b3_Vec3 point;
    if (PyVec3_Parse(arg, &point) < 0) {
        return NULL;
    }
    if (b3_convexhull_contains_point(&self->value, point)) {
        Py_RETURN_TRUE;
    }
    Py_RETURN_FALSE;
}

static PyObject *hull_aabb(PyConvexHullObject *self, PyObject *Py_UNUSED(ignored)) {
    b3_Vec3 lo, hi;
    b3_convexhull_compute_aabb(&self->value, &lo, &hi);
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

static PyObject *hull_overlaps(PyConvexHullObject *self, PyObject *arg) {
    b3_Shape self_shape = b3_shape_from_hull(self->value);
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

static PyObject *hull_raycast(PyConvexHullObject *self, PyObject *args, PyObject *kwds) {
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

    b3_RayHit rayhit = b3_convexhull_raycast(&self->value, origin, direction, (b3_real)max_t);
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

static PyMethodDef hull_methods[] = {
    {"contains_point", (PyCFunction)hull_contains_point, METH_O, "contains_point(point) -> bool"},
    {"aabb", (PyCFunction)hull_aabb, METH_NOARGS, "aabb() -> (Vec3 min, Vec3 max)"},
    {"overlaps", (PyCFunction)hull_overlaps, METH_O,
     "overlaps(other: Box3D | Sphere | Capsule | ConvexHull) -> ContactInfo | None"},
    {"raycast", (PyCFunction)hull_raycast, METH_VARARGS | METH_KEYWORDS,
     "raycast(origin, direction, max_t=inf) -> RayHit | None (v1: approximated via the hull's AABB, "
     "see docs/limitations.md)"},
    {NULL},
};

PyTypeObject PyConvexHull_Type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "pybox3d.ConvexHull",
    .tp_basicsize = sizeof(PyConvexHullObject),
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = PyDoc_STR(
        "ConvexHull(center, vertices, orientation=None)\n\n"
        "A convex polyhedron from a caller-supplied, already-convex vertex set "
        "(v1: no hull computation from an arbitrary point cloud -- at most "
        "32 vertices, in local/unrotated/uncentered space relative to `center`)."
    ),
    .tp_new = hull_new,
    .tp_dealloc = (destructor)hull_dealloc,
    .tp_repr = (reprfunc)hull_repr,
    .tp_getset = hull_getset,
    .tp_methods = hull_methods,
};
