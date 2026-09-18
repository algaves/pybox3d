#include "py_compound.h"
#include "py_vec3.h"
#include "py_quat.h"
#include "py_box3d.h"
#include "py_shape.h"

#include <float.h>

PyObject *PyCompound_FromCompound(b3_Compound compound) {
    PyCompoundObject *obj = PyObject_New(PyCompoundObject, &PyCompound_Type);
    if (obj == NULL) {
        return NULL;
    }
    obj->value = compound;
    return (PyObject *)obj;
}

static int shape_to_compound_child(
    b3_Vec3 local_position, b3_Quat local_orientation, const b3_Shape *shape, b3_CompoundChild *out
) {
    switch (shape->kind) {
        case B3_SHAPE_SPHERE:
            *out = b3_compound_child_sphere(local_position, shape->as.sphere);
            return 0;
        case B3_SHAPE_CAPSULE:
            *out = b3_compound_child_capsule(local_position, local_orientation, shape->as.capsule);
            return 0;
        case B3_SHAPE_HULL:
            *out = b3_compound_child_hull(local_position, local_orientation, shape->as.hull);
            return 0;
        case B3_SHAPE_BOX:
            *out = b3_compound_child_box(local_position, local_orientation, shape->as.box);
            return 0;
        case B3_SHAPE_COMPOUND:
        default:
            PyErr_SetString(
                PyExc_TypeError,
                "Compound children cannot themselves be a Compound (no nesting -- see "
                "box3d/compound.h)"
            );
            return -1;
    }
}

/* Parses one child entry: (local_position, shape) or
 * (local_position, local_orientation, shape). */
static int parse_child_entry(PyObject *entry, b3_CompoundChild *out) {
    PyObject *seq = PySequence_Fast(entry, "each child must be a (local_position, shape) or "
                                            "(local_position, local_orientation, shape) sequence");
    if (seq == NULL) {
        return -1;
    }
    Py_ssize_t n = PySequence_Fast_GET_SIZE(seq);
    if (n != 2 && n != 3) {
        Py_DECREF(seq);
        PyErr_SetString(
            PyExc_ValueError,
            "each child must be a 2-tuple (local_position, shape) or a 3-tuple "
            "(local_position, local_orientation, shape)"
        );
        return -1;
    }

    b3_Vec3 local_position;
    if (PyVec3_Parse(PySequence_Fast_GET_ITEM(seq, 0), &local_position) < 0) {
        Py_DECREF(seq);
        return -1;
    }

    b3_Quat local_orientation = b3_quat_identity();
    PyObject *shape_obj = PySequence_Fast_GET_ITEM(seq, n - 1);
    if (n == 3) {
        PyObject *orientation_obj = PySequence_Fast_GET_ITEM(seq, 1);
        if (orientation_obj != Py_None && PyQuat_Parse(orientation_obj, &local_orientation) < 0) {
            Py_DECREF(seq);
            return -1;
        }
    }

    b3_Shape shape;
    if (PyShape_Parse(shape_obj, &shape) < 0) {
        Py_DECREF(seq);
        return -1;
    }
    Py_DECREF(seq);

    return shape_to_compound_child(local_position, local_orientation, &shape, out);
}

int PyCompound_ParseChildren(
    PyObject *children_obj, b3_CompoundChild out[B3_COMPOUND_MAX_CHILDREN], int *out_count
) {
    PyObject *seq = PySequence_Fast(children_obj, "children must be a sequence");
    if (seq == NULL) {
        return -1;
    }
    Py_ssize_t n = PySequence_Fast_GET_SIZE(seq);
    if (n > B3_COMPOUND_MAX_CHILDREN) {
        Py_DECREF(seq);
        PyErr_Format(
            PyExc_ValueError, "Compound supports at most %d children, got %zd",
            B3_COMPOUND_MAX_CHILDREN, n
        );
        return -1;
    }

    for (Py_ssize_t i = 0; i < n; i++) {
        if (parse_child_entry(PySequence_Fast_GET_ITEM(seq, i), &out[i]) < 0) {
            Py_DECREF(seq);
            return -1;
        }
    }
    Py_DECREF(seq);
    *out_count = (int)n;
    return 0;
}

static PyObject *compound_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"center", "children", "orientation", NULL};
    PyObject *center_obj, *children_obj;
    PyObject *orientation_obj = Py_None;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "OO|O", kwlist, &center_obj, &children_obj, &orientation_obj
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

    b3_CompoundChild children[B3_COMPOUND_MAX_CHILDREN];
    int child_count;
    if (PyCompound_ParseChildren(children_obj, children, &child_count) < 0) {
        return NULL;
    }

    PyCompoundObject *self = (PyCompoundObject *)type->tp_alloc(type, 0);
    if (self == NULL) {
        return NULL;
    }
    self->value = b3_compound_make(center, orientation, children, child_count);
    return (PyObject *)self;
}

static void compound_dealloc(PyCompoundObject *self) {
    Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyObject *compound_repr(PyCompoundObject *self) {
    return PyUnicode_FromFormat(
        "Compound(center=(%g, %g, %g), child_count=%d)",
        (double)self->value.center.x, (double)self->value.center.y, (double)self->value.center.z,
        self->value.child_count
    );
}

static PyObject *compound_get_center(PyCompoundObject *self, void *closure) {
    (void)closure;
    return PyVec3_FromVec3(self->value.center);
}
static int compound_set_center(PyCompoundObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete center");
        return -1;
    }
    return PyVec3_Parse(value, &self->value.center);
}

static PyObject *compound_get_orientation(PyCompoundObject *self, void *closure) {
    (void)closure;
    return PyQuat_FromQuat(self->value.orientation);
}
static int compound_set_orientation(PyCompoundObject *self, PyObject *value, void *closure) {
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

static PyObject *compound_get_child_count(PyCompoundObject *self, void *closure) {
    (void)closure;
    return PyLong_FromLong(self->value.child_count);
}

static PyGetSetDef compound_getset[] = {
    {"center", (getter)compound_get_center, (setter)compound_set_center, "world-space center (Vec3)", NULL},
    {"orientation", (getter)compound_get_orientation, (setter)compound_set_orientation, "orientation (Quat)", NULL},
    {"child_count", (getter)compound_get_child_count, NULL, "number of children (read-only)", NULL},
    {NULL},
};

static PyObject *compound_contains_point(PyCompoundObject *self, PyObject *arg) {
    b3_Vec3 point;
    if (PyVec3_Parse(arg, &point) < 0) {
        return NULL;
    }
    if (b3_compound_contains_point(&self->value, point)) {
        Py_RETURN_TRUE;
    }
    Py_RETURN_FALSE;
}

static PyObject *compound_aabb(PyCompoundObject *self, PyObject *Py_UNUSED(ignored)) {
    b3_Vec3 lo, hi;
    b3_compound_compute_aabb(&self->value, &lo, &hi);
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

static PyObject *compound_overlaps(PyCompoundObject *self, PyObject *arg) {
    b3_Shape self_shape = b3_shape_from_compound(self->value);
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

static PyObject *compound_raycast(PyCompoundObject *self, PyObject *args, PyObject *kwds) {
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

    b3_RayHit rayhit = b3_compound_raycast(&self->value, origin, direction, (b3_real)max_t);
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

static PyMethodDef compound_methods[] = {
    {"contains_point", (PyCFunction)compound_contains_point, METH_O, "contains_point(point) -> bool"},
    {"aabb", (PyCFunction)compound_aabb, METH_NOARGS, "aabb() -> (Vec3 min, Vec3 max)"},
    {"overlaps", (PyCFunction)compound_overlaps, METH_O,
     "overlaps(other: Box3D | Sphere | Capsule | ConvexHull | Compound) -> ContactInfo | None"},
    {"raycast", (PyCFunction)compound_raycast, METH_VARARGS | METH_KEYWORDS,
     "raycast(origin, direction, max_t=inf) -> RayHit | None (closest hit among all children)"},
    {NULL},
};

PyTypeObject PyCompound_Type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "pybox3d.Compound",
    .tp_basicsize = sizeof(PyCompoundObject),
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = PyDoc_STR(
        "Compound(center, children, orientation=None)\n\n"
        "Multiple leaf shapes (Box3D/Sphere/Capsule/ConvexHull -- no nested "
        "Compound) rigidly attached at fixed local offsets. `children` is a "
        "sequence of (local_position, shape) or "
        "(local_position, local_orientation, shape) entries, at most 8."
    ),
    .tp_new = compound_new,
    .tp_dealloc = (destructor)compound_dealloc,
    .tp_repr = (reprfunc)compound_repr,
    .tp_getset = compound_getset,
    .tp_methods = compound_methods,
};
