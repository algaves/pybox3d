#include "py_capsule.h"
#include "py_vec3.h"
#include "py_quat.h"
#include "py_box3d.h"
#include "py_shape.h"

#include <float.h>

PyObject *PyCapsule3D_FromCapsule(b3_Capsule capsule) {
    PyCapsuleObject *obj = PyObject_New(PyCapsuleObject, &PyCapsule3D_Type);
    if (obj == NULL) {
        return NULL;
    }
    obj->value = capsule;
    return (PyObject *)obj;
}

static PyObject *capsule_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"center", "radius", "half_height", "orientation", NULL};
    PyObject *center_obj;
    double radius, half_height;
    PyObject *orientation_obj = Py_None;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "Odd|O", kwlist, &center_obj, &radius, &half_height, &orientation_obj
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

    PyCapsuleObject *self = (PyCapsuleObject *)type->tp_alloc(type, 0);
    if (self == NULL) {
        return NULL;
    }
    self->value = b3_capsule_make(center, orientation, (b3_real)radius, (b3_real)half_height);
    return (PyObject *)self;
}

static void capsule_dealloc(PyCapsuleObject *self) {
    Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyObject *capsule_repr(PyCapsuleObject *self) {
    return PyUnicode_FromFormat(
        "Capsule(center=(%g, %g, %g), radius=%g, half_height=%g)",
        (double)self->value.center.x, (double)self->value.center.y, (double)self->value.center.z,
        (double)self->value.radius, (double)self->value.half_height
    );
}

static PyObject *capsule_get_center(PyCapsuleObject *self, void *closure) {
    (void)closure;
    return PyVec3_FromVec3(self->value.center);
}
static int capsule_set_center(PyCapsuleObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete center");
        return -1;
    }
    return PyVec3_Parse(value, &self->value.center);
}

static PyObject *capsule_get_orientation(PyCapsuleObject *self, void *closure) {
    (void)closure;
    return PyQuat_FromQuat(self->value.orientation);
}
static int capsule_set_orientation(PyCapsuleObject *self, PyObject *value, void *closure) {
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

static PyObject *capsule_get_radius(PyCapsuleObject *self, void *closure) {
    (void)closure;
    return PyFloat_FromDouble((double)self->value.radius);
}
static int capsule_set_radius(PyCapsuleObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete radius");
        return -1;
    }
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    self->value.radius = (b3_real)v;
    return 0;
}

static PyObject *capsule_get_half_height(PyCapsuleObject *self, void *closure) {
    (void)closure;
    return PyFloat_FromDouble((double)self->value.half_height);
}
static int capsule_set_half_height(PyCapsuleObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete half_height");
        return -1;
    }
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    self->value.half_height = (b3_real)v;
    return 0;
}

static PyGetSetDef capsule_getset[] = {
    {"center", (getter)capsule_get_center, (setter)capsule_set_center, "center point (Vec3)", NULL},
    {"orientation", (getter)capsule_get_orientation, (setter)capsule_set_orientation,
     "orientation quaternion; the capsule's segment runs along its local +Y axis", NULL},
    {"radius", (getter)capsule_get_radius, (setter)capsule_set_radius, "radius", NULL},
    {"half_height", (getter)capsule_get_half_height, (setter)capsule_set_half_height,
     "half-length of the inner segment (not including the two hemispherical caps)", NULL},
    {NULL},
};

static PyObject *capsule_contains_point(PyCapsuleObject *self, PyObject *arg) {
    b3_Vec3 point;
    if (PyVec3_Parse(arg, &point) < 0) {
        return NULL;
    }
    if (b3_capsule_contains_point(&self->value, point)) {
        Py_RETURN_TRUE;
    }
    Py_RETURN_FALSE;
}

static PyObject *capsule_aabb(PyCapsuleObject *self, PyObject *Py_UNUSED(ignored)) {
    b3_Vec3 lo, hi;
    b3_capsule_compute_aabb(&self->value, &lo, &hi);
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

static PyObject *capsule_overlaps(PyCapsuleObject *self, PyObject *arg) {
    b3_Shape self_shape = b3_shape_from_capsule(self->value);
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

static PyObject *capsule_raycast(PyCapsuleObject *self, PyObject *args, PyObject *kwds) {
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

    b3_RayHit rayhit = b3_capsule_raycast(&self->value, origin, direction, (b3_real)max_t);
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

static PyMethodDef capsule_methods[] = {
    {"contains_point", (PyCFunction)capsule_contains_point, METH_O, "contains_point(point) -> bool"},
    {"aabb", (PyCFunction)capsule_aabb, METH_NOARGS, "aabb() -> (Vec3 min, Vec3 max)"},
    {"overlaps", (PyCFunction)capsule_overlaps, METH_O,
     "overlaps(other: Box3D | Sphere | Capsule) -> ContactInfo | None"},
    {"raycast", (PyCFunction)capsule_raycast, METH_VARARGS | METH_KEYWORDS,
     "raycast(origin, direction, max_t=inf) -> RayHit | None"},
    {NULL},
};

PyTypeObject PyCapsule3D_Type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "pybox3d.Capsule",
    .tp_basicsize = sizeof(PyCapsuleObject),
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = PyDoc_STR(
        "Capsule(center, radius, half_height, orientation=None)\n\n"
        "A 3D capsule: a line segment of length 2*half_height along the "
        "local +Y axis (rotated by orientation), swept by radius."
    ),
    .tp_new = capsule_new,
    .tp_dealloc = (destructor)capsule_dealloc,
    .tp_repr = (reprfunc)capsule_repr,
    .tp_getset = capsule_getset,
    .tp_methods = capsule_methods,
};
