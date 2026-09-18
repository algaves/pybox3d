#include "py_sphere.h"
#include "py_vec3.h"
#include "py_box3d.h"
#include "py_shape.h"

#include <float.h>

PyObject *PySphere_FromSphere(b3_Sphere sphere) {
    PySphereObject *obj = PyObject_New(PySphereObject, &PySphere_Type);
    if (obj == NULL) {
        return NULL;
    }
    obj->value = sphere;
    return (PyObject *)obj;
}

static PyObject *sphere_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"center", "radius", NULL};
    PyObject *center_obj;
    double radius;
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "Od", kwlist, &center_obj, &radius)) {
        return NULL;
    }
    b3_Vec3 center;
    if (PyVec3_Parse(center_obj, &center) < 0) {
        return NULL;
    }

    PySphereObject *self = (PySphereObject *)type->tp_alloc(type, 0);
    if (self == NULL) {
        return NULL;
    }
    self->value = b3_sphere_make(center, (b3_real)radius);
    return (PyObject *)self;
}

static void sphere_dealloc(PySphereObject *self) {
    Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyObject *sphere_repr(PySphereObject *self) {
    return PyUnicode_FromFormat(
        "Sphere(center=(%g, %g, %g), radius=%g)",
        (double)self->value.center.x, (double)self->value.center.y, (double)self->value.center.z,
        (double)self->value.radius
    );
}

static PyObject *sphere_get_center(PySphereObject *self, void *closure) {
    (void)closure;
    return PyVec3_FromVec3(self->value.center);
}
static int sphere_set_center(PySphereObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete center");
        return -1;
    }
    return PyVec3_Parse(value, &self->value.center);
}

static PyObject *sphere_get_radius(PySphereObject *self, void *closure) {
    (void)closure;
    return PyFloat_FromDouble((double)self->value.radius);
}
static int sphere_set_radius(PySphereObject *self, PyObject *value, void *closure) {
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

static PyGetSetDef sphere_getset[] = {
    {"center", (getter)sphere_get_center, (setter)sphere_set_center, "center point (Vec3)", NULL},
    {"radius", (getter)sphere_get_radius, (setter)sphere_set_radius, "radius", NULL},
    {NULL},
};

static PyObject *sphere_contains_point(PySphereObject *self, PyObject *arg) {
    b3_Vec3 point;
    if (PyVec3_Parse(arg, &point) < 0) {
        return NULL;
    }
    if (b3_sphere_contains_point(&self->value, point)) {
        Py_RETURN_TRUE;
    }
    Py_RETURN_FALSE;
}

static PyObject *sphere_aabb(PySphereObject *self, PyObject *Py_UNUSED(ignored)) {
    b3_Vec3 lo, hi;
    b3_sphere_compute_aabb(&self->value, &lo, &hi);
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

static PyObject *sphere_overlaps(PySphereObject *self, PyObject *arg) {
    b3_Shape self_shape = b3_shape_from_sphere(self->value);
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

static PyObject *sphere_raycast(PySphereObject *self, PyObject *args, PyObject *kwds) {
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

    b3_RayHit rayhit = b3_sphere_raycast(&self->value, origin, direction, (b3_real)max_t);
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

static PyMethodDef sphere_methods[] = {
    {"contains_point", (PyCFunction)sphere_contains_point, METH_O, "contains_point(point) -> bool"},
    {"aabb", (PyCFunction)sphere_aabb, METH_NOARGS, "aabb() -> (Vec3 min, Vec3 max)"},
    {"overlaps", (PyCFunction)sphere_overlaps, METH_O,
     "overlaps(other: Box3D | Sphere | Capsule) -> ContactInfo | None"},
    {"raycast", (PyCFunction)sphere_raycast, METH_VARARGS | METH_KEYWORDS,
     "raycast(origin, direction, max_t=inf) -> RayHit | None"},
    {NULL},
};

PyTypeObject PySphere_Type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "pybox3d.Sphere",
    .tp_basicsize = sizeof(PySphereObject),
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = PyDoc_STR("Sphere(center, radius)\n\nA 3D sphere shape."),
    .tp_new = sphere_new,
    .tp_dealloc = (destructor)sphere_dealloc,
    .tp_repr = (reprfunc)sphere_repr,
    .tp_getset = sphere_getset,
    .tp_methods = sphere_methods,
};
