#include "py_box3d.h"
#include "py_vec3.h"
#include "py_quat.h"

#include <float.h>

PyTypeObject *PyContactInfo_Type = NULL;
PyTypeObject *PyRayHit_Type = NULL;

PyObject *PyBox3D_FromBox3D(b3_Box3D box) {
    PyBox3DObject *obj = PyObject_New(PyBox3DObject, &PyBox3D_Type);
    if (obj == NULL) {
        return NULL;
    }
    obj->value = box;
    return (PyObject *)obj;
}

static PyObject *box3d_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"center", "half_extents", "orientation", NULL};
    PyObject *center_obj, *half_extents_obj;
    PyObject *orientation_obj = Py_None;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "OO|O", kwlist, &center_obj, &half_extents_obj, &orientation_obj
        )) {
        return NULL;
    }

    b3_Vec3 center, half_extents;
    if (PyVec3_Parse(center_obj, &center) < 0) {
        return NULL;
    }
    if (PyVec3_Parse(half_extents_obj, &half_extents) < 0) {
        return NULL;
    }

    b3_Box3D value;
    if (orientation_obj == Py_None) {
        value = b3_box3d_make_aabb(center, half_extents);
    } else {
        b3_Quat orientation;
        if (PyQuat_Parse(orientation_obj, &orientation) < 0) {
            return NULL;
        }
        value = b3_box3d_make_obb(center, half_extents, orientation);
    }

    PyBox3DObject *self = (PyBox3DObject *)type->tp_alloc(type, 0);
    if (self == NULL) {
        return NULL;
    }
    self->value = value;
    return (PyObject *)self;
}

static void box3d_dealloc(PyBox3DObject *self) {
    Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyObject *box3d_repr(PyBox3DObject *self) {
    const char *kind = self->value.kind == B3_BOX_KIND_OBB ? "OBB" : "AABB";
    return PyUnicode_FromFormat(
        "Box3D(kind=%s, center=(%g, %g, %g), half_extents=(%g, %g, %g))",
        kind,
        (double)self->value.center.x, (double)self->value.center.y, (double)self->value.center.z,
        (double)self->value.half_extents.x, (double)self->value.half_extents.y, (double)self->value.half_extents.z
    );
}

static PyObject *box3d_get_center(PyBox3DObject *self, void *closure) {
    (void)closure;
    return PyVec3_FromVec3(self->value.center);
}
static int box3d_set_center(PyBox3DObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete center");
        return -1;
    }
    return PyVec3_Parse(value, &self->value.center);
}

static PyObject *box3d_get_half_extents(PyBox3DObject *self, void *closure) {
    (void)closure;
    return PyVec3_FromVec3(self->value.half_extents);
}
static int box3d_set_half_extents(PyBox3DObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete half_extents");
        return -1;
    }
    return PyVec3_Parse(value, &self->value.half_extents);
}

static PyObject *box3d_get_orientation(PyBox3DObject *self, void *closure) {
    (void)closure;
    return PyQuat_FromQuat(self->value.orientation);
}
static int box3d_set_orientation(PyBox3DObject *self, PyObject *value, void *closure) {
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
    self->value.kind = B3_BOX_KIND_OBB;
    return 0;
}

static PyObject *box3d_get_kind(PyBox3DObject *self, void *closure) {
    (void)closure;
    return PyLong_FromLong((long)self->value.kind);
}

static PyGetSetDef box3d_getset[] = {
    {"center", (getter)box3d_get_center, (setter)box3d_set_center, "center point (Vec3)", NULL},
    {"half_extents", (getter)box3d_get_half_extents, (setter)box3d_set_half_extents, "half-extents along each local axis (Vec3)", NULL},
    {"orientation", (getter)box3d_get_orientation, (setter)box3d_set_orientation, "orientation quaternion (Quat)", NULL},
    {"kind", (getter)box3d_get_kind, NULL, "BOX_KIND_AABB or BOX_KIND_OBB", NULL},
    {NULL},
};

static PyObject *box3d_contains_point(PyBox3DObject *self, PyObject *arg) {
    b3_Vec3 point;
    if (PyVec3_Parse(arg, &point) < 0) {
        return NULL;
    }
    if (b3_box3d_contains_point(&self->value, point)) {
        Py_RETURN_TRUE;
    }
    Py_RETURN_FALSE;
}

static PyObject *box3d_aabb(PyBox3DObject *self, PyObject *Py_UNUSED(ignored)) {
    b3_Vec3 lo, hi;
    b3_box3d_compute_aabb(&self->value, &lo, &hi);
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

static PyObject *box3d_overlaps(PyBox3DObject *self, PyObject *arg) {
    if (!PyBox3D_Check(arg)) {
        PyErr_SetString(PyExc_TypeError, "overlaps() argument must be a Box3D");
        return NULL;
    }
    b3_ContactInfo contact;
    int hit = b3_box3d_overlap(&self->value, &((PyBox3DObject *)arg)->value, &contact);
    if (!hit) {
        Py_RETURN_NONE;
    }

    PyObject *result = PyStructSequence_New(PyContactInfo_Type);
    if (result == NULL) {
        return NULL;
    }
    PyObject *normal = PyVec3_FromVec3(contact.normal);
    if (normal == NULL) {
        Py_DECREF(result);
        return NULL;
    }
    PyObject *penetration = PyFloat_FromDouble((double)contact.penetration);
    if (penetration == NULL) {
        Py_DECREF(normal);
        Py_DECREF(result);
        return NULL;
    }
    PyStructSequence_SET_ITEM(result, 0, normal);
    PyStructSequence_SET_ITEM(result, 1, penetration);
    return result;
}

static PyObject *box3d_raycast(PyBox3DObject *self, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"origin", "direction", "max_t", NULL};
    PyObject *origin_obj, *direction_obj;
    double max_t = DBL_MAX;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "OO|d", kwlist, &origin_obj, &direction_obj, &max_t
        )) {
        return NULL;
    }
    b3_Vec3 origin, direction;
    if (PyVec3_Parse(origin_obj, &origin) < 0) {
        return NULL;
    }
    if (PyVec3_Parse(direction_obj, &direction) < 0) {
        return NULL;
    }

    b3_RayHit rayhit = b3_box3d_raycast(&self->value, origin, direction, (b3_real)max_t);
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

static PyMethodDef box3d_methods[] = {
    {"contains_point", (PyCFunction)box3d_contains_point, METH_O, "contains_point(point) -> bool"},
    {"aabb", (PyCFunction)box3d_aabb, METH_NOARGS, "aabb() -> (Vec3 min, Vec3 max)"},
    {"overlaps", (PyCFunction)box3d_overlaps, METH_O, "overlaps(other: Box3D) -> ContactInfo | None"},
    {"raycast", (PyCFunction)box3d_raycast, METH_VARARGS | METH_KEYWORDS,
     "raycast(origin, direction, max_t=inf) -> RayHit | None"},
    {NULL},
};

PyTypeObject PyBox3D_Type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "pybox3d.Box3D",
    .tp_basicsize = sizeof(PyBox3DObject),
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = PyDoc_STR(
        "Box3D(center, half_extents, orientation=None)\n\n"
        "A 3D box: an AABB when orientation is None, an OBB otherwise."
    ),
    .tp_new = box3d_new,
    .tp_dealloc = (destructor)box3d_dealloc,
    .tp_repr = (reprfunc)box3d_repr,
    .tp_getset = box3d_getset,
    .tp_methods = box3d_methods,
};

static PyStructSequence_Field contact_info_fields[] = {
    {"normal", "unit-length contact normal, pointing from the first box towards the second"},
    {"penetration", "penetration depth along `normal`, in world units (>= 0)"},
    {NULL},
};
static PyStructSequence_Desc contact_info_desc = {
    "pybox3d.ContactInfo",
    "Result of Box3D.overlaps(): overlap normal and penetration depth.",
    contact_info_fields,
    2,
};

static PyStructSequence_Field ray_hit_fields[] = {
    {"t", "ray parameter at the hit point (point = origin + t * direction)"},
    {"point", "world-space hit point (Vec3)"},
    {"normal", "surface normal at the hit point (Vec3)"},
    {NULL},
};
static PyStructSequence_Desc ray_hit_desc = {
    "pybox3d.RayHit",
    "Result of Box3D.raycast(): hit parameter, point, and surface normal.",
    ray_hit_fields,
    3,
};

int pybox3d_box3d_module_init(PyObject *module) {
    if (PyType_Ready(&PyBox3D_Type) < 0) {
        return -1;
    }
    if (PyModule_AddObjectRef(module, "Box3D", (PyObject *)&PyBox3D_Type) < 0) {
        return -1;
    }

    PyContactInfo_Type = PyStructSequence_NewType(&contact_info_desc);
    if (PyContactInfo_Type == NULL) {
        return -1;
    }
    if (PyModule_AddObjectRef(module, "ContactInfo", (PyObject *)PyContactInfo_Type) < 0) {
        return -1;
    }

    PyRayHit_Type = PyStructSequence_NewType(&ray_hit_desc);
    if (PyRayHit_Type == NULL) {
        return -1;
    }
    if (PyModule_AddObjectRef(module, "RayHit", (PyObject *)PyRayHit_Type) < 0) {
        return -1;
    }

    if (PyModule_AddIntConstant(module, "BOX_KIND_AABB", B3_BOX_KIND_AABB) < 0) {
        return -1;
    }
    if (PyModule_AddIntConstant(module, "BOX_KIND_OBB", B3_BOX_KIND_OBB) < 0) {
        return -1;
    }

    return 0;
}
