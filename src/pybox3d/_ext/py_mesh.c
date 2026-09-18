#include "py_mesh.h"
#include "py_vec3.h"
#include "py_quat.h"
#include "py_box3d.h"
#include "py_shape.h"

#include <float.h>

PyObject *PyTriangleMesh_FromMesh(b3_TriangleMesh mesh) {
    PyTriangleMeshObject *obj = PyObject_New(PyTriangleMeshObject, &PyTriangleMesh_Type);
    if (obj == NULL) {
        return NULL;
    }
    obj->value = mesh;
    return (PyObject *)obj;
}

/* Parses `triangles_obj` (a sequence of 3-Vec3-like sequences) into a
 * flat array of 3*count Vec3s. Returns the triangle count, or -1 with an
 * exception set. */
static Py_ssize_t parse_triangles(PyObject *triangles_obj, b3_Vec3 out[B3_MESH_MAX_TRIANGLES * 3]) {
    PyObject *seq = PySequence_Fast(triangles_obj, "triangles must be a sequence of 3-point sequences");
    if (seq == NULL) {
        return -1;
    }
    Py_ssize_t n = PySequence_Fast_GET_SIZE(seq);
    if (n > B3_MESH_MAX_TRIANGLES) {
        Py_DECREF(seq);
        PyErr_Format(
            PyExc_ValueError, "TriangleMesh supports at most %d triangles, got %zd",
            B3_MESH_MAX_TRIANGLES, n
        );
        return -1;
    }

    for (Py_ssize_t i = 0; i < n; i++) {
        PyObject *tri_obj = PySequence_Fast_GET_ITEM(seq, i);
        PyObject *tri_seq = PySequence_Fast(tri_obj, "each triangle must be a 3-point sequence");
        if (tri_seq == NULL) {
            Py_DECREF(seq);
            return -1;
        }
        if (PySequence_Fast_GET_SIZE(tri_seq) != 3) {
            Py_DECREF(tri_seq);
            Py_DECREF(seq);
            PyErr_SetString(PyExc_ValueError, "each triangle must have exactly 3 points");
            return -1;
        }
        for (int k = 0; k < 3; k++) {
            if (PyVec3_Parse(PySequence_Fast_GET_ITEM(tri_seq, k), &out[i * 3 + k]) < 0) {
                Py_DECREF(tri_seq);
                Py_DECREF(seq);
                return -1;
            }
        }
        Py_DECREF(tri_seq);
    }
    Py_DECREF(seq);
    return n;
}

static PyObject *mesh_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    static char *kwlist[] = {"center", "triangles", "orientation", NULL};
    PyObject *center_obj, *triangles_obj;
    PyObject *orientation_obj = Py_None;
    if (!PyArg_ParseTupleAndKeywords(
            args, kwds, "OO|O", kwlist, &center_obj, &triangles_obj, &orientation_obj
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

    b3_Vec3 local_vertices[B3_MESH_MAX_TRIANGLES * 3];
    Py_ssize_t n = parse_triangles(triangles_obj, local_vertices);
    if (n < 0) {
        return NULL;
    }

    PyTriangleMeshObject *self = (PyTriangleMeshObject *)type->tp_alloc(type, 0);
    if (self == NULL) {
        return NULL;
    }
    self->value = b3_trianglemesh_make(center, orientation, local_vertices, (int)n);
    return (PyObject *)self;
}

static void mesh_dealloc(PyTriangleMeshObject *self) {
    Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyObject *mesh_repr(PyTriangleMeshObject *self) {
    return PyUnicode_FromFormat(
        "TriangleMesh(center=(%g, %g, %g), triangle_count=%d)",
        (double)self->value.center.x, (double)self->value.center.y, (double)self->value.center.z,
        self->value.triangle_count
    );
}

static PyObject *mesh_get_center(PyTriangleMeshObject *self, void *closure) {
    (void)closure;
    return PyVec3_FromVec3(self->value.center);
}
static int mesh_set_center(PyTriangleMeshObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete center");
        return -1;
    }
    return PyVec3_Parse(value, &self->value.center);
}

static PyObject *mesh_get_orientation(PyTriangleMeshObject *self, void *closure) {
    (void)closure;
    return PyQuat_FromQuat(self->value.orientation);
}
static int mesh_set_orientation(PyTriangleMeshObject *self, PyObject *value, void *closure) {
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

static PyObject *mesh_get_triangle_count(PyTriangleMeshObject *self, void *closure) {
    (void)closure;
    return PyLong_FromLong(self->value.triangle_count);
}

static PyGetSetDef mesh_getset[] = {
    {"center", (getter)mesh_get_center, (setter)mesh_set_center, "world-space center (Vec3)", NULL},
    {"orientation", (getter)mesh_get_orientation, (setter)mesh_set_orientation, "orientation (Quat)", NULL},
    {"triangle_count", (getter)mesh_get_triangle_count, NULL, "number of triangles (read-only)", NULL},
    {NULL},
};

static PyObject *mesh_contains_point(PyTriangleMeshObject *self, PyObject *arg) {
    b3_Vec3 point;
    if (PyVec3_Parse(arg, &point) < 0) {
        return NULL;
    }
    if (b3_trianglemesh_contains_point(&self->value, point)) {
        Py_RETURN_TRUE;
    }
    Py_RETURN_FALSE;
}

static PyObject *mesh_aabb(PyTriangleMeshObject *self, PyObject *Py_UNUSED(ignored)) {
    b3_Vec3 lo, hi;
    b3_trianglemesh_compute_aabb(&self->value, &lo, &hi);
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

static PyObject *mesh_overlaps(PyTriangleMeshObject *self, PyObject *arg) {
    b3_Shape self_shape = b3_shape_from_mesh(self->value);
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

static PyObject *mesh_raycast(PyTriangleMeshObject *self, PyObject *args, PyObject *kwds) {
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

    b3_RayHit rayhit = b3_trianglemesh_raycast(&self->value, origin, direction, (b3_real)max_t);
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

static PyMethodDef mesh_methods[] = {
    {"contains_point", (PyCFunction)mesh_contains_point, METH_O,
     "contains_point(point) -> bool (v1: always False, see docs/limitations.md)"},
    {"aabb", (PyCFunction)mesh_aabb, METH_NOARGS, "aabb() -> (Vec3 min, Vec3 max)"},
    {"overlaps", (PyCFunction)mesh_overlaps, METH_O,
     "overlaps(other: Box3D | Sphere | Capsule | ConvexHull | Compound) -> ContactInfo | None"},
    {"raycast", (PyCFunction)mesh_raycast, METH_VARARGS | METH_KEYWORDS,
     "raycast(origin, direction, max_t=inf) -> RayHit | None (v1: approximated via the mesh's "
     "AABB, see docs/limitations.md)"},
    {NULL},
};

PyTypeObject PyTriangleMesh_Type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "pybox3d.TriangleMesh",
    .tp_basicsize = sizeof(PyTriangleMeshObject),
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = PyDoc_STR(
        "TriangleMesh(center, triangles, orientation=None)\n\n"
        "A static-only triangle soup: `triangles` is a sequence of "
        "3-point sequences (local/unrotated/uncentered relative to `center`), "
        "at most 64 triangles. Not a shape RigidBody.shape returns as dynamic "
        "-- only static bodies (mass=0) can use it, see RigidBody.mesh()."
    ),
    .tp_new = mesh_new,
    .tp_dealloc = (destructor)mesh_dealloc,
    .tp_repr = (reprfunc)mesh_repr,
    .tp_getset = mesh_getset,
    .tp_methods = mesh_methods,
};
