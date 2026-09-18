#include "py_shape.h"
#include "py_box3d.h"
#include "py_sphere.h"
#include "py_capsule.h"
#include "py_hull.h"
#include "py_compound.h"
#include "py_mesh.h"
#include "py_heightfield.h"

int PyShape_Parse(PyObject *obj, b3_Shape *out) {
    if (PyBox3D_Check(obj)) {
        *out = b3_shape_from_box(((PyBox3DObject *)obj)->value);
        return 0;
    }
    if (PySphere_Check(obj)) {
        *out = b3_shape_from_sphere(((PySphereObject *)obj)->value);
        return 0;
    }
    if (PyCapsule3D_Check(obj)) {
        *out = b3_shape_from_capsule(((PyCapsuleObject *)obj)->value);
        return 0;
    }
    if (PyConvexHull_Check(obj)) {
        *out = b3_shape_from_hull(((PyConvexHullObject *)obj)->value);
        return 0;
    }
    if (PyCompound_Check(obj)) {
        *out = b3_shape_from_compound(((PyCompoundObject *)obj)->value);
        return 0;
    }
    if (PyTriangleMesh_Check(obj)) {
        *out = b3_shape_from_mesh(((PyTriangleMeshObject *)obj)->value);
        return 0;
    }
    if (PyHeightField_Check(obj)) {
        *out = b3_shape_from_heightfield(((PyHeightFieldObject *)obj)->value);
        return 0;
    }
    PyErr_SetString(
        PyExc_TypeError,
        "expected a Box3D, Sphere, Capsule, ConvexHull, Compound, TriangleMesh, or HeightField"
    );
    return -1;
}

PyObject *PyShape_Wrap(b3_Shape shape) {
    switch (shape.kind) {
        case B3_SHAPE_SPHERE:
            return PySphere_FromSphere(shape.as.sphere);
        case B3_SHAPE_CAPSULE:
            return PyCapsule3D_FromCapsule(shape.as.capsule);
        case B3_SHAPE_HULL:
            return PyConvexHull_FromHull(shape.as.hull);
        case B3_SHAPE_COMPOUND:
            return PyCompound_FromCompound(shape.as.compound);
        case B3_SHAPE_MESH:
            return PyTriangleMesh_FromMesh(shape.as.mesh);
        case B3_SHAPE_HEIGHTFIELD:
            return PyHeightField_FromHeightField(shape.as.heightfield);
        case B3_SHAPE_BOX:
        default:
            return PyBox3D_FromBox3D(shape.as.box);
    }
}
