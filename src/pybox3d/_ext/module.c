#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include "py_errors.h"
#include "py_vec3.h"
#include "py_quat.h"
#include "py_box3d.h"
#include "py_sphere.h"
#include "py_capsule.h"
#include "py_hull.h"
#include "py_compound.h"
#include "py_mesh.h"
#include "py_heightfield.h"
#include "py_rigidbody.h"
#include "py_joint.h"
#include "py_world.h"
#include "py_worldsnapshot.h"
#include "py_character.h"

#define PYBOX3D_VERSION "2026b0"

static PyModuleDef pybox3d_module = {
    PyModuleDef_HEAD_INIT,
    .m_name = "pybox3d._pybox3d",
    .m_doc = PyDoc_STR("CPython C-API extension backing the pybox3d package."),
    .m_size = -1,
};

static int add_type(PyObject *module, PyTypeObject *type, const char *name) {
    if (PyType_Ready(type) < 0) {
        return -1;
    }
    return PyModule_AddObjectRef(module, name, (PyObject *)type);
}

PyMODINIT_FUNC PyInit__pybox3d(void) {
    PyObject *module = PyModule_Create(&pybox3d_module);
    if (module == NULL) {
        return NULL;
    }

    if (add_type(module, &PyVec3_Type, "Vec3") < 0) goto fail;
    if (add_type(module, &PyQuat_Type, "Quat") < 0) goto fail;
    if (add_type(module, &PyRigidBody_Type, "RigidBody") < 0) goto fail;
    if (add_type(module, &PyDistanceJoint_Type, "DistanceJoint") < 0) goto fail;
    if (add_type(module, &PyJoint_Type, "Joint") < 0) goto fail;
    if (add_type(module, &PyWorld_Type, "World") < 0) goto fail;
    if (add_type(module, &PyWorldSnapshot_Type, "WorldSnapshot") < 0) goto fail;
    if (add_type(module, &PyCharacterMover_Type, "CharacterMover") < 0) goto fail;

    /* Box3D + ContactInfo/RayHit struct-sequence types + BOX_KIND_* constants. */
    if (pybox3d_box3d_module_init(module) < 0) goto fail;

    if (add_type(module, &PySphere_Type, "Sphere") < 0) goto fail;
    if (add_type(module, &PyCapsule3D_Type, "Capsule") < 0) goto fail;
    if (add_type(module, &PyConvexHull_Type, "ConvexHull") < 0) goto fail;
    if (add_type(module, &PyCompound_Type, "Compound") < 0) goto fail;
    if (add_type(module, &PyTriangleMesh_Type, "TriangleMesh") < 0) goto fail;
    if (add_type(module, &PyHeightField_Type, "HeightField") < 0) goto fail;

    if (PyModule_AddIntConstant(module, "HULL_MAX_VERTICES", B3_HULL_MAX_VERTICES) < 0) goto fail;
    if (PyModule_AddIntConstant(module, "COMPOUND_MAX_CHILDREN", B3_COMPOUND_MAX_CHILDREN) < 0) goto fail;
    if (PyModule_AddIntConstant(module, "MESH_MAX_TRIANGLES", B3_MESH_MAX_TRIANGLES) < 0) goto fail;
    if (PyModule_AddIntConstant(module, "HEIGHTFIELD_MAX_ROWS", B3_HEIGHTFIELD_MAX_ROWS) < 0) goto fail;
    if (PyModule_AddIntConstant(module, "HEIGHTFIELD_MAX_COLS", B3_HEIGHTFIELD_MAX_COLS) < 0) goto fail;

    if (pybox3d_errors_init(module) < 0) goto fail;

    if (PyModule_AddStringConstant(module, "__version__", PYBOX3D_VERSION) < 0) goto fail;

    return module;

fail:
    Py_DECREF(module);
    return NULL;
}
