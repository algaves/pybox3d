#include "py_joint.h"
#include "py_world.h"
#include "py_rigidbody.h"
#include "py_vec3.h"
#include "py_quat.h"

PyObject *PyDistanceJoint_FromWorldId(PyObject *world, b3_JointId id) {
    PyDistanceJointObject *self = PyObject_New(PyDistanceJointObject, &PyDistanceJoint_Type);
    if (self == NULL) {
        return NULL;
    }
    Py_INCREF(world);
    self->world = world;
    self->world_id = id;
    return (PyObject *)self;
}

b3_DistanceJoint *PyDistanceJoint_Resolve(PyDistanceJointObject *self) {
    b3_DistanceJoint *joint =
        b3_world_get_joint_by_id(&((PyWorldObject *)self->world)->world, self->world_id);
    if (joint == NULL) {
        PyErr_SetString(
            PyExc_ValueError,
            "this DistanceJoint is no longer valid: it was removed from its World "
            "(via World.remove_joint, or as a side effect of removing a body it "
            "referenced)"
        );
        return NULL;
    }
    return joint;
}

static PyObject *joint_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    (void)type;
    (void)args;
    (void)kwds;
    PyErr_SetString(
        PyExc_TypeError, "DistanceJoint cannot be constructed directly; use World.add_joint()"
    );
    return NULL;
}

static void joint_dealloc(PyDistanceJointObject *self) {
    Py_XDECREF(self->world);
    Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyObject *joint_repr(PyDistanceJointObject *self) {
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) {
        PyErr_Clear();
        return PyUnicode_FromString("<DistanceJoint (stale World reference)>");
    }
    return PyUnicode_FromFormat(
        "DistanceJoint(body_a_id=%d, body_b_id=%d, rest_length=%g)", joint->body_a_id.index,
        joint->body_b_id.index, (double)joint->params.distance.rest_length
    );
}

static PyObject *joint_get_rest_length(PyDistanceJointObject *self, void *closure) {
    (void)closure;
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) return NULL;
    return PyFloat_FromDouble((double)joint->params.distance.rest_length);
}
static int joint_set_rest_length(PyDistanceJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete rest_length");
        return -1;
    }
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) return -1;
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    if (v < 0.0) {
        PyErr_SetString(PyExc_ValueError, "rest_length must be >= 0");
        return -1;
    }
    joint->params.distance.rest_length = (b3_real)v;
    return 0;
}

static PyObject *joint_get_anchor_a(PyDistanceJointObject *self, void *closure) {
    (void)closure;
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) return NULL;
    return PyVec3_FromVec3(joint->anchor_a);
}
static int joint_set_anchor_a(PyDistanceJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete anchor_a");
        return -1;
    }
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) return -1;
    return PyVec3_Parse(value, &joint->anchor_a);
}

static PyObject *joint_get_anchor_b(PyDistanceJointObject *self, void *closure) {
    (void)closure;
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) return NULL;
    return PyVec3_FromVec3(joint->anchor_b);
}
static int joint_set_anchor_b(PyDistanceJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete anchor_b");
        return -1;
    }
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) return -1;
    return PyVec3_Parse(value, &joint->anchor_b);
}

static PyObject *joint_get_min_length(PyDistanceJointObject *self, void *closure) {
    (void)closure;
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) return NULL;
    return PyFloat_FromDouble((double)joint->params.distance.min_length);
}
static int joint_set_min_length(PyDistanceJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete min_length");
        return -1;
    }
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) return -1;
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    if (v < 0.0) {
        PyErr_SetString(PyExc_ValueError, "min_length must be >= 0");
        return -1;
    }
    joint->params.distance.min_length = (b3_real)v;
    return 0;
}

static PyObject *joint_get_max_length(PyDistanceJointObject *self, void *closure) {
    (void)closure;
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) return NULL;
    return PyFloat_FromDouble((double)joint->params.distance.max_length);
}
static int joint_set_max_length(PyDistanceJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete max_length");
        return -1;
    }
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) return -1;
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    if (v < 0.0) {
        PyErr_SetString(PyExc_ValueError, "max_length must be >= 0");
        return -1;
    }
    joint->params.distance.max_length = (b3_real)v;
    return 0;
}

static PyObject *joint_get_has_limits(PyDistanceJointObject *self, void *closure) {
    (void)closure;
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) return NULL;
    return PyBool_FromLong(joint->params.distance.has_limits);
}
static int joint_set_has_limits(PyDistanceJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete has_limits");
        return -1;
    }
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) return -1;
    int truth = PyObject_IsTrue(value);
    if (truth < 0) return -1;
    joint->params.distance.has_limits = truth;
    return 0;
}

static PyObject *joint_get_stiffness(PyDistanceJointObject *self, void *closure) {
    (void)closure;
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) return NULL;
    return PyFloat_FromDouble((double)joint->params.distance.stiffness);
}
static int joint_set_stiffness(PyDistanceJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete stiffness");
        return -1;
    }
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) return -1;
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    if (v < 0.0) {
        PyErr_SetString(PyExc_ValueError, "stiffness must be >= 0");
        return -1;
    }
    joint->params.distance.stiffness = (b3_real)v;
    return 0;
}

static PyObject *joint_get_damping(PyDistanceJointObject *self, void *closure) {
    (void)closure;
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) return NULL;
    return PyFloat_FromDouble((double)joint->params.distance.damping);
}
static int joint_set_damping(PyDistanceJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete damping");
        return -1;
    }
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) return -1;
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    if (v < 0.0) {
        PyErr_SetString(PyExc_ValueError, "damping must be >= 0");
        return -1;
    }
    joint->params.distance.damping = (b3_real)v;
    return 0;
}

static PyObject *joint_get_body_a(PyDistanceJointObject *self, void *closure) {
    (void)closure;
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) return NULL;
    return PyRigidBody_FromWorldId(self->world, joint->body_a_id);
}

static PyObject *joint_get_body_b(PyDistanceJointObject *self, void *closure) {
    (void)closure;
    b3_DistanceJoint *joint = PyDistanceJoint_Resolve(self);
    if (joint == NULL) return NULL;
    return PyRigidBody_FromWorldId(self->world, joint->body_b_id);
}

static PyGetSetDef joint_getset[] = {
    {"rest_length", (getter)joint_get_rest_length, (setter)joint_set_rest_length,
     "target distance between the two world-space anchors (rigid mode) or the spring's "
     "target distance (spring mode, stiffness > 0)", NULL},
    {"anchor_a", (getter)joint_get_anchor_a, (setter)joint_set_anchor_a,
     "body_a-local offset from its center that this joint is anchored to (Vec3)", NULL},
    {"anchor_b", (getter)joint_get_anchor_b, (setter)joint_set_anchor_b,
     "body_b-local offset from its center that this joint is anchored to (Vec3)", NULL},
    {"has_limits", (getter)joint_get_has_limits, (setter)joint_set_has_limits,
     "whether min_length/max_length are enforced", NULL},
    {"min_length", (getter)joint_get_min_length, (setter)joint_set_min_length,
     "lower distance bound, only enforced while has_limits is true", NULL},
    {"max_length", (getter)joint_get_max_length, (setter)joint_set_max_length,
     "upper distance bound, only enforced while has_limits is true", NULL},
    {"stiffness", (getter)joint_get_stiffness, (setter)joint_set_stiffness,
     "spring stiffness; <= 0 (default) means a rigid constraint instead of a spring", NULL},
    {"damping", (getter)joint_get_damping, (setter)joint_set_damping,
     "spring damping coefficient, only used while stiffness > 0", NULL},
    {"body_a", (getter)joint_get_body_a, NULL,
     "first connected body (a fresh world-backed RigidBody handle each access, read-only)", NULL},
    {"body_b", (getter)joint_get_body_b, NULL,
     "second connected body (a fresh world-backed RigidBody handle each access, read-only)", NULL},
    {NULL},
};

PyTypeObject PyDistanceJoint_Type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "pybox3d.DistanceJoint",
    .tp_basicsize = sizeof(PyDistanceJointObject),
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = PyDoc_STR(
        "A distance constraint between two bodies, created via World.add_joint(). "
        "Anchored at anchor_a/anchor_b (body-local offsets, default each body's "
        "center). In rigid mode (stiffness <= 0, the default) it holds the "
        "distance between the two world-space anchors at exactly rest_length -- "
        "a stiff rod. In spring mode (stiffness > 0) it instead applies a "
        "Hooke's-law force toward rest_length, optionally hard-stopped at "
        "min_length/max_length when has_limits is set."
    ),
    .tp_new = joint_new,
    .tp_dealloc = (destructor)joint_dealloc,
    .tp_repr = (reprfunc)joint_repr,
    .tp_getset = joint_getset,
};

/* ---- PyJoint: every b3_JointKind other than Distance ---- */

static const char *joint_kind_name(b3_JointKind kind) {
    switch (kind) {
        case B3_JOINT_DISTANCE: return "distance";
        case B3_JOINT_SPHERICAL: return "spherical";
        case B3_JOINT_REVOLUTE: return "revolute";
        case B3_JOINT_PRISMATIC: return "prismatic";
        case B3_JOINT_WELD: return "weld";
        case B3_JOINT_MOTOR: return "motor";
        case B3_JOINT_WHEEL: return "wheel";
        case B3_JOINT_FILTER: return "filter";
        case B3_JOINT_PARALLEL: return "parallel";
        default: return "unknown";
    }
}

PyObject *PyJoint_FromWorldId(PyObject *world, b3_JointId id) {
    PyJointObject *self = PyObject_New(PyJointObject, &PyJoint_Type);
    if (self == NULL) {
        return NULL;
    }
    Py_INCREF(world);
    self->world = world;
    self->world_id = id;
    return (PyObject *)self;
}

b3_Joint *PyJoint_Resolve(PyJointObject *self) {
    b3_Joint *joint = b3_world_get_joint_by_id(&((PyWorldObject *)self->world)->world, self->world_id);
    if (joint == NULL) {
        PyErr_SetString(
            PyExc_ValueError,
            "this joint is no longer valid: it was removed from its World "
            "(via World.remove_joint, or as a side effect of removing a body it "
            "referenced)"
        );
        return NULL;
    }
    return joint;
}

/* Fails (returning -1/NULL to the caller) with AttributeError if `joint`
 * isn't one of the two kinds in `allowed`, naming `attr` and the actual
 * kind in the message. Shared by every kind-specific getter/setter
 * below -- most attributes apply to exactly one or two kinds. */
static int joint_require_kind2(
    const b3_Joint *joint, const char *attr, b3_JointKind allowed_1, b3_JointKind allowed_2
) {
    if (joint->kind == allowed_1 || joint->kind == allowed_2) {
        return 0;
    }
    PyErr_Format(
        PyExc_AttributeError, "'%s' is not available on a %s joint", attr,
        joint_kind_name(joint->kind)
    );
    return -1;
}

static PyObject *pyjoint_new(PyTypeObject *type, PyObject *args, PyObject *kwds) {
    (void)type;
    (void)args;
    (void)kwds;
    PyErr_SetString(
        PyExc_TypeError,
        "Joint cannot be constructed directly; use one of World.add_spherical_joint()/"
        "add_revolute_joint()/add_prismatic_joint()/add_weld_joint()/add_motor_joint()/"
        "add_wheel_joint()/add_filter_joint()/add_parallel_joint()"
    );
    return NULL;
}

static void pyjoint_dealloc(PyJointObject *self) {
    Py_XDECREF(self->world);
    Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyObject *pyjoint_repr(PyJointObject *self) {
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) {
        PyErr_Clear();
        return PyUnicode_FromString("<Joint (stale World reference)>");
    }
    return PyUnicode_FromFormat(
        "Joint(kind=%s, body_a_id=%d, body_b_id=%d)", joint_kind_name(joint->kind),
        joint->body_a_id.index, joint->body_b_id.index
    );
}

static PyObject *pyjoint_get_kind(PyJointObject *self, void *closure) {
    (void)closure;
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return NULL;
    return PyUnicode_FromString(joint_kind_name(joint->kind));
}

static PyObject *pyjoint_get_anchor_a(PyJointObject *self, void *closure) {
    (void)closure;
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return NULL;
    return PyVec3_FromVec3(joint->anchor_a);
}
static int pyjoint_set_anchor_a(PyJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete anchor_a");
        return -1;
    }
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return -1;
    return PyVec3_Parse(value, &joint->anchor_a);
}

static PyObject *pyjoint_get_anchor_b(PyJointObject *self, void *closure) {
    (void)closure;
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return NULL;
    return PyVec3_FromVec3(joint->anchor_b);
}
static int pyjoint_set_anchor_b(PyJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete anchor_b");
        return -1;
    }
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return -1;
    return PyVec3_Parse(value, &joint->anchor_b);
}

/* axis: Revolute's hinge axis or Prismatic's slide axis (both body-A-local). */
static PyObject *pyjoint_get_axis(PyJointObject *self, void *closure) {
    (void)closure;
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return NULL;
    if (joint_require_kind2(joint, "axis", B3_JOINT_REVOLUTE, B3_JOINT_PRISMATIC) < 0) return NULL;
    return PyVec3_FromVec3(
        joint->kind == B3_JOINT_REVOLUTE ? joint->params.revolute.axis_a : joint->params.prismatic.axis_a
    );
}
static int pyjoint_set_axis(PyJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete axis");
        return -1;
    }
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return -1;
    if (joint_require_kind2(joint, "axis", B3_JOINT_REVOLUTE, B3_JOINT_PRISMATIC) < 0) return -1;
    b3_Vec3 axis;
    if (PyVec3_Parse(value, &axis) < 0) return -1;
    if (joint->kind == B3_JOINT_REVOLUTE) {
        joint->params.revolute.axis_a = axis;
    } else {
        joint->params.prismatic.axis_a = axis;
    }
    return 0;
}

static PyObject *pyjoint_get_suspension_axis(PyJointObject *self, void *closure) {
    (void)closure;
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return NULL;
    if (joint_require_kind2(joint, "suspension_axis", B3_JOINT_WHEEL, B3_JOINT_WHEEL) < 0) return NULL;
    return PyVec3_FromVec3(joint->params.wheel.suspension_axis_a);
}
static int pyjoint_set_suspension_axis(PyJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete suspension_axis");
        return -1;
    }
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return -1;
    if (joint_require_kind2(joint, "suspension_axis", B3_JOINT_WHEEL, B3_JOINT_WHEEL) < 0) return -1;
    return PyVec3_Parse(value, &joint->params.wheel.suspension_axis_a);
}

static PyObject *pyjoint_get_axle_axis(PyJointObject *self, void *closure) {
    (void)closure;
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return NULL;
    if (joint_require_kind2(joint, "axle_axis", B3_JOINT_WHEEL, B3_JOINT_WHEEL) < 0) return NULL;
    return PyVec3_FromVec3(joint->params.wheel.axle_axis_a);
}
static int pyjoint_set_axle_axis(PyJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete axle_axis");
        return -1;
    }
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return -1;
    if (joint_require_kind2(joint, "axle_axis", B3_JOINT_WHEEL, B3_JOINT_WHEEL) < 0) return -1;
    return PyVec3_Parse(value, &joint->params.wheel.axle_axis_a);
}

static PyObject *pyjoint_get_has_limits(PyJointObject *self, void *closure) {
    (void)closure;
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return NULL;
    if (joint_require_kind2(joint, "has_limits", B3_JOINT_PRISMATIC, B3_JOINT_WHEEL) < 0) return NULL;
    int v = joint->kind == B3_JOINT_PRISMATIC ? joint->params.prismatic.has_limits
                                               : joint->params.wheel.has_limits;
    return PyBool_FromLong(v);
}
static int pyjoint_set_has_limits(PyJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete has_limits");
        return -1;
    }
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return -1;
    if (joint_require_kind2(joint, "has_limits", B3_JOINT_PRISMATIC, B3_JOINT_WHEEL) < 0) return -1;
    int truth = PyObject_IsTrue(value);
    if (truth < 0) return -1;
    if (joint->kind == B3_JOINT_PRISMATIC) {
        joint->params.prismatic.has_limits = truth;
    } else {
        joint->params.wheel.has_limits = truth;
    }
    return 0;
}

static PyObject *pyjoint_get_min_translation(PyJointObject *self, void *closure) {
    (void)closure;
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return NULL;
    if (joint_require_kind2(joint, "min_translation", B3_JOINT_PRISMATIC, B3_JOINT_WHEEL) < 0) return NULL;
    b3_real v = joint->kind == B3_JOINT_PRISMATIC ? joint->params.prismatic.min_translation
                                                   : joint->params.wheel.min_translation;
    return PyFloat_FromDouble((double)v);
}
static int pyjoint_set_min_translation(PyJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete min_translation");
        return -1;
    }
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return -1;
    if (joint_require_kind2(joint, "min_translation", B3_JOINT_PRISMATIC, B3_JOINT_WHEEL) < 0) return -1;
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    if (joint->kind == B3_JOINT_PRISMATIC) {
        joint->params.prismatic.min_translation = (b3_real)v;
    } else {
        joint->params.wheel.min_translation = (b3_real)v;
    }
    return 0;
}

static PyObject *pyjoint_get_max_translation(PyJointObject *self, void *closure) {
    (void)closure;
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return NULL;
    if (joint_require_kind2(joint, "max_translation", B3_JOINT_PRISMATIC, B3_JOINT_WHEEL) < 0) return NULL;
    b3_real v = joint->kind == B3_JOINT_PRISMATIC ? joint->params.prismatic.max_translation
                                                   : joint->params.wheel.max_translation;
    return PyFloat_FromDouble((double)v);
}
static int pyjoint_set_max_translation(PyJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete max_translation");
        return -1;
    }
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return -1;
    if (joint_require_kind2(joint, "max_translation", B3_JOINT_PRISMATIC, B3_JOINT_WHEEL) < 0) return -1;
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    if (joint->kind == B3_JOINT_PRISMATIC) {
        joint->params.prismatic.max_translation = (b3_real)v;
    } else {
        joint->params.wheel.max_translation = (b3_real)v;
    }
    return 0;
}

static PyObject *pyjoint_get_enable_motor(PyJointObject *self, void *closure) {
    (void)closure;
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return NULL;
    if (joint_require_kind2(joint, "enable_motor", B3_JOINT_REVOLUTE, B3_JOINT_PRISMATIC) < 0) return NULL;
    int v = joint->kind == B3_JOINT_REVOLUTE ? joint->params.revolute.enable_motor
                                              : joint->params.prismatic.enable_motor;
    return PyBool_FromLong(v);
}
static int pyjoint_set_enable_motor(PyJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete enable_motor");
        return -1;
    }
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return -1;
    if (joint_require_kind2(joint, "enable_motor", B3_JOINT_REVOLUTE, B3_JOINT_PRISMATIC) < 0) return -1;
    int truth = PyObject_IsTrue(value);
    if (truth < 0) return -1;
    if (joint->kind == B3_JOINT_REVOLUTE) {
        joint->params.revolute.enable_motor = truth;
    } else {
        joint->params.prismatic.enable_motor = truth;
    }
    return 0;
}

static PyObject *pyjoint_get_motor_speed(PyJointObject *self, void *closure) {
    (void)closure;
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return NULL;
    if (joint_require_kind2(joint, "motor_speed", B3_JOINT_REVOLUTE, B3_JOINT_PRISMATIC) < 0) return NULL;
    b3_real v = joint->kind == B3_JOINT_REVOLUTE ? joint->params.revolute.motor_speed
                                                  : joint->params.prismatic.motor_speed;
    return PyFloat_FromDouble((double)v);
}
static int pyjoint_set_motor_speed(PyJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete motor_speed");
        return -1;
    }
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return -1;
    if (joint_require_kind2(joint, "motor_speed", B3_JOINT_REVOLUTE, B3_JOINT_PRISMATIC) < 0) return -1;
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    if (joint->kind == B3_JOINT_REVOLUTE) {
        joint->params.revolute.motor_speed = (b3_real)v;
    } else {
        joint->params.prismatic.motor_speed = (b3_real)v;
    }
    return 0;
}

static PyObject *pyjoint_get_max_motor_effort(PyJointObject *self, void *closure) {
    (void)closure;
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return NULL;
    if (joint_require_kind2(joint, "max_motor_effort", B3_JOINT_REVOLUTE, B3_JOINT_PRISMATIC) < 0) return NULL;
    b3_real v = joint->kind == B3_JOINT_REVOLUTE ? joint->params.revolute.max_motor_torque
                                                  : joint->params.prismatic.max_motor_force;
    return PyFloat_FromDouble((double)v);
}
static int pyjoint_set_max_motor_effort(PyJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete max_motor_effort");
        return -1;
    }
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return -1;
    if (joint_require_kind2(joint, "max_motor_effort", B3_JOINT_REVOLUTE, B3_JOINT_PRISMATIC) < 0) return -1;
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    if (v < 0.0) {
        PyErr_SetString(PyExc_ValueError, "max_motor_effort must be >= 0");
        return -1;
    }
    if (joint->kind == B3_JOINT_REVOLUTE) {
        joint->params.revolute.max_motor_torque = (b3_real)v;
    } else {
        joint->params.prismatic.max_motor_force = (b3_real)v;
    }
    return 0;
}

static PyObject *pyjoint_get_suspension_stiffness(PyJointObject *self, void *closure) {
    (void)closure;
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return NULL;
    if (joint_require_kind2(joint, "suspension_stiffness", B3_JOINT_WHEEL, B3_JOINT_WHEEL) < 0) return NULL;
    return PyFloat_FromDouble((double)joint->params.wheel.suspension_stiffness);
}
static int pyjoint_set_suspension_stiffness(PyJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete suspension_stiffness");
        return -1;
    }
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return -1;
    if (joint_require_kind2(joint, "suspension_stiffness", B3_JOINT_WHEEL, B3_JOINT_WHEEL) < 0) return -1;
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    if (v < 0.0) {
        PyErr_SetString(PyExc_ValueError, "suspension_stiffness must be >= 0");
        return -1;
    }
    joint->params.wheel.suspension_stiffness = (b3_real)v;
    return 0;
}

static PyObject *pyjoint_get_suspension_damping(PyJointObject *self, void *closure) {
    (void)closure;
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return NULL;
    if (joint_require_kind2(joint, "suspension_damping", B3_JOINT_WHEEL, B3_JOINT_WHEEL) < 0) return NULL;
    return PyFloat_FromDouble((double)joint->params.wheel.suspension_damping);
}
static int pyjoint_set_suspension_damping(PyJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete suspension_damping");
        return -1;
    }
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return -1;
    if (joint_require_kind2(joint, "suspension_damping", B3_JOINT_WHEEL, B3_JOINT_WHEEL) < 0) return -1;
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    if (v < 0.0) {
        PyErr_SetString(PyExc_ValueError, "suspension_damping must be >= 0");
        return -1;
    }
    joint->params.wheel.suspension_damping = (b3_real)v;
    return 0;
}

static PyObject *pyjoint_get_linear_offset(PyJointObject *self, void *closure) {
    (void)closure;
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return NULL;
    if (joint_require_kind2(joint, "linear_offset", B3_JOINT_MOTOR, B3_JOINT_MOTOR) < 0) return NULL;
    return PyVec3_FromVec3(joint->params.motor.linear_offset);
}
static int pyjoint_set_linear_offset(PyJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete linear_offset");
        return -1;
    }
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return -1;
    if (joint_require_kind2(joint, "linear_offset", B3_JOINT_MOTOR, B3_JOINT_MOTOR) < 0) return -1;
    return PyVec3_Parse(value, &joint->params.motor.linear_offset);
}

static PyObject *pyjoint_get_angular_offset(PyJointObject *self, void *closure) {
    (void)closure;
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return NULL;
    if (joint_require_kind2(joint, "angular_offset", B3_JOINT_MOTOR, B3_JOINT_MOTOR) < 0) return NULL;
    return PyQuat_FromQuat(joint->params.motor.angular_offset);
}
static int pyjoint_set_angular_offset(PyJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete angular_offset");
        return -1;
    }
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return -1;
    if (joint_require_kind2(joint, "angular_offset", B3_JOINT_MOTOR, B3_JOINT_MOTOR) < 0) return -1;
    return PyQuat_Parse(value, &joint->params.motor.angular_offset);
}

static PyObject *pyjoint_get_linear_stiffness(PyJointObject *self, void *closure) {
    (void)closure;
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return NULL;
    if (joint_require_kind2(joint, "linear_stiffness", B3_JOINT_MOTOR, B3_JOINT_MOTOR) < 0) return NULL;
    return PyFloat_FromDouble((double)joint->params.motor.linear_stiffness);
}
static int pyjoint_set_linear_stiffness(PyJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete linear_stiffness");
        return -1;
    }
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return -1;
    if (joint_require_kind2(joint, "linear_stiffness", B3_JOINT_MOTOR, B3_JOINT_MOTOR) < 0) return -1;
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    if (v < 0.0) {
        PyErr_SetString(PyExc_ValueError, "linear_stiffness must be >= 0");
        return -1;
    }
    joint->params.motor.linear_stiffness = (b3_real)v;
    return 0;
}

static PyObject *pyjoint_get_linear_damping(PyJointObject *self, void *closure) {
    (void)closure;
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return NULL;
    if (joint_require_kind2(joint, "linear_damping", B3_JOINT_MOTOR, B3_JOINT_MOTOR) < 0) return NULL;
    return PyFloat_FromDouble((double)joint->params.motor.linear_damping);
}
static int pyjoint_set_linear_damping(PyJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete linear_damping");
        return -1;
    }
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return -1;
    if (joint_require_kind2(joint, "linear_damping", B3_JOINT_MOTOR, B3_JOINT_MOTOR) < 0) return -1;
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    if (v < 0.0) {
        PyErr_SetString(PyExc_ValueError, "linear_damping must be >= 0");
        return -1;
    }
    joint->params.motor.linear_damping = (b3_real)v;
    return 0;
}

static PyObject *pyjoint_get_angular_stiffness(PyJointObject *self, void *closure) {
    (void)closure;
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return NULL;
    if (joint_require_kind2(joint, "angular_stiffness", B3_JOINT_MOTOR, B3_JOINT_MOTOR) < 0) return NULL;
    return PyFloat_FromDouble((double)joint->params.motor.angular_stiffness);
}
static int pyjoint_set_angular_stiffness(PyJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete angular_stiffness");
        return -1;
    }
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return -1;
    if (joint_require_kind2(joint, "angular_stiffness", B3_JOINT_MOTOR, B3_JOINT_MOTOR) < 0) return -1;
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    if (v < 0.0) {
        PyErr_SetString(PyExc_ValueError, "angular_stiffness must be >= 0");
        return -1;
    }
    joint->params.motor.angular_stiffness = (b3_real)v;
    return 0;
}

static PyObject *pyjoint_get_angular_damping(PyJointObject *self, void *closure) {
    (void)closure;
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return NULL;
    if (joint_require_kind2(joint, "angular_damping", B3_JOINT_MOTOR, B3_JOINT_MOTOR) < 0) return NULL;
    return PyFloat_FromDouble((double)joint->params.motor.angular_damping);
}
static int pyjoint_set_angular_damping(PyJointObject *self, PyObject *value, void *closure) {
    (void)closure;
    if (value == NULL) {
        PyErr_SetString(PyExc_AttributeError, "cannot delete angular_damping");
        return -1;
    }
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return -1;
    if (joint_require_kind2(joint, "angular_damping", B3_JOINT_MOTOR, B3_JOINT_MOTOR) < 0) return -1;
    double v = PyFloat_AsDouble(value);
    if (v == -1.0 && PyErr_Occurred()) return -1;
    if (v < 0.0) {
        PyErr_SetString(PyExc_ValueError, "angular_damping must be >= 0");
        return -1;
    }
    joint->params.motor.angular_damping = (b3_real)v;
    return 0;
}

static PyObject *pyjoint_get_body_a(PyJointObject *self, void *closure) {
    (void)closure;
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return NULL;
    return PyRigidBody_FromWorldId(self->world, joint->body_a_id);
}

static PyObject *pyjoint_get_body_b(PyJointObject *self, void *closure) {
    (void)closure;
    b3_Joint *joint = PyJoint_Resolve(self);
    if (joint == NULL) return NULL;
    return PyRigidBody_FromWorldId(self->world, joint->body_b_id);
}

static PyGetSetDef pyjoint_getset[] = {
    {"kind", (getter)pyjoint_get_kind, NULL,
     "which joint kind this is: 'spherical', 'revolute', 'prismatic', 'weld', 'motor', "
     "'wheel', 'filter', or 'parallel' (read-only)", NULL},
    {"anchor_a", (getter)pyjoint_get_anchor_a, (setter)pyjoint_set_anchor_a,
     "body_a-local offset from its center that this joint is anchored to (Vec3)", NULL},
    {"anchor_b", (getter)pyjoint_get_anchor_b, (setter)pyjoint_set_anchor_b,
     "body_b-local offset from its center that this joint is anchored to (Vec3)", NULL},
    {"axis", (getter)pyjoint_get_axis, (setter)pyjoint_set_axis,
     "body_a-local hinge axis (Revolute) or slide axis (Prismatic)", NULL},
    {"suspension_axis", (getter)pyjoint_get_suspension_axis, (setter)pyjoint_set_suspension_axis,
     "body_a-local suspension (translation) axis (Wheel only)", NULL},
    {"axle_axis", (getter)pyjoint_get_axle_axis, (setter)pyjoint_set_axle_axis,
     "body_a-local free-spin axis (Wheel only)", NULL},
    {"has_limits", (getter)pyjoint_get_has_limits, (setter)pyjoint_set_has_limits,
     "whether min_translation/max_translation are enforced (Prismatic/Wheel only)", NULL},
    {"min_translation", (getter)pyjoint_get_min_translation, (setter)pyjoint_set_min_translation,
     "lower translation bound along axis/suspension_axis (Prismatic/Wheel only)", NULL},
    {"max_translation", (getter)pyjoint_get_max_translation, (setter)pyjoint_set_max_translation,
     "upper translation bound along axis/suspension_axis (Prismatic/Wheel only)", NULL},
    {"enable_motor", (getter)pyjoint_get_enable_motor, (setter)pyjoint_set_enable_motor,
     "whether the motor drives toward motor_speed (Revolute/Prismatic only)", NULL},
    {"motor_speed", (getter)pyjoint_get_motor_speed, (setter)pyjoint_set_motor_speed,
     "target relative angular (Revolute, rad/s) or linear (Prismatic, m/s) velocity", NULL},
    {"max_motor_effort", (getter)pyjoint_get_max_motor_effort, (setter)pyjoint_set_max_motor_effort,
     "clamps the motor's torque (Revolute, N*m) or force (Prismatic, N) per step", NULL},
    {"suspension_stiffness", (getter)pyjoint_get_suspension_stiffness,
     (setter)pyjoint_set_suspension_stiffness,
     "suspension spring stiffness; <= 0 (default) means free slide (Wheel only)", NULL},
    {"suspension_damping", (getter)pyjoint_get_suspension_damping, (setter)pyjoint_set_suspension_damping,
     "suspension spring damping, only used while suspension_stiffness > 0 (Wheel only)", NULL},
    {"linear_offset", (getter)pyjoint_get_linear_offset, (setter)pyjoint_set_linear_offset,
     "target position of body_b relative to body_a's frame (Motor only)", NULL},
    {"angular_offset", (getter)pyjoint_get_angular_offset, (setter)pyjoint_set_angular_offset,
     "target orientation of body_b relative to body_a (Quat, Motor only)", NULL},
    {"linear_stiffness", (getter)pyjoint_get_linear_stiffness, (setter)pyjoint_set_linear_stiffness,
     "linear spring stiffness toward linear_offset; <= 0 disables it (Motor only)", NULL},
    {"linear_damping", (getter)pyjoint_get_linear_damping, (setter)pyjoint_set_linear_damping,
     "linear spring damping (Motor only)", NULL},
    {"angular_stiffness", (getter)pyjoint_get_angular_stiffness, (setter)pyjoint_set_angular_stiffness,
     "angular spring stiffness toward angular_offset; <= 0 disables it (Motor only)", NULL},
    {"angular_damping", (getter)pyjoint_get_angular_damping, (setter)pyjoint_set_angular_damping,
     "angular spring damping (Motor only)", NULL},
    {"body_a", (getter)pyjoint_get_body_a, NULL,
     "first connected body (a fresh world-backed RigidBody handle each access, read-only)", NULL},
    {"body_b", (getter)pyjoint_get_body_b, NULL,
     "second connected body (a fresh world-backed RigidBody handle each access, read-only)", NULL},
    {NULL},
};

PyTypeObject PyJoint_Type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "pybox3d.Joint",
    .tp_basicsize = sizeof(PyJointObject),
    .tp_flags = Py_TPFLAGS_DEFAULT,
    .tp_doc = PyDoc_STR(
        "A joint other than DistanceJoint, created via one of World's add_*_joint() "
        "methods -- see `kind` for which one. Every attribute that doesn't apply to "
        "this handle's kind raises AttributeError; see World's add_*_joint() "
        "docstrings for which attributes matter for each kind."
    ),
    .tp_new = pyjoint_new,
    .tp_dealloc = (destructor)pyjoint_dealloc,
    .tp_repr = (reprfunc)pyjoint_repr,
    .tp_getset = pyjoint_getset,
};
