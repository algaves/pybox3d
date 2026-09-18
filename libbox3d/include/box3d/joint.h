#ifndef BOX3D_JOINT_H
#define BOX3D_JOINT_H

#include "box3d_export.h"
#include "box3d/vec3.h"
#include "box3d/quat.h"
#include "box3d/id.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    B3_JOINT_DISTANCE = 0,
    B3_JOINT_SPHERICAL,
    B3_JOINT_REVOLUTE,
    B3_JOINT_PRISMATIC,
    B3_JOINT_WELD,
    B3_JOINT_MOTOR,
    B3_JOINT_WHEEL,
    B3_JOINT_FILTER,
    B3_JOINT_PARALLEL
} b3_JointKind;

typedef struct {
    b3_real rest_length;
    b3_real min_length; /* only enforced when has_limits is nonzero */
    b3_real max_length; /* only enforced when has_limits is nonzero */
    int has_limits;
    b3_real stiffness; /* <= 0: rigid (default); > 0: spring toward rest_length */
    b3_real damping;   /* only used when stiffness > 0 */
} b3_DistanceJointParams;

typedef struct {
    b3_Vec3 axis_a; /* hinge axis, body-A-local, normalized */
    int enable_motor;
    b3_real motor_speed;      /* target relative angular velocity around the hinge axis */
    b3_real max_motor_torque; /* N*m; clamps the motor's impulse per step */
} b3_RevoluteJointParams;

typedef struct {
    b3_Vec3 axis_a; /* slide axis, body-A-local, normalized */
    b3_real min_translation;
    b3_real max_translation;
    int has_limits;
    int enable_motor;
    b3_real motor_speed;     /* target relative linear velocity along the slide axis */
    b3_real max_motor_force; /* N; clamps the motor's impulse per step */
} b3_PrismaticJointParams;

typedef struct {
    b3_Vec3 linear_offset;  /* target position of body B's anchor in body A's local frame */
    b3_Quat angular_offset; /* target orientation of body B relative to body A */
    b3_real linear_stiffness;
    b3_real linear_damping;
    b3_real angular_stiffness;
    b3_real angular_damping;
} b3_MotorJointParams;

typedef struct {
    b3_Vec3 suspension_axis_a; /* body-A-local, normalized */
    b3_Vec3 axle_axis_a;       /* body-A-local, normalized, the free spin axis */
    b3_real min_translation;
    b3_real max_translation;
    int has_limits;
    b3_real suspension_stiffness; /* <= 0: free slide (default); > 0: spring toward zero offset */
    b3_real suspension_damping;
} b3_WheelJointParams;

/* A constraint (or, for Filter, an anti-constraint) between two bodies
 * (referenced by stable b3_BodyId, not raw index -- see box3d/id.h). All
 * kinds but Filter are anchored at a per-body local-space offset
 * (`anchor_a`/`anchor_b`, default the origin = body center): the
 * world-space anchor of a body is `position + rotate(orientation,
 * local_anchor)`.
 *
 * b3_World stores every kind in one array/slot-map (mirroring b3_Shape's
 * tagged union for shapes) -- `kind` selects which member of `params` is
 * active; World.add_joint()/add_spherical_joint()/etc. on the Python side
 * are what construct each kind, since the fields that matter (and their
 * defaults) differ per kind.
 *
 * v1 scope cut, all kinds: positional (Baumgarte) drift correction only
 * ever targets the *translation* part of a constraint (like
 * b3_DistanceJoint before this); any rotation-locking constraint
 * (Revolute's non-hinge axes, Prismatic/Weld/Parallel's full lock,
 * Wheel's non-spin axes) is velocity-only -- it stops the bodies from
 * actively drifting apart in orientation under load, but there's no
 * active pull back to alignment if they're already misaligned (e.g. from
 * floating-point drift over many steps). See docs/limitations.md. */
typedef struct {
    b3_BodyId body_a_id;
    b3_BodyId body_b_id;
    b3_JointKind kind;

    b3_Vec3 anchor_a; /* body-A-local offset from its center */
    b3_Vec3 anchor_b; /* body-B-local offset from its center */

    union {
        b3_DistanceJointParams distance;
        b3_RevoluteJointParams revolute;
        b3_PrismaticJointParams prismatic;
        b3_MotorJointParams motor;
        b3_WheelJointParams wheel;
        /* Spherical, Weld, Filter, and Parallel need no params beyond
         * anchor_a/anchor_b (Filter doesn't even use those). */
    } params;

    /* Internal bookkeeping for b3_World: this joint's own stable slot id
     * (see box3d/slotmap.h). Do not set this directly. */
    b3_Id slot_id;
} b3_Joint;

/* Backward-compatible alias: b3_DistanceJoint was its own struct before
 * every joint kind was unified into b3_Joint. A distance joint is just a
 * b3_Joint with kind == B3_JOINT_DISTANCE and params.distance filled in,
 * so the alias keeps existing code (and the field-access pattern it
 * uses) working unchanged. */
typedef b3_Joint b3_DistanceJoint;

#ifdef __cplusplus
}
#endif

#endif /* BOX3D_JOINT_H */
