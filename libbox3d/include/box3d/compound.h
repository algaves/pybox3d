#ifndef BOX3D_COMPOUND_H
#define BOX3D_COMPOUND_H

#include "box3d_export.h"
#include "box3d/vec3.h"
#include "box3d/quat.h"
#include "box3d/collision.h"
#include "box3d/box3d.h"
#include "box3d/sphere.h"
#include "box3d/capsule.h"
#include "box3d/hull.h"

#ifdef __cplusplus
extern "C" {
#endif

/* A compound's children are one level of "leaf" shapes only -- no
 * nested compounds. This isn't just a v1 scope cut: b3_Shape (box3d/
 * shape.h) embeds a b3_Compound by value, so a child kind that could
 * itself be a b3_Shape/b3_Compound would make the struct's size depend
 * on itself, which C can't express without heap indirection (which
 * b3_RigidBody's "no internal heap pointers" invariant rules out --
 * see rigidbody.h). Nesting compounds is rarely needed in practice
 * anyway; flatten instead. */
typedef enum {
    B3_COMPOUND_CHILD_BOX = 0,
    B3_COMPOUND_CHILD_SPHERE = 1,
    B3_COMPOUND_CHILD_CAPSULE = 2,
    B3_COMPOUND_CHILD_HULL = 3
} b3_CompoundChildKind;

typedef struct {
    b3_Vec3 local_position;
    b3_Quat local_orientation;
    b3_CompoundChildKind kind;
    union {
        b3_Box3D box;
        b3_Sphere sphere;
        b3_Capsule capsule;
        b3_ConvexHull hull;
    } as;
} b3_CompoundChild;

/* Same no-heap-allocation rationale as b3_ConvexHull -- see box3d/hull.h. */
#define B3_COMPOUND_MAX_CHILDREN 8

typedef struct {
    b3_Vec3 center;
    b3_Quat orientation;
    b3_CompoundChild children[B3_COMPOUND_MAX_CHILDREN];
    int child_count;
} b3_Compound;

B3_API b3_CompoundChild b3_compound_child_box(b3_Vec3 local_position, b3_Quat local_orientation, b3_Box3D box);
B3_API b3_CompoundChild b3_compound_child_sphere(b3_Vec3 local_position, b3_Sphere sphere);
B3_API b3_CompoundChild b3_compound_child_capsule(
    b3_Vec3 local_position, b3_Quat local_orientation, b3_Capsule capsule
);
B3_API b3_CompoundChild b3_compound_child_hull(
    b3_Vec3 local_position, b3_Quat local_orientation, b3_ConvexHull hull
);

/* Silently truncates to B3_COMPOUND_MAX_CHILDREN if given more. */
B3_API b3_Compound b3_compound_make(
    b3_Vec3 center, b3_Quat orientation, const b3_CompoundChild *children, int child_count
);

/* This child's shape at its current world pose (the compound's pose
 * composed with the child's local transform). */
B3_API void b3_compound_child_world_pose(
    const b3_Compound *compound, int index, b3_Vec3 *out_center, b3_Quat *out_orientation
);

B3_API void b3_compound_compute_aabb(const b3_Compound *compound, b3_Vec3 *out_min, b3_Vec3 *out_max);
B3_API int b3_compound_contains_point(const b3_Compound *compound, b3_Vec3 point);
B3_API b3_Vec3 b3_compound_support(const b3_Compound *compound, b3_Vec3 direction);
B3_API b3_RayHit b3_compound_raycast(
    const b3_Compound *compound, b3_Vec3 origin, b3_Vec3 dir, b3_real max_t
);

#ifdef __cplusplus
}
#endif

#endif /* BOX3D_COMPOUND_H */
