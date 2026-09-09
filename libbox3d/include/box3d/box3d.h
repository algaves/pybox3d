#ifndef BOX3D_BOX3D_H
#define BOX3D_BOX3D_H

#include "box3d_export.h"
#include "box3d/vec3.h"
#include "box3d/quat.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    B3_BOX_KIND_AABB = 0,
    B3_BOX_KIND_OBB = 1
} b3_BoxKind;

typedef struct {
    b3_Vec3 center;
    b3_Vec3 half_extents;
    b3_Quat orientation; /* identity for B3_BOX_KIND_AABB */
    b3_BoxKind kind;
} b3_Box3D;

B3_API b3_Box3D b3_box3d_make_aabb(b3_Vec3 center, b3_Vec3 half_extents);
B3_API b3_Box3D b3_box3d_make_obb(b3_Vec3 center, b3_Vec3 half_extents, b3_Quat orientation);

/* World-space axis-aligned bounding box that encloses `box` (identity for
 * AABB kind, computed from the rotation for OBB kind). */
B3_API void b3_box3d_compute_aabb(const b3_Box3D *box, b3_Vec3 *out_min, b3_Vec3 *out_max);

B3_API int b3_box3d_contains_point(const b3_Box3D *box, b3_Vec3 point);

typedef struct {
    b3_Vec3 normal;     /* points from `a` towards `b`, unit length */
    b3_real penetration; /* >= 0 when boxes overlap */
} b3_ContactInfo;

/* Separating Axis Theorem test over all 15 candidate axes (3 face normals
 * of `a`, 3 of `b`, 9 edge-edge cross products). Returns nonzero if the
 * boxes overlap and, when `out` is non-NULL, fills in an approximate
 * contact normal/penetration derived from the least-penetrating face axis
 * (edge-edge axes are used only for the separation test itself, matching
 * the "basic" scope of this library -- true edge-edge contact normals are
 * out of scope for v1). */
B3_API int b3_box3d_overlap(const b3_Box3D *a, const b3_Box3D *b, b3_ContactInfo *out);

typedef struct {
    int hit;
    b3_real t;
    b3_Vec3 point;
    b3_Vec3 normal;
} b3_RayHit;

/* Slab-method ray/box test. `dir` need not be normalized; `t` and `max_t`
 * are then in units of `dir`'s length. */
B3_API b3_RayHit b3_box3d_raycast(const b3_Box3D *box, b3_Vec3 origin, b3_Vec3 dir, b3_real max_t);

#ifdef __cplusplus
}
#endif

#endif /* BOX3D_BOX3D_H */
