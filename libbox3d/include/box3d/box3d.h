#ifndef BOX3D_BOX3D_H
#define BOX3D_BOX3D_H

#include "box3d_export.h"
#include "box3d/vec3.h"
#include "box3d/quat.h"
#include "box3d/collision.h"

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

/* The point on `box`'s surface farthest along `direction` (need not be
 * unit length). The generic GJK/EPA collision core (box3d/gjk.h) uses
 * this same "support function" shape for every convex shape kind; box-box
 * itself still uses the exact SAT test below rather than GJK. */
B3_API b3_Vec3 b3_box3d_support(const b3_Box3D *box, b3_Vec3 direction);

/* Separating Axis Theorem test over all 15 candidate axes (3 face normals
 * of `a`, 3 of `b`, 9 edge-edge cross products). Returns nonzero if the
 * boxes overlap and, when `out` is non-NULL, fills in the exact
 * least-penetrating axis as the contact normal/penetration -- a face
 * axis (`out->point` is then the midpoint of each box's support point
 * along the normal) or, when an edge-edge axis actually has the least
 * overlap, that exact edge-edge normal (`out->point` is then the
 * midpoint of the two edges' closest points). Either way this is still
 * a single representative point, not a full manifold -- see
 * b3_ContactInfo. */
B3_API int b3_box3d_overlap(const b3_Box3D *a, const b3_Box3D *b, b3_ContactInfo *out);

/* Slab-method ray/box test. `dir` need not be normalized; `t` and `max_t`
 * are then in units of `dir`'s length. */
B3_API b3_RayHit b3_box3d_raycast(const b3_Box3D *box, b3_Vec3 origin, b3_Vec3 dir, b3_real max_t);

#ifdef __cplusplus
}
#endif

#endif /* BOX3D_BOX3D_H */
