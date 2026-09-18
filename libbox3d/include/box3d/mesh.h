#ifndef BOX3D_MESH_H
#define BOX3D_MESH_H

#include "box3d_export.h"
#include "box3d/vec3.h"
#include "box3d/quat.h"
#include "box3d/collision.h"
#include "box3d/gjk.h"

#ifdef __cplusplus
extern "C" {
#endif

/* v1 simplification: static-only (see b3_rigidbody_init_mesh -- there is
 * no mass parameter) and no heap allocation (fixed triangle cap,
 * matching b3_RigidBody's no-internal-heap-pointers invariant -- see
 * rigidbody.h). A flat triangle soup, not required to be a closed
 * manifold. */
#define B3_MESH_MAX_TRIANGLES 64

typedef struct {
    b3_Vec3 center;
    b3_Quat orientation;
    /* triangle i = local_vertices[3*i], [3*i+1], [3*i+2] */
    b3_Vec3 local_vertices[B3_MESH_MAX_TRIANGLES * 3];
    int triangle_count;
} b3_TriangleMesh;

/* `local_triangle_vertices` has 3*triangle_count entries. Silently
 * truncates to B3_MESH_MAX_TRIANGLES if given more. */
B3_API b3_TriangleMesh b3_trianglemesh_make(
    b3_Vec3 center, b3_Quat orientation, const b3_Vec3 *local_triangle_vertices, int triangle_count
);

B3_API void b3_trianglemesh_triangle_world(const b3_TriangleMesh *mesh, int triangle_index, b3_Vec3 out[3]);

B3_API void b3_trianglemesh_compute_aabb(const b3_TriangleMesh *mesh, b3_Vec3 *out_min, b3_Vec3 *out_max);

/* v1: always returns 0. "Inside" isn't well-defined for an arbitrary
 * (possibly open, inconsistently-wound) triangle soup without requiring
 * and validating a closed manifold, which v1 doesn't do -- see
 * docs/limitations.md. */
B3_API int b3_trianglemesh_contains_point(const b3_TriangleMesh *mesh, b3_Vec3 point);

/* v1 approximation: like b3_convexhull_raycast, tests the mesh's AABB
 * rather than its exact triangles. */
B3_API b3_RayHit b3_trianglemesh_raycast(const b3_TriangleMesh *mesh, b3_Vec3 origin, b3_Vec3 dir, b3_real max_t);

/* GJK/EPA between one world-space triangle and a support-function shape.
 * `out->normal` points from the triangle towards `other_shape`. The
 * building block b3_trianglemesh_overlap_shape (below) and
 * box3d/heightfield.h's per-cell test are both built from this. */
B3_API int b3_trianglemesh_triangle_vs_shape(
    const b3_Vec3 triangle[3], const void *other_shape, b3_SupportFn other_support, b3_ContactInfo *out
);

/* Tests every triangle against a support-function shape, using
 * `other_aabb_min`/`max` (the other shape's own precomputed AABB) to
 * quick-reject triangles that can't possibly touch it, and keeping the
 * deepest overlap found. Returns nonzero if any triangle overlaps. */
B3_API int b3_trianglemesh_overlap_shape(
    const b3_TriangleMesh *mesh, const void *other_shape, b3_SupportFn other_support,
    b3_Vec3 other_aabb_min, b3_Vec3 other_aabb_max, b3_ContactInfo *out
);

#ifdef __cplusplus
}
#endif

#endif /* BOX3D_MESH_H */
