#ifndef BOX3D_HULL_H
#define BOX3D_HULL_H

#include "box3d_export.h"
#include "box3d/vec3.h"
#include "box3d/quat.h"
#include "box3d/collision.h"

#ifdef __cplusplus
extern "C" {
#endif

/* v1 simplification: no heap allocation (matches b3_RigidBody's "no
 * internal heap pointers, safe to copy by value" invariant -- see
 * rigidbody.h), so vertices live in a fixed-capacity embedded array
 * rather than being computed/owned dynamically. The caller must supply
 * an already-convex vertex set (no incremental/quickhull construction
 * from an arbitrary point cloud in v1); b3_convexhull_make silently
 * truncates to B3_HULL_MAX_VERTICES if given more. */
#define B3_HULL_MAX_VERTICES 32

typedef struct {
    b3_Vec3 center;
    b3_Quat orientation;
    b3_Vec3 local_vertices[B3_HULL_MAX_VERTICES]; /* local (unrotated, uncentered) space */
    int vertex_count;
} b3_ConvexHull;

B3_API b3_ConvexHull b3_convexhull_make(
    b3_Vec3 center, b3_Quat orientation, const b3_Vec3 *local_vertices, int vertex_count
);

B3_API b3_Vec3 b3_convexhull_world_vertex(const b3_ConvexHull *hull, int index);

B3_API void b3_convexhull_compute_aabb(const b3_ConvexHull *hull, b3_Vec3 *out_min, b3_Vec3 *out_max);

/* Exact: implemented as a GJK intersection test against a zero-radius
 * point, which is equivalent to point-in-convex-hull containment. */
B3_API int b3_convexhull_contains_point(const b3_ConvexHull *hull, b3_Vec3 point);

/* Brute-force O(vertex_count) max-dot scan; the GJK/EPA collision core
 * (box3d/gjk.h) uses this as its support function for any pair
 * involving a hull. */
B3_API b3_Vec3 b3_convexhull_support(const b3_ConvexHull *hull, b3_Vec3 direction);

/* v1 approximation: without face data (see the no-quickhull note above),
 * an exact hull-surface raycast isn't available, so this tests the
 * hull's AABB instead -- see docs/limitations.md. */
B3_API b3_RayHit b3_convexhull_raycast(
    const b3_ConvexHull *hull, b3_Vec3 origin, b3_Vec3 dir, b3_real max_t
);

#ifdef __cplusplus
}
#endif

#endif /* BOX3D_HULL_H */
