#ifndef BOX3D_HEIGHTFIELD_H
#define BOX3D_HEIGHTFIELD_H

#include "box3d_export.h"
#include "box3d/vec3.h"
#include "box3d/quat.h"
#include "box3d/collision.h"
#include "box3d/gjk.h"

#ifdef __cplusplus
extern "C" {
#endif

/* v1 simplification: static-only and no heap allocation, same rationale
 * as box3d/mesh.h. Grid vertices lie in the local XZ plane, spaced
 * `cell_size` apart and centered on `center`; `heights[row][col]` is the
 * local Y height at each grid vertex. Each cell's two triangles are
 * generated on demand (not stored) and tested via
 * box3d/mesh.h's b3_trianglemesh_triangle_vs_shape. */
#define B3_HEIGHTFIELD_MAX_ROWS 16
#define B3_HEIGHTFIELD_MAX_COLS 16

typedef struct {
    b3_Vec3 center;
    b3_Quat orientation;
    b3_real cell_size;
    b3_real heights[B3_HEIGHTFIELD_MAX_ROWS][B3_HEIGHTFIELD_MAX_COLS];
    int rows;
    int cols;
} b3_HeightField;

/* `row_major_heights` has rows*cols entries, row-major. Silently
 * truncates to B3_HEIGHTFIELD_MAX_ROWS/COLS if given more. */
B3_API b3_HeightField b3_heightfield_make(
    b3_Vec3 center, b3_Quat orientation, b3_real cell_size,
    const b3_real *row_major_heights, int rows, int cols
);

B3_API b3_Vec3 b3_heightfield_world_vertex(const b3_HeightField *hf, int row, int col);

B3_API void b3_heightfield_compute_aabb(const b3_HeightField *hf, b3_Vec3 *out_min, b3_Vec3 *out_max);

/* v1: always returns 0 -- see b3_trianglemesh_contains_point. */
B3_API int b3_heightfield_contains_point(const b3_HeightField *hf, b3_Vec3 point);

/* v1 approximation: tests the field's AABB, like b3_trianglemesh_raycast. */
B3_API b3_RayHit b3_heightfield_raycast(const b3_HeightField *hf, b3_Vec3 origin, b3_Vec3 dir, b3_real max_t);

/* Tests every grid cell's two triangles against a support-function
 * shape, quick-rejecting cells via `other_aabb_min`/`max`, keeping the
 * deepest overlap found. Mirrors b3_trianglemesh_overlap_shape. */
B3_API int b3_heightfield_overlap_shape(
    const b3_HeightField *hf, const void *other_shape, b3_SupportFn other_support,
    b3_Vec3 other_aabb_min, b3_Vec3 other_aabb_max, b3_ContactInfo *out
);

#ifdef __cplusplus
}
#endif

#endif /* BOX3D_HEIGHTFIELD_H */
