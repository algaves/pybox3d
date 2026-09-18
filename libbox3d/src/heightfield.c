#include "box3d/heightfield.h"
#include "box3d/box3d.h"
#include "box3d/mesh.h"

#include <math.h>

b3_HeightField b3_heightfield_make(
    b3_Vec3 center, b3_Quat orientation, b3_real cell_size,
    const b3_real *row_major_heights, int rows, int cols
) {
    b3_HeightField hf;
    hf.center = center;
    hf.orientation = b3_quat_normalize(orientation);
    hf.cell_size = cell_size;
    if (rows > B3_HEIGHTFIELD_MAX_ROWS) {
        rows = B3_HEIGHTFIELD_MAX_ROWS;
    }
    if (cols > B3_HEIGHTFIELD_MAX_COLS) {
        cols = B3_HEIGHTFIELD_MAX_COLS;
    }
    if (rows < 0) rows = 0;
    if (cols < 0) cols = 0;
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            hf.heights[r][c] = row_major_heights[r * cols + c];
        }
    }
    hf.rows = rows;
    hf.cols = cols;
    return hf;
}

b3_Vec3 b3_heightfield_world_vertex(const b3_HeightField *hf, int row, int col) {
    b3_real x = ((b3_real)col - (b3_real)(hf->cols - 1) * 0.5f) * hf->cell_size;
    b3_real z = ((b3_real)row - (b3_real)(hf->rows - 1) * 0.5f) * hf->cell_size;
    b3_real y = hf->heights[row][col];
    b3_Vec3 local = b3_vec3_make(x, y, z);
    return b3_vec3_add(hf->center, b3_quat_rotate_vec3(hf->orientation, local));
}

void b3_heightfield_compute_aabb(const b3_HeightField *hf, b3_Vec3 *out_min, b3_Vec3 *out_max) {
    if (hf->rows == 0 || hf->cols == 0) {
        *out_min = hf->center;
        *out_max = hf->center;
        return;
    }
    b3_Vec3 lo = b3_heightfield_world_vertex(hf, 0, 0);
    b3_Vec3 hi = lo;
    for (int r = 0; r < hf->rows; r++) {
        for (int c = 0; c < hf->cols; c++) {
            b3_Vec3 v = b3_heightfield_world_vertex(hf, r, c);
            lo.x = fminf(lo.x, v.x);
            lo.y = fminf(lo.y, v.y);
            lo.z = fminf(lo.z, v.z);
            hi.x = fmaxf(hi.x, v.x);
            hi.y = fmaxf(hi.y, v.y);
            hi.z = fmaxf(hi.z, v.z);
        }
    }
    *out_min = lo;
    *out_max = hi;
}

int b3_heightfield_contains_point(const b3_HeightField *hf, b3_Vec3 point) {
    (void)hf;
    (void)point;
    return 0;
}

b3_RayHit b3_heightfield_raycast(const b3_HeightField *hf, b3_Vec3 origin, b3_Vec3 dir, b3_real max_t) {
    b3_Vec3 lo, hi;
    b3_heightfield_compute_aabb(hf, &lo, &hi);
    b3_Vec3 half = b3_vec3_scale(b3_vec3_sub(hi, lo), 0.5f);
    b3_Vec3 center = b3_vec3_scale(b3_vec3_add(lo, hi), 0.5f);
    b3_Box3D aabb_box = b3_box3d_make_aabb(center, half);
    return b3_box3d_raycast(&aabb_box, origin, dir, max_t);
}

static void consider(
    const b3_Vec3 tri[3], const void *other_shape, b3_SupportFn other_support,
    b3_ContactInfo *best, int *found
) {
    b3_ContactInfo candidate;
    if (b3_trianglemesh_triangle_vs_shape(tri, other_shape, other_support, &candidate)) {
        if (!*found || candidate.penetration > best->penetration) {
            *best = candidate;
            *found = 1;
        }
    }
}

int b3_heightfield_overlap_shape(
    const b3_HeightField *hf, const void *other_shape, b3_SupportFn other_support,
    b3_Vec3 other_aabb_min, b3_Vec3 other_aabb_max, b3_ContactInfo *out
) {
    int found = 0;
    b3_ContactInfo best;

    for (int r = 0; r + 1 < hf->rows; r++) {
        for (int c = 0; c + 1 < hf->cols; c++) {
            b3_Vec3 p00 = b3_heightfield_world_vertex(hf, r, c);
            b3_Vec3 p01 = b3_heightfield_world_vertex(hf, r, c + 1);
            b3_Vec3 p10 = b3_heightfield_world_vertex(hf, r + 1, c);
            b3_Vec3 p11 = b3_heightfield_world_vertex(hf, r + 1, c + 1);

            b3_Vec3 clo = p00, chi = p00;
            b3_Vec3 rest[3] = {p01, p10, p11};
            for (int k = 0; k < 3; k++) {
                clo.x = fminf(clo.x, rest[k].x);
                clo.y = fminf(clo.y, rest[k].y);
                clo.z = fminf(clo.z, rest[k].z);
                chi.x = fmaxf(chi.x, rest[k].x);
                chi.y = fmaxf(chi.y, rest[k].y);
                chi.z = fmaxf(chi.z, rest[k].z);
            }
            if (clo.x > other_aabb_max.x || chi.x < other_aabb_min.x ||
                clo.y > other_aabb_max.y || chi.y < other_aabb_min.y ||
                clo.z > other_aabb_max.z || chi.z < other_aabb_min.z) {
                continue;
            }

            b3_Vec3 tri_a[3] = {p00, p10, p11};
            b3_Vec3 tri_b[3] = {p00, p11, p01};
            consider(tri_a, other_shape, other_support, &best, &found);
            consider(tri_b, other_shape, other_support, &best, &found);
        }
    }

    if (found && out) {
        *out = best;
    }
    return found;
}
