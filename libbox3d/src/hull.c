#include "box3d/hull.h"
#include "box3d/box3d.h"
#include "box3d/gjk.h"

#include <math.h>
#include <stddef.h>

b3_ConvexHull b3_convexhull_make(
    b3_Vec3 center, b3_Quat orientation, const b3_Vec3 *local_vertices, int vertex_count
) {
    b3_ConvexHull hull;
    hull.center = center;
    hull.orientation = b3_quat_normalize(orientation);
    if (vertex_count > B3_HULL_MAX_VERTICES) {
        vertex_count = B3_HULL_MAX_VERTICES;
    }
    if (vertex_count < 0) {
        vertex_count = 0;
    }
    for (int i = 0; i < vertex_count; i++) {
        hull.local_vertices[i] = local_vertices[i];
    }
    hull.vertex_count = vertex_count;
    return hull;
}

b3_Vec3 b3_convexhull_world_vertex(const b3_ConvexHull *hull, int index) {
    return b3_vec3_add(hull->center, b3_quat_rotate_vec3(hull->orientation, hull->local_vertices[index]));
}

void b3_convexhull_compute_aabb(const b3_ConvexHull *hull, b3_Vec3 *out_min, b3_Vec3 *out_max) {
    if (hull->vertex_count == 0) {
        *out_min = hull->center;
        *out_max = hull->center;
        return;
    }
    b3_Vec3 lo = b3_convexhull_world_vertex(hull, 0);
    b3_Vec3 hi = lo;
    for (int i = 1; i < hull->vertex_count; i++) {
        b3_Vec3 v = b3_convexhull_world_vertex(hull, i);
        lo.x = fminf(lo.x, v.x);
        lo.y = fminf(lo.y, v.y);
        lo.z = fminf(lo.z, v.z);
        hi.x = fmaxf(hi.x, v.x);
        hi.y = fmaxf(hi.y, v.y);
        hi.z = fmaxf(hi.z, v.z);
    }
    *out_min = lo;
    *out_max = hi;
}

b3_Vec3 b3_convexhull_support(const b3_ConvexHull *hull, b3_Vec3 direction) {
    if (hull->vertex_count == 0) {
        return hull->center;
    }
    /* Rotate the direction into local space once, rather than rotating
     * every vertex into world space just to scan them. */
    b3_Quat inv = b3_quat_conjugate(hull->orientation);
    b3_Vec3 local_dir = b3_quat_rotate_vec3(inv, direction);

    b3_real dots[B3_HULL_MAX_VERTICES];
    b3_real best_dot = b3_vec3_dot(hull->local_vertices[0], local_dir);
    dots[0] = best_dot;
    for (int i = 1; i < hull->vertex_count; i++) {
        dots[i] = b3_vec3_dot(hull->local_vertices[i], local_dir);
        if (dots[i] > best_dot) {
            best_dot = dots[i];
        }
    }

    /* Whenever `direction` is exactly (or nearly) normal to a face or
     * edge, several vertices tie for the maximum -- the true support is
     * that whole face/edge, not any single one of its corners.
     * Arbitrarily picking the first tied vertex biases GJK/EPA's search
     * in a way that's especially visible against a smooth shape (a
     * sphere/capsule) on an axis-aligned hull; averaging the tied
     * vertices instead is the standard fix and keeps the result stable
     * regardless of vertex order. */
    const b3_real tie_eps = 1e-5f * (fabsf(best_dot) + 1.0f);
    b3_Vec3 sum = b3_vec3_zero();
    int count = 0;
    for (int i = 0; i < hull->vertex_count; i++) {
        if (dots[i] >= best_dot - tie_eps) {
            sum = b3_vec3_add(sum, hull->local_vertices[i]);
            count++;
        }
    }
    b3_Vec3 local_point = b3_vec3_scale(sum, 1.0f / (b3_real)count);
    return b3_vec3_add(hull->center, b3_quat_rotate_vec3(hull->orientation, local_point));
}

static b3_Vec3 hull_support_adapter(const void *shape, b3_Vec3 direction) {
    return b3_convexhull_support((const b3_ConvexHull *)shape, direction);
}

static b3_Vec3 point_support_adapter(const void *point, b3_Vec3 direction) {
    (void)direction;
    return *(const b3_Vec3 *)point;
}

int b3_convexhull_contains_point(const b3_ConvexHull *hull, b3_Vec3 point) {
    /* Point-in-convex-hull containment is exactly a GJK intersection test
     * between the hull and a zero-radius "point shape" whose support
     * function always returns the point itself. */
    b3_GjkInput input;
    input.shape_a = hull;
    input.support_a = hull_support_adapter;
    input.shape_b = &point;
    input.support_b = point_support_adapter;
    return b3_gjk_epa_overlap(&input, NULL);
}

b3_RayHit b3_convexhull_raycast(const b3_ConvexHull *hull, b3_Vec3 origin, b3_Vec3 dir, b3_real max_t) {
    /* v1 approximation: without face data, an exact hull-surface raycast
     * isn't available (see box3d/hull.h), so this tests the hull's AABB
     * instead -- see docs/limitations.md. */
    b3_Vec3 lo, hi;
    b3_convexhull_compute_aabb(hull, &lo, &hi);
    b3_Vec3 half = b3_vec3_scale(b3_vec3_sub(hi, lo), 0.5f);
    b3_Vec3 center = b3_vec3_scale(b3_vec3_add(lo, hi), 0.5f);
    b3_Box3D aabb_box = b3_box3d_make_aabb(center, half);
    return b3_box3d_raycast(&aabb_box, origin, dir, max_t);
}
