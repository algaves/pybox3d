#include "box3d/mesh.h"
#include "box3d/box3d.h"

#include <math.h>

b3_TriangleMesh b3_trianglemesh_make(
    b3_Vec3 center, b3_Quat orientation, const b3_Vec3 *local_triangle_vertices, int triangle_count
) {
    b3_TriangleMesh mesh;
    mesh.center = center;
    mesh.orientation = b3_quat_normalize(orientation);
    if (triangle_count > B3_MESH_MAX_TRIANGLES) {
        triangle_count = B3_MESH_MAX_TRIANGLES;
    }
    if (triangle_count < 0) {
        triangle_count = 0;
    }
    for (int i = 0; i < triangle_count * 3; i++) {
        mesh.local_vertices[i] = local_triangle_vertices[i];
    }
    mesh.triangle_count = triangle_count;
    return mesh;
}

void b3_trianglemesh_triangle_world(const b3_TriangleMesh *mesh, int triangle_index, b3_Vec3 out[3]) {
    for (int k = 0; k < 3; k++) {
        b3_Vec3 local = mesh->local_vertices[triangle_index * 3 + k];
        out[k] = b3_vec3_add(mesh->center, b3_quat_rotate_vec3(mesh->orientation, local));
    }
}

void b3_trianglemesh_compute_aabb(const b3_TriangleMesh *mesh, b3_Vec3 *out_min, b3_Vec3 *out_max) {
    if (mesh->triangle_count == 0) {
        *out_min = mesh->center;
        *out_max = mesh->center;
        return;
    }
    b3_Vec3 tri[3];
    b3_trianglemesh_triangle_world(mesh, 0, tri);
    b3_Vec3 lo = tri[0], hi = tri[0];
    for (int i = 0; i < mesh->triangle_count; i++) {
        b3_trianglemesh_triangle_world(mesh, i, tri);
        for (int k = 0; k < 3; k++) {
            b3_Vec3 v = tri[k];
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

int b3_trianglemesh_contains_point(const b3_TriangleMesh *mesh, b3_Vec3 point) {
    (void)mesh;
    (void)point;
    return 0;
}

b3_RayHit b3_trianglemesh_raycast(const b3_TriangleMesh *mesh, b3_Vec3 origin, b3_Vec3 dir, b3_real max_t) {
    b3_Vec3 lo, hi;
    b3_trianglemesh_compute_aabb(mesh, &lo, &hi);
    b3_Vec3 half = b3_vec3_scale(b3_vec3_sub(hi, lo), 0.5f);
    b3_Vec3 center = b3_vec3_scale(b3_vec3_add(lo, hi), 0.5f);
    b3_Box3D aabb_box = b3_box3d_make_aabb(center, half);
    return b3_box3d_raycast(&aabb_box, origin, dir, max_t);
}

/* Deliberately does *not* average tied vertices the way
 * b3_convexhull_support does for a compact, roughly-centered hull:
 * averaging is still a mathematically valid support point, but for a
 * large or elongated triangle (a common case for a mesh/height-field
 * "ground") it noticeably degrades EPA's convergence, since a point
 * that's merely *close to* the true maximum (an average of near-, not
 * exactly-, tied vertices) is no longer a true extreme point. Any
 * resulting "wrong corner" bias in the reported contact point is fixed
 * up separately below, in b3_trianglemesh_triangle_vs_shape. */
static b3_Vec3 triangle_support(const void *tri_ptr, b3_Vec3 direction) {
    const b3_Vec3 *tri = (const b3_Vec3 *)tri_ptr;
    int best = 0;
    b3_real best_dot = b3_vec3_dot(tri[0], direction);
    for (int i = 1; i < 3; i++) {
        b3_real d = b3_vec3_dot(tri[i], direction);
        if (d > best_dot) {
            best_dot = d;
            best = i;
        }
    }
    return tri[best];
}

/* Closest point on triangle `tri` to `p` (Ericson, "Real-Time Collision
 * Detection" 5.1.5). Used to place the reported contact point exactly:
 * triangle_support(normal) alone can land on a vertex far from the
 * other shape (e.g. any corner of a whole flat ground triangle, for a
 * query direction along its own face normal, regardless of where the
 * other shape actually sits on that triangle) -- clamping the *other*
 * shape's own witness point onto this triangle instead gives a point
 * that's actually within the contact region whenever one exists. */
static b3_Vec3 closest_point_on_triangle(b3_Vec3 p, const b3_Vec3 tri[3]) {
    b3_Vec3 a = tri[0], b = tri[1], c = tri[2];
    b3_Vec3 ab = b3_vec3_sub(b, a), ac = b3_vec3_sub(c, a), ap = b3_vec3_sub(p, a);
    b3_real d1 = b3_vec3_dot(ab, ap), d2 = b3_vec3_dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) {
        return a;
    }

    b3_Vec3 bp = b3_vec3_sub(p, b);
    b3_real d3 = b3_vec3_dot(ab, bp), d4 = b3_vec3_dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) {
        return b;
    }

    b3_real vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        b3_real v = d1 / (d1 - d3);
        return b3_vec3_add(a, b3_vec3_scale(ab, v));
    }

    b3_Vec3 cp = b3_vec3_sub(p, c);
    b3_real d5 = b3_vec3_dot(ab, cp), d6 = b3_vec3_dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) {
        return c;
    }

    b3_real vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        b3_real w = d2 / (d2 - d6);
        return b3_vec3_add(a, b3_vec3_scale(ac, w));
    }

    b3_real va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        b3_real w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b3_vec3_add(b, b3_vec3_scale(b3_vec3_sub(c, b), w));
    }

    b3_real denom = 1.0f / (va + vb + vc);
    b3_real v = vb * denom, w = vc * denom;
    return b3_vec3_add(a, b3_vec3_add(b3_vec3_scale(ab, v), b3_vec3_scale(ac, w)));
}

int b3_trianglemesh_triangle_vs_shape(
    const b3_Vec3 triangle[3], const void *other_shape, b3_SupportFn other_support, b3_ContactInfo *out
) {
    b3_GjkInput input;
    input.shape_a = triangle;
    input.support_a = triangle_support;
    input.shape_b = other_shape;
    input.support_b = other_support;
    int hit = b3_gjk_epa_overlap(&input, out);
    if (hit && out) {
        /* Re-place the contact point at the other shape's own witness
         * point (typically well-localized, e.g. exactly a sphere's
         * bottom), clamped onto this specific triangle. */
        b3_Vec3 other_witness = other_support(other_shape, b3_vec3_negate(out->normal));
        out->point = closest_point_on_triangle(other_witness, triangle);
    }
    return hit;
}

int b3_trianglemesh_overlap_shape(
    const b3_TriangleMesh *mesh, const void *other_shape, b3_SupportFn other_support,
    b3_Vec3 other_aabb_min, b3_Vec3 other_aabb_max, b3_ContactInfo *out
) {
    int found = 0;
    b3_ContactInfo best;
    for (int i = 0; i < mesh->triangle_count; i++) {
        b3_Vec3 tri[3];
        b3_trianglemesh_triangle_world(mesh, i, tri);

        b3_Vec3 tlo = tri[0], thi = tri[0];
        for (int k = 1; k < 3; k++) {
            tlo.x = fminf(tlo.x, tri[k].x);
            tlo.y = fminf(tlo.y, tri[k].y);
            tlo.z = fminf(tlo.z, tri[k].z);
            thi.x = fmaxf(thi.x, tri[k].x);
            thi.y = fmaxf(thi.y, tri[k].y);
            thi.z = fmaxf(thi.z, tri[k].z);
        }
        if (tlo.x > other_aabb_max.x || thi.x < other_aabb_min.x ||
            tlo.y > other_aabb_max.y || thi.y < other_aabb_min.y ||
            tlo.z > other_aabb_max.z || thi.z < other_aabb_min.z) {
            continue;
        }

        b3_ContactInfo candidate;
        if (b3_trianglemesh_triangle_vs_shape(tri, other_shape, other_support, &candidate)) {
            if (!found || candidate.penetration > best.penetration) {
                best = candidate;
                found = 1;
            }
        }
    }
    if (found && out) {
        *out = best;
    }
    return found;
}
