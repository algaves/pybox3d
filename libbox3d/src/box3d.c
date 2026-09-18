#include "box3d/box3d.h"

#include <math.h>
#include <string.h>

static const b3_real B3_AXIS_EPS = 1e-8f;

b3_Box3D b3_box3d_make_aabb(b3_Vec3 center, b3_Vec3 half_extents) {
    b3_Box3D box;
    box.center = center;
    box.half_extents = half_extents;
    box.orientation = b3_quat_identity();
    box.kind = B3_BOX_KIND_AABB;
    return box;
}

b3_Box3D b3_box3d_make_obb(b3_Vec3 center, b3_Vec3 half_extents, b3_Quat orientation) {
    b3_Box3D box;
    box.center = center;
    box.half_extents = half_extents;
    box.orientation = b3_quat_normalize(orientation);
    box.kind = B3_BOX_KIND_OBB;
    return box;
}

/* World-space unit vectors for the box's three local axes (columns of the
 * rotation matrix). Identity orientation yields the standard basis, which
 * makes this a correct no-op fast path for AABB-kind boxes too. */
static void box3d_axes(const b3_Box3D *box, b3_Vec3 axes[3]) {
    b3_real m[9];
    b3_quat_to_mat3(box->orientation, m);
    axes[0] = b3_vec3_make(m[0], m[3], m[6]);
    axes[1] = b3_vec3_make(m[1], m[4], m[7]);
    axes[2] = b3_vec3_make(m[2], m[5], m[8]);
}

void b3_box3d_compute_aabb(const b3_Box3D *box, b3_Vec3 *out_min, b3_Vec3 *out_max) {
    if (box->kind == B3_BOX_KIND_AABB) {
        *out_min = b3_vec3_sub(box->center, box->half_extents);
        *out_max = b3_vec3_add(box->center, box->half_extents);
        return;
    }

    b3_Vec3 axes[3];
    box3d_axes(box, axes);
    b3_real he[3] = {box->half_extents.x, box->half_extents.y, box->half_extents.z};
    b3_real axis_components[3][3] = {
        {axes[0].x, axes[0].y, axes[0].z},
        {axes[1].x, axes[1].y, axes[1].z},
        {axes[2].x, axes[2].y, axes[2].z},
    };

    b3_real extent[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 3; i++) {
        for (int k = 0; k < 3; k++) {
            extent[k] += he[i] * fabsf(axis_components[i][k]);
        }
    }

    b3_Vec3 world_extent = b3_vec3_make(extent[0], extent[1], extent[2]);
    *out_min = b3_vec3_sub(box->center, world_extent);
    *out_max = b3_vec3_add(box->center, world_extent);
}

b3_Vec3 b3_box3d_support(const b3_Box3D *box, b3_Vec3 direction) {
    b3_Vec3 axes[3];
    box3d_axes(box, axes);
    b3_real he[3] = {box->half_extents.x, box->half_extents.y, box->half_extents.z};
    b3_Vec3 dir = b3_vec3_normalize(direction);

    b3_Vec3 result = box->center;
    for (int i = 0; i < 3; i++) {
        /* Whenever `dir` is exactly (or nearly) perpendicular to this
         * axis, +he[i] and -he[i] are equally valid support points along
         * `dir` (both contribute ~0 to the dot product) -- the box's true
         * support set along this axis is the whole face, not one
         * arbitrary corner. Using their midpoint (0 offset) instead of
         * always picking +1 keeps this function a mathematically valid
         * GJK/EPA support (see box3d/hull.h's analogous fix for
         * ConvexHull), while avoiding a far, physically-unrepresentative
         * corner when this is used to place a contact point for a large
         * box (e.g. static ground) against a much smaller one -- see
         * b3_box3d_overlap and box3d/gjk.h's epa_contact_point. */
        b3_real d = b3_vec3_dot(axes[i], dir);
        b3_real sign = 0.0f;
        if (d > 1e-5f) {
            sign = 1.0f;
        } else if (d < -1e-5f) {
            sign = -1.0f;
        }
        result = b3_vec3_add(result, b3_vec3_scale(axes[i], he[i] * sign));
    }
    return result;
}

int b3_box3d_contains_point(const b3_Box3D *box, b3_Vec3 point) {
    b3_Vec3 local = b3_vec3_sub(point, box->center);
    if (box->kind == B3_BOX_KIND_OBB) {
        local = b3_quat_rotate_vec3(b3_quat_conjugate(box->orientation), local);
    }
    return fabsf(local.x) <= box->half_extents.x &&
           fabsf(local.y) <= box->half_extents.y &&
           fabsf(local.z) <= box->half_extents.z;
}

/* Tests a single candidate separating axis `axis` (need not be unit
 * length; the inequality below is scale-invariant for any nonzero axis).
 * Returns 1 if `axis` separates the boxes. When it doesn't separate them
 * and `axis` *is* unit length, `*overlap_out` receives the true-world-unit
 * overlap amount along it (used by callers restricted to face axes, where
 * the box axes are always unit length). */
static int axis_separates(
    b3_Vec3 axis,
    b3_Vec3 t,
    const b3_Vec3 a_axes[3], const b3_real a_he[3],
    const b3_Vec3 b_axes[3], const b3_real b_he[3],
    b3_real *overlap_out
) {
    if (b3_vec3_length_sq(axis) < B3_AXIS_EPS) {
        return 0; /* degenerate (near-parallel edges): not a valid test */
    }

    b3_real ra = 0.0f, rb = 0.0f;
    for (int i = 0; i < 3; i++) {
        ra += a_he[i] * fabsf(b3_vec3_dot(a_axes[i], axis));
        rb += b_he[i] * fabsf(b3_vec3_dot(b_axes[i], axis));
    }
    b3_real dist = fabsf(b3_vec3_dot(t, axis));

    if (overlap_out) {
        *overlap_out = (ra + rb) - dist;
    }
    return dist > ra + rb;
}

/* The specific edge of `box` running along its local axis `edge_axis`
 * that faces `axis` (i.e. the one whose other two local coordinates are
 * chosen to maximize the extent along `axis`) -- the edge actually
 * involved in an edge-edge contact along that separating axis. */
static void box3d_edge_segment(
    b3_Vec3 center, const b3_Vec3 axes[3], const b3_real he[3], int edge_axis, b3_Vec3 axis,
    b3_Vec3 *out_p0, b3_Vec3 *out_p1
) {
    b3_Vec3 base = center;
    for (int k = 0; k < 3; k++) {
        if (k == edge_axis) {
            continue;
        }
        b3_real sign = b3_vec3_dot(axes[k], axis) >= 0.0f ? 1.0f : -1.0f;
        base = b3_vec3_add(base, b3_vec3_scale(axes[k], he[k] * sign));
    }
    b3_Vec3 offset = b3_vec3_scale(axes[edge_axis], he[edge_axis]);
    *out_p0 = b3_vec3_sub(base, offset);
    *out_p1 = b3_vec3_add(base, offset);
}

static b3_real clamp01(b3_real v) {
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

/* Closest points between two line segments (Ericson, "Real-Time
 * Collision Detection" 5.1.9), used to place the contact point exactly
 * for an edge-edge configuration rather than approximating it via
 * support points (which, for two crossed edges, land on the far corners
 * rather than where the edges actually meet). */
static void closest_points_segment_segment(
    b3_Vec3 p1, b3_Vec3 q1, b3_Vec3 p2, b3_Vec3 q2, b3_Vec3 *c1, b3_Vec3 *c2
) {
    b3_Vec3 d1 = b3_vec3_sub(q1, p1);
    b3_Vec3 d2 = b3_vec3_sub(q2, p2);
    b3_Vec3 r = b3_vec3_sub(p1, p2);
    b3_real a = b3_vec3_dot(d1, d1);
    b3_real e = b3_vec3_dot(d2, d2);
    b3_real f = b3_vec3_dot(d2, r);

    b3_real s, t;
    if (a <= 1e-9f && e <= 1e-9f) {
        s = 0.0f;
        t = 0.0f;
    } else if (a <= 1e-9f) {
        s = 0.0f;
        t = clamp01(f / e);
    } else {
        b3_real c = b3_vec3_dot(d1, r);
        if (e <= 1e-9f) {
            t = 0.0f;
            s = clamp01(-c / a);
        } else {
            b3_real b = b3_vec3_dot(d1, d2);
            b3_real denom = a * e - b * b;
            s = denom != 0.0f ? clamp01((b * f - c * e) / denom) : 0.0f;
            t = (b * s + f) / e;
            if (t < 0.0f) {
                t = 0.0f;
                s = clamp01(-c / a);
            } else if (t > 1.0f) {
                t = 1.0f;
                s = clamp01((b - c) / a);
            }
        }
    }
    *c1 = b3_vec3_add(p1, b3_vec3_scale(d1, s));
    *c2 = b3_vec3_add(p2, b3_vec3_scale(d2, t));
}

int b3_box3d_overlap(const b3_Box3D *a, const b3_Box3D *b, b3_ContactInfo *out) {
    b3_Vec3 a_axes[3], b_axes[3];
    box3d_axes(a, a_axes);
    box3d_axes(b, b_axes);
    b3_real a_he[3] = {a->half_extents.x, a->half_extents.y, a->half_extents.z};
    b3_real b_he[3] = {b->half_extents.x, b->half_extents.y, b->half_extents.z};
    b3_Vec3 t = b3_vec3_sub(b->center, a->center);

    b3_real best_overlap = -1.0f;
    int best_is_a = 1, best_index = 0;
    int best_is_edge = 0, best_edge_i = 0, best_edge_j = 0;

    /* Face axes of A and B: real unit-length axes, so the overlap amount
     * computed here is directly usable as a penetration depth. */
    for (int i = 0; i < 3; i++) {
        b3_real overlap;
        if (axis_separates(a_axes[i], t, a_axes, a_he, b_axes, b_he, &overlap)) {
            return 0;
        }
        if (best_overlap < 0.0f || overlap < best_overlap) {
            best_overlap = overlap;
            best_is_a = 1;
            best_index = i;
            best_is_edge = 0;
        }
    }
    for (int i = 0; i < 3; i++) {
        b3_real overlap;
        if (axis_separates(b_axes[i], t, a_axes, a_he, b_axes, b_he, &overlap)) {
            return 0;
        }
        if (overlap < best_overlap) {
            best_overlap = overlap;
            best_is_a = 0;
            best_index = i;
            best_is_edge = 0;
        }
    }

    /* Edge-edge axes: cross(a_axes[i], b_axes[j]) isn't unit length in
     * general (it shrinks to zero as the two edges approach parallel),
     * so it's normalized before both the separation test and the
     * overlap comparison below -- otherwise the returned overlap amount
     * wouldn't be comparable to the face axes' world-unit values.
     * Whenever the exact edge-edge axis turns out to have the least
     * overlap of all 15 candidates, it -- not a face axis -- is the
     * true minimum-penetration direction, and is now reported as such
     * instead of always falling back to a face-based approximation. */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            b3_Vec3 axis = b3_vec3_normalize(b3_vec3_cross(a_axes[i], b_axes[j]));
            if (b3_vec3_length_sq(axis) < 0.5f) {
                continue; /* degenerate (near-parallel edges): not a valid axis */
            }
            b3_real overlap;
            if (axis_separates(axis, t, a_axes, a_he, b_axes, b_he, &overlap)) {
                return 0;
            }
            if (overlap < best_overlap) {
                best_overlap = overlap;
                best_is_edge = 1;
                best_edge_i = i;
                best_edge_j = j;
            }
        }
    }

    if (out) {
        b3_Vec3 normal;
        b3_Vec3 point;
        if (best_is_edge) {
            normal = b3_vec3_normalize(b3_vec3_cross(a_axes[best_edge_i], b_axes[best_edge_j]));
            if (b3_vec3_dot(normal, t) < 0.0f) {
                normal = b3_vec3_negate(normal);
            }
            b3_Vec3 a_p0, a_p1, b_p0, b_p1;
            box3d_edge_segment(a->center, a_axes, a_he, best_edge_i, normal, &a_p0, &a_p1);
            box3d_edge_segment(b->center, b_axes, b_he, best_edge_j, normal, &b_p0, &b_p1);
            b3_Vec3 ca, cb;
            closest_points_segment_segment(a_p0, a_p1, b_p0, b_p1, &ca, &cb);
            point = b3_vec3_scale(b3_vec3_add(ca, cb), 0.5f);
        } else {
            normal = best_is_a ? a_axes[best_index] : b_axes[best_index];
            if (b3_vec3_dot(normal, t) < 0.0f) {
                normal = b3_vec3_negate(normal);
            }
            b3_Vec3 pa = b3_box3d_support(a, normal);
            b3_Vec3 pb = b3_box3d_support(b, b3_vec3_negate(normal));
            point = b3_vec3_scale(b3_vec3_add(pa, pb), 0.5f);
        }
        out->normal = normal;
        out->penetration = best_overlap < 0.0f ? 0.0f : best_overlap;
        out->point = point;
    }
    return 1;
}

b3_RayHit b3_box3d_raycast(const b3_Box3D *box, b3_Vec3 origin, b3_Vec3 dir, b3_real max_t) {
    b3_RayHit miss;
    memset(&miss, 0, sizeof(miss));
    miss.hit = 0;

    b3_Vec3 local_origin = b3_vec3_sub(origin, box->center);
    b3_Vec3 local_dir = dir;
    if (box->kind == B3_BOX_KIND_OBB) {
        b3_Quat inv = b3_quat_conjugate(box->orientation);
        local_origin = b3_quat_rotate_vec3(inv, local_origin);
        local_dir = b3_quat_rotate_vec3(inv, local_dir);
    }

    b3_real he[3] = {box->half_extents.x, box->half_extents.y, box->half_extents.z};
    b3_real o[3] = {local_origin.x, local_origin.y, local_origin.z};
    b3_real d[3] = {local_dir.x, local_dir.y, local_dir.z};

    b3_real t_min = 0.0f, t_max = max_t;
    int hit_axis = -1, hit_sign = 1;

    for (int i = 0; i < 3; i++) {
        if (fabsf(d[i]) < 1e-9f) {
            if (o[i] < -he[i] || o[i] > he[i]) {
                return miss; /* parallel to slab and outside it */
            }
            continue;
        }
        b3_real inv_d = 1.0f / d[i];
        b3_real t_neg = (-he[i] - o[i]) * inv_d; /* plane x = -he[i], outward normal -axis */
        b3_real t_pos = (he[i] - o[i]) * inv_d;  /* plane x = +he[i], outward normal +axis */
        b3_real t_near, t_far;
        int near_sign;
        if (t_neg < t_pos) {
            t_near = t_neg;
            t_far = t_pos;
            near_sign = -1;
        } else {
            t_near = t_pos;
            t_far = t_neg;
            near_sign = 1;
        }
        if (t_near > t_min) {
            t_min = t_near;
            hit_axis = i;
            hit_sign = near_sign;
        }
        if (t_far < t_max) {
            t_max = t_far;
        }
        if (t_min > t_max) {
            return miss;
        }
    }

    if (hit_axis < 0) {
        /* Ray origin starts inside the box: report it as an immediate hit
         * at t=0 with no well-defined surface normal. */
        b3_RayHit hit;
        hit.hit = 1;
        hit.t = 0.0f;
        hit.point = origin;
        hit.normal = b3_vec3_zero();
        return hit;
    }

    b3_Vec3 local_normal = b3_vec3_zero();
    if (hit_axis == 0) local_normal.x = (b3_real)hit_sign;
    if (hit_axis == 1) local_normal.y = (b3_real)hit_sign;
    if (hit_axis == 2) local_normal.z = (b3_real)hit_sign;

    b3_Vec3 world_normal = local_normal;
    if (box->kind == B3_BOX_KIND_OBB) {
        world_normal = b3_quat_rotate_vec3(box->orientation, local_normal);
    }

    b3_RayHit hit;
    hit.hit = 1;
    hit.t = t_min;
    hit.point = b3_vec3_add(origin, b3_vec3_scale(dir, t_min));
    hit.normal = world_normal;
    return hit;
}
