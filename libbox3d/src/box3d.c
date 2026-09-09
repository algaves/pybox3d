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

int b3_box3d_overlap(const b3_Box3D *a, const b3_Box3D *b, b3_ContactInfo *out) {
    b3_Vec3 a_axes[3], b_axes[3];
    box3d_axes(a, a_axes);
    box3d_axes(b, b_axes);
    b3_real a_he[3] = {a->half_extents.x, a->half_extents.y, a->half_extents.z};
    b3_real b_he[3] = {b->half_extents.x, b->half_extents.y, b->half_extents.z};
    b3_Vec3 t = b3_vec3_sub(b->center, a->center);

    b3_real best_overlap = -1.0f;
    int best_is_a = 1, best_index = 0;

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
        }
    }

    /* Edge-edge axes: only used to rule out separation, per this library's
     * documented v1 scope (see box3d.h). */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            b3_Vec3 axis = b3_vec3_cross(a_axes[i], b_axes[j]);
            if (axis_separates(axis, t, a_axes, a_he, b_axes, b_he, NULL)) {
                return 0;
            }
        }
    }

    if (out) {
        b3_Vec3 normal = best_is_a ? a_axes[best_index] : b_axes[best_index];
        /* Orient normal to point from a towards b. */
        if (b3_vec3_dot(normal, t) < 0.0f) {
            normal = b3_vec3_negate(normal);
        }
        out->normal = normal;
        out->penetration = best_overlap < 0.0f ? 0.0f : best_overlap;
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
