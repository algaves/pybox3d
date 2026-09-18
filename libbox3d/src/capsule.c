#include "box3d/capsule.h"

#include <math.h>
#include <string.h>

b3_Capsule b3_capsule_make(
    b3_Vec3 center, b3_Quat orientation, b3_real radius, b3_real half_height
) {
    b3_Capsule capsule;
    capsule.center = center;
    capsule.orientation = b3_quat_normalize(orientation);
    capsule.radius = radius;
    capsule.half_height = half_height;
    return capsule;
}

void b3_capsule_segment(const b3_Capsule *capsule, b3_Vec3 *out_p0, b3_Vec3 *out_p1) {
    b3_Vec3 axis = b3_quat_rotate_vec3(capsule->orientation, b3_vec3_make(0.0f, 1.0f, 0.0f));
    b3_Vec3 offset = b3_vec3_scale(axis, capsule->half_height);
    *out_p0 = b3_vec3_sub(capsule->center, offset);
    *out_p1 = b3_vec3_add(capsule->center, offset);
}

void b3_capsule_compute_aabb(const b3_Capsule *capsule, b3_Vec3 *out_min, b3_Vec3 *out_max) {
    b3_Vec3 p0, p1;
    b3_capsule_segment(capsule, &p0, &p1);
    b3_Vec3 r = b3_vec3_make(capsule->radius, capsule->radius, capsule->radius);
    b3_Vec3 lo0 = b3_vec3_sub(p0, r), hi0 = b3_vec3_add(p0, r);
    b3_Vec3 lo1 = b3_vec3_sub(p1, r), hi1 = b3_vec3_add(p1, r);
    *out_min = b3_vec3_make(fminf(lo0.x, lo1.x), fminf(lo0.y, lo1.y), fminf(lo0.z, lo1.z));
    *out_max = b3_vec3_make(fmaxf(hi0.x, hi1.x), fmaxf(hi0.y, hi1.y), fmaxf(hi0.z, hi1.z));
}

/* Closest point on segment p0-p1 to `point`. */
static b3_Vec3 closest_point_on_segment(b3_Vec3 p0, b3_Vec3 p1, b3_Vec3 point) {
    b3_Vec3 d = b3_vec3_sub(p1, p0);
    b3_real len_sq = b3_vec3_length_sq(d);
    if (len_sq < 1e-12f) {
        return p0;
    }
    b3_real t = b3_vec3_dot(b3_vec3_sub(point, p0), d) / len_sq;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return b3_vec3_add(p0, b3_vec3_scale(d, t));
}

int b3_capsule_contains_point(const b3_Capsule *capsule, b3_Vec3 point) {
    b3_Vec3 p0, p1;
    b3_capsule_segment(capsule, &p0, &p1);
    b3_Vec3 closest = closest_point_on_segment(p0, p1, point);
    b3_Vec3 delta = b3_vec3_sub(point, closest);
    return b3_vec3_length_sq(delta) <= capsule->radius * capsule->radius;
}

b3_Vec3 b3_capsule_support(const b3_Capsule *capsule, b3_Vec3 direction) {
    b3_Vec3 p0, p1;
    b3_capsule_segment(capsule, &p0, &p1);
    b3_Vec3 axis = b3_vec3_sub(p1, p0);
    b3_Vec3 base = b3_vec3_dot(axis, direction) >= 0.0f ? p1 : p0;
    b3_Vec3 dir = b3_vec3_normalize(direction);
    return b3_vec3_add(base, b3_vec3_scale(dir, capsule->radius));
}

static b3_RayHit ray_sphere(b3_Vec3 center, b3_real radius, b3_Vec3 origin, b3_Vec3 dir, b3_real max_t) {
    b3_RayHit miss;
    memset(&miss, 0, sizeof(miss));
    miss.hit = 0;

    b3_Vec3 m = b3_vec3_sub(origin, center);
    b3_real b = b3_vec3_dot(m, dir);
    b3_real c = b3_vec3_dot(m, m) - radius * radius;
    if (c > 0.0f && b > 0.0f) {
        return miss;
    }
    b3_real a = b3_vec3_dot(dir, dir);
    if (a < 1e-12f) {
        return miss;
    }
    b3_real discr = b * b - a * c;
    if (discr < 0.0f) {
        return miss;
    }
    b3_real t = (-b - sqrtf(discr)) / a;
    if (t < 0.0f) t = 0.0f;
    if (t > max_t) {
        return miss;
    }
    b3_RayHit hit;
    hit.hit = 1;
    hit.t = t;
    hit.point = b3_vec3_add(origin, b3_vec3_scale(dir, t));
    hit.normal = b3_vec3_normalize(b3_vec3_sub(hit.point, center));
    return hit;
}

/* Ray vs. capsule: test the infinite cylinder around the segment, clip
 * the hit to the segment's extent, and fall back to a sphere test at
 * whichever cap the clipped hit falls beyond. Standard technique, e.g.
 * Ericson, "Real-Time Collision Detection" 5.3.7. */
b3_RayHit b3_capsule_raycast(const b3_Capsule *capsule, b3_Vec3 origin, b3_Vec3 dir, b3_real max_t) {
    b3_RayHit miss;
    memset(&miss, 0, sizeof(miss));
    miss.hit = 0;

    b3_Vec3 p0, p1;
    b3_capsule_segment(capsule, &p0, &p1);
    b3_real r = capsule->radius;

    b3_Vec3 d = b3_vec3_sub(p1, p0);
    b3_Vec3 m = b3_vec3_sub(origin, p0);
    b3_Vec3 n = dir;

    b3_real md = b3_vec3_dot(m, d);
    b3_real nd = b3_vec3_dot(n, d);
    b3_real dd = b3_vec3_dot(d, d);

    if (dd < 1e-12f) {
        /* Degenerate (half_height ~ 0): just a sphere at the center. */
        return ray_sphere(capsule->center, r, origin, dir, max_t);
    }

    if (md < 0.0f && md + nd < 0.0f) {
        return ray_sphere(p0, r, origin, dir, max_t);
    }
    if (md > dd && md + nd > dd) {
        return ray_sphere(p1, r, origin, dir, max_t);
    }

    b3_real nn = b3_vec3_dot(n, n);
    b3_real mn = b3_vec3_dot(m, n);
    b3_real a = dd * nn - nd * nd;
    b3_real k = b3_vec3_dot(m, m) - r * r;
    b3_real c = dd * k - md * md;

    if (fabsf(a) < 1e-9f) {
        /* Ray parallel to the capsule axis. */
        if (c > 0.0f) {
            return miss; /* outside the infinite cylinder, never enters */
        }
        if (md < 0.0f) {
            return ray_sphere(p0, r, origin, dir, max_t);
        }
        if (md > dd) {
            return ray_sphere(p1, r, origin, dir, max_t);
        }
        b3_RayHit hit;
        hit.hit = 1;
        hit.t = 0.0f;
        hit.point = origin;
        hit.normal = b3_vec3_zero();
        return hit;
    }

    b3_real b = dd * mn - nd * md;
    b3_real discr = b * b - a * c;
    if (discr < 0.0f) {
        return miss;
    }
    b3_real t = (-b - sqrtf(discr)) / a;
    if (t < 0.0f) {
        t = 0.0f;
    }

    b3_real s = md + t * nd; /* position along the axis at the hit */
    if (s < 0.0f) {
        return ray_sphere(p0, r, origin, dir, max_t);
    }
    if (s > dd) {
        return ray_sphere(p1, r, origin, dir, max_t);
    }
    if (t > max_t) {
        return miss;
    }

    b3_Vec3 point = b3_vec3_add(origin, b3_vec3_scale(dir, t));
    b3_Vec3 axis_point = b3_vec3_add(p0, b3_vec3_scale(d, s / dd));
    b3_RayHit hit;
    hit.hit = 1;
    hit.t = t;
    hit.point = point;
    hit.normal = b3_vec3_normalize(b3_vec3_sub(point, axis_point));
    return hit;
}
