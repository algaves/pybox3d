#include "box3d/sphere.h"

#include <math.h>
#include <string.h>

b3_Sphere b3_sphere_make(b3_Vec3 center, b3_real radius) {
    b3_Sphere sphere;
    sphere.center = center;
    sphere.radius = radius;
    return sphere;
}

void b3_sphere_compute_aabb(const b3_Sphere *sphere, b3_Vec3 *out_min, b3_Vec3 *out_max) {
    b3_Vec3 r = b3_vec3_make(sphere->radius, sphere->radius, sphere->radius);
    *out_min = b3_vec3_sub(sphere->center, r);
    *out_max = b3_vec3_add(sphere->center, r);
}

int b3_sphere_contains_point(const b3_Sphere *sphere, b3_Vec3 point) {
    b3_Vec3 delta = b3_vec3_sub(point, sphere->center);
    return b3_vec3_length_sq(delta) <= sphere->radius * sphere->radius;
}

b3_Vec3 b3_sphere_support(const b3_Sphere *sphere, b3_Vec3 direction) {
    b3_Vec3 dir = b3_vec3_normalize(direction);
    return b3_vec3_add(sphere->center, b3_vec3_scale(dir, sphere->radius));
}

b3_RayHit b3_sphere_raycast(const b3_Sphere *sphere, b3_Vec3 origin, b3_Vec3 dir, b3_real max_t) {
    b3_RayHit miss;
    memset(&miss, 0, sizeof(miss));
    miss.hit = 0;

    b3_Vec3 m = b3_vec3_sub(origin, sphere->center);
    b3_real b = b3_vec3_dot(m, dir);
    b3_real c = b3_vec3_dot(m, m) - sphere->radius * sphere->radius;

    /* Ray origin outside the sphere and pointing away: no hit. */
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
    if (t < 0.0f) {
        t = 0.0f; /* origin is inside the sphere */
    }
    if (t > max_t) {
        return miss;
    }

    b3_RayHit hit;
    hit.hit = 1;
    hit.t = t;
    hit.point = b3_vec3_add(origin, b3_vec3_scale(dir, t));
    hit.normal = b3_vec3_normalize(b3_vec3_sub(hit.point, sphere->center));
    return hit;
}
