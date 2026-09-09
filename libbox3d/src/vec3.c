#include "box3d/vec3.h"

#include <math.h>

b3_Vec3 b3_vec3_make(b3_real x, b3_real y, b3_real z) {
    b3_Vec3 v = {x, y, z};
    return v;
}

b3_Vec3 b3_vec3_zero(void) {
    return b3_vec3_make(0.0f, 0.0f, 0.0f);
}

b3_Vec3 b3_vec3_add(b3_Vec3 a, b3_Vec3 b) {
    return b3_vec3_make(a.x + b.x, a.y + b.y, a.z + b.z);
}

b3_Vec3 b3_vec3_sub(b3_Vec3 a, b3_Vec3 b) {
    return b3_vec3_make(a.x - b.x, a.y - b.y, a.z - b.z);
}

b3_Vec3 b3_vec3_scale(b3_Vec3 v, b3_real s) {
    return b3_vec3_make(v.x * s, v.y * s, v.z * s);
}

b3_Vec3 b3_vec3_negate(b3_Vec3 v) {
    return b3_vec3_make(-v.x, -v.y, -v.z);
}

b3_real b3_vec3_dot(b3_Vec3 a, b3_Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

b3_Vec3 b3_vec3_cross(b3_Vec3 a, b3_Vec3 b) {
    return b3_vec3_make(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

b3_real b3_vec3_length_sq(b3_Vec3 v) {
    return b3_vec3_dot(v, v);
}

b3_real b3_vec3_length(b3_Vec3 v) {
    return sqrtf(b3_vec3_length_sq(v));
}

b3_Vec3 b3_vec3_normalize(b3_Vec3 v) {
    b3_real len = b3_vec3_length(v);
    if (len < 1e-8f) {
        return b3_vec3_zero();
    }
    return b3_vec3_scale(v, 1.0f / len);
}

int b3_vec3_equal(b3_Vec3 a, b3_Vec3 b) {
    const b3_real eps = 1e-6f;
    return fabsf(a.x - b.x) < eps && fabsf(a.y - b.y) < eps && fabsf(a.z - b.z) < eps;
}
