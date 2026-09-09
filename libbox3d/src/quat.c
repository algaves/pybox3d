#include "box3d/quat.h"

#include <math.h>

b3_Quat b3_quat_make(b3_real x, b3_real y, b3_real z, b3_real w) {
    b3_Quat q = {x, y, z, w};
    return q;
}

b3_Quat b3_quat_identity(void) {
    return b3_quat_make(0.0f, 0.0f, 0.0f, 1.0f);
}

b3_Quat b3_quat_from_axis_angle(b3_Vec3 axis, b3_real angle_radians) {
    b3_Vec3 n = b3_vec3_normalize(axis);
    b3_real half = angle_radians * 0.5f;
    b3_real s = sinf(half);
    return b3_quat_make(n.x * s, n.y * s, n.z * s, cosf(half));
}

b3_Quat b3_quat_mul(b3_Quat a, b3_Quat b) {
    return b3_quat_make(
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    );
}

b3_Quat b3_quat_normalize(b3_Quat q) {
    b3_real len = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (len < 1e-8f) {
        return b3_quat_identity();
    }
    b3_real inv = 1.0f / len;
    return b3_quat_make(q.x * inv, q.y * inv, q.z * inv, q.w * inv);
}

b3_Quat b3_quat_conjugate(b3_Quat q) {
    return b3_quat_make(-q.x, -q.y, -q.z, q.w);
}

b3_Vec3 b3_quat_rotate_vec3(b3_Quat q, b3_Vec3 v) {
    /* t = 2 * cross(q.xyz, v); result = v + q.w * t + cross(q.xyz, t) */
    b3_Vec3 qv = b3_vec3_make(q.x, q.y, q.z);
    b3_Vec3 t = b3_vec3_scale(b3_vec3_cross(qv, v), 2.0f);
    b3_Vec3 result = b3_vec3_add(v, b3_vec3_scale(t, q.w));
    result = b3_vec3_add(result, b3_vec3_cross(qv, t));
    return result;
}

void b3_quat_to_mat3(b3_Quat q, b3_real out[9]) {
    b3_real x = q.x, y = q.y, z = q.z, w = q.w;
    b3_real x2 = x + x, y2 = y + y, z2 = z + z;
    b3_real xx = x * x2, xy = x * y2, xz = x * z2;
    b3_real yy = y * y2, yz = y * z2, zz = z * z2;
    b3_real wx = w * x2, wy = w * y2, wz = w * z2;

    out[0] = 1.0f - (yy + zz);
    out[1] = xy - wz;
    out[2] = xz + wy;

    out[3] = xy + wz;
    out[4] = 1.0f - (xx + zz);
    out[5] = yz - wx;

    out[6] = xz - wy;
    out[7] = yz + wx;
    out[8] = 1.0f - (xx + yy);
}

b3_Quat b3_quat_integrate(b3_Quat q, b3_Vec3 angular_velocity, b3_real dt) {
    b3_Quat omega = b3_quat_make(
        angular_velocity.x, angular_velocity.y, angular_velocity.z, 0.0f
    );
    b3_Quat delta = b3_quat_mul(omega, q);
    b3_Quat result = b3_quat_make(
        q.x + 0.5f * dt * delta.x,
        q.y + 0.5f * dt * delta.y,
        q.z + 0.5f * dt * delta.z,
        q.w + 0.5f * dt * delta.w
    );
    return b3_quat_normalize(result);
}
