#ifndef BOX3D_QUAT_H
#define BOX3D_QUAT_H

#include "box3d_export.h"
#include "box3d/vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    b3_real x, y, z, w;
} b3_Quat;

B3_API b3_Quat b3_quat_make(b3_real x, b3_real y, b3_real z, b3_real w);
B3_API b3_Quat b3_quat_identity(void);
B3_API b3_Quat b3_quat_from_axis_angle(b3_Vec3 axis, b3_real angle_radians);

B3_API b3_Quat b3_quat_mul(b3_Quat a, b3_Quat b);
B3_API b3_Quat b3_quat_normalize(b3_Quat q);
B3_API b3_Quat b3_quat_conjugate(b3_Quat q);

B3_API b3_Vec3 b3_quat_rotate_vec3(b3_Quat q, b3_Vec3 v);

/* Row-major 3x3 rotation matrix, out[0..2] = row 0, out[3..5] = row 1, ... */
B3_API void b3_quat_to_mat3(b3_Quat q, b3_real out[9]);

/* Semi-implicit orientation integration given an angular velocity (world
 * frame, rad/s) and a timestep; returns a normalized result. */
B3_API b3_Quat b3_quat_integrate(b3_Quat q, b3_Vec3 angular_velocity, b3_real dt);

#ifdef __cplusplus
}
#endif

#endif /* BOX3D_QUAT_H */
