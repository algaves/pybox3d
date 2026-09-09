#ifndef BOX3D_VEC3_H
#define BOX3D_VEC3_H

#include "box3d_export.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef float b3_real;

typedef struct {
    b3_real x, y, z;
} b3_Vec3;

B3_API b3_Vec3 b3_vec3_make(b3_real x, b3_real y, b3_real z);
B3_API b3_Vec3 b3_vec3_zero(void);

B3_API b3_Vec3 b3_vec3_add(b3_Vec3 a, b3_Vec3 b);
B3_API b3_Vec3 b3_vec3_sub(b3_Vec3 a, b3_Vec3 b);
B3_API b3_Vec3 b3_vec3_scale(b3_Vec3 v, b3_real s);
B3_API b3_Vec3 b3_vec3_negate(b3_Vec3 v);

B3_API b3_real b3_vec3_dot(b3_Vec3 a, b3_Vec3 b);
B3_API b3_Vec3 b3_vec3_cross(b3_Vec3 a, b3_Vec3 b);

B3_API b3_real b3_vec3_length_sq(b3_Vec3 v);
B3_API b3_real b3_vec3_length(b3_Vec3 v);

/* Returns the zero vector if v is (numerically) the zero vector, rather
 * than dividing by zero. */
B3_API b3_Vec3 b3_vec3_normalize(b3_Vec3 v);

B3_API int b3_vec3_equal(b3_Vec3 a, b3_Vec3 b);

#ifdef __cplusplus
}
#endif

#endif /* BOX3D_VEC3_H */
