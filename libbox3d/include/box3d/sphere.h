#ifndef BOX3D_SPHERE_H
#define BOX3D_SPHERE_H

#include "box3d_export.h"
#include "box3d/vec3.h"
#include "box3d/collision.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    b3_Vec3 center;
    b3_real radius;
} b3_Sphere;

B3_API b3_Sphere b3_sphere_make(b3_Vec3 center, b3_real radius);

B3_API void b3_sphere_compute_aabb(const b3_Sphere *sphere, b3_Vec3 *out_min, b3_Vec3 *out_max);
B3_API int b3_sphere_contains_point(const b3_Sphere *sphere, b3_Vec3 point);

/* The point on `sphere`'s surface farthest along `direction` (need not be
 * unit length; the zero vector maps to `center`). */
B3_API b3_Vec3 b3_sphere_support(const b3_Sphere *sphere, b3_Vec3 direction);

B3_API b3_RayHit b3_sphere_raycast(const b3_Sphere *sphere, b3_Vec3 origin, b3_Vec3 dir, b3_real max_t);

#ifdef __cplusplus
}
#endif

#endif /* BOX3D_SPHERE_H */
