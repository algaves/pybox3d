#ifndef BOX3D_CAPSULE_H
#define BOX3D_CAPSULE_H

#include "box3d_export.h"
#include "box3d/vec3.h"
#include "box3d/quat.h"
#include "box3d/collision.h"

#ifdef __cplusplus
extern "C" {
#endif

/* A capsule: a line segment of length 2*half_height along the body-local
 * +Y axis (rotated by `orientation`), swept by `radius`. */
typedef struct {
    b3_Vec3 center;
    b3_Quat orientation;
    b3_real radius;
    b3_real half_height; /* of the inner segment, not including the caps */
} b3_Capsule;

B3_API b3_Capsule b3_capsule_make(
    b3_Vec3 center, b3_Quat orientation, b3_real radius, b3_real half_height
);

/* The two inner segment endpoints in world space (the centers of the two
 * hemispherical caps). */
B3_API void b3_capsule_segment(const b3_Capsule *capsule, b3_Vec3 *out_p0, b3_Vec3 *out_p1);

B3_API void b3_capsule_compute_aabb(const b3_Capsule *capsule, b3_Vec3 *out_min, b3_Vec3 *out_max);
B3_API int b3_capsule_contains_point(const b3_Capsule *capsule, b3_Vec3 point);

/* The point on `capsule`'s surface farthest along `direction` (need not
 * be unit length). */
B3_API b3_Vec3 b3_capsule_support(const b3_Capsule *capsule, b3_Vec3 direction);

B3_API b3_RayHit b3_capsule_raycast(
    const b3_Capsule *capsule, b3_Vec3 origin, b3_Vec3 dir, b3_real max_t
);

#ifdef __cplusplus
}
#endif

#endif /* BOX3D_CAPSULE_H */
