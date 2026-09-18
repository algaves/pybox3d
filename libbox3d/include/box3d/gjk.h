#ifndef BOX3D_GJK_H
#define BOX3D_GJK_H

#include "box3d_export.h"
#include "box3d/vec3.h"
#include "box3d/collision.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Generic convex-vs-convex intersection test (GJK) plus penetration
 * extraction (EPA), driven purely through a support-function interface --
 * no dependency on box3d/shape.h, so any convex shape kind (box, sphere,
 * capsule, hull, ...) can plug in via a small adapter. This is the
 * fallback path box3d/shape.h's b3_shape_overlap() uses for any pair that
 * isn't box-vs-box (which keeps using box3d.h's exact SAT instead). */
typedef b3_Vec3 (*b3_SupportFn)(const void *shape, b3_Vec3 direction);

typedef struct {
    const void *shape_a;
    b3_SupportFn support_a;
    const void *shape_b;
    b3_SupportFn support_b;
} b3_GjkInput;

/* Returns nonzero if the two shapes overlap and, when `out` is non-NULL,
 * fills in the penetration normal/depth (via EPA) and a representative
 * contact point (the midpoint of each shape's support point along the
 * normal -- the same v1 single-point approximation box3d.h's SAT path
 * uses, see b3_ContactInfo). */
B3_API int b3_gjk_epa_overlap(const b3_GjkInput *input, b3_ContactInfo *out);

#ifdef __cplusplus
}
#endif

#endif /* BOX3D_GJK_H */
