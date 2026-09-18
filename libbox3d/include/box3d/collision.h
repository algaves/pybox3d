#ifndef BOX3D_COLLISION_H
#define BOX3D_COLLISION_H

#include "box3d_export.h"
#include "box3d/vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Shared result types for any shape-vs-shape overlap/raycast test (box,
 * sphere, capsule, ...) -- not box-specific despite originating in
 * box3d.h. */

typedef struct {
    b3_Vec3 normal;      /* points from `a` towards `b`, unit length */
    b3_real penetration; /* >= 0 when the shapes overlap */
    /* v1 simplification: one representative world-space contact point
     * (the midpoint of each shape's support point along `normal`), not a
     * full multi-point manifold -- see docs/limitations.md. */
    b3_Vec3 point;
} b3_ContactInfo;

typedef struct {
    int hit;
    b3_real t;
    b3_Vec3 point;
    b3_Vec3 normal;
} b3_RayHit;

#ifdef __cplusplus
}
#endif

#endif /* BOX3D_COLLISION_H */
