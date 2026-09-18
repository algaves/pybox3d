#ifndef BOX3D_SHAPE_H
#define BOX3D_SHAPE_H

#include "box3d_export.h"
#include "box3d/vec3.h"
#include "box3d/quat.h"
#include "box3d/collision.h"
#include "box3d/box3d.h"
#include "box3d/sphere.h"
#include "box3d/capsule.h"
#include "box3d/hull.h"
#include "box3d/compound.h"
#include "box3d/mesh.h"
#include "box3d/heightfield.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    B3_SHAPE_BOX = 0,
    B3_SHAPE_SPHERE = 1,
    B3_SHAPE_CAPSULE = 2,
    B3_SHAPE_HULL = 3,
    B3_SHAPE_COMPOUND = 4,
    B3_SHAPE_MESH = 5,
    B3_SHAPE_HEIGHTFIELD = 6
} b3_ShapeKind;

/* A tagged union over every collision shape kind a b3_RigidBody can have.
 * Box-vs-box still goes through box3d.h's exact SAT; every other pair
 * goes through the generic GJK/EPA core (box3d/gjk.h) via b3_shape_support
 * -- see b3_shape_overlap. A b3_Compound's own children are a separate,
 * smaller "leaf" union (box3d/compound.h) that excludes B3_SHAPE_COMPOUND
 * -- see that header for why nesting isn't supported. */
typedef struct {
    b3_ShapeKind kind;
    union {
        b3_Box3D box;
        b3_Sphere sphere;
        b3_Capsule capsule;
        b3_ConvexHull hull;
        b3_Compound compound;
        b3_TriangleMesh mesh;
        b3_HeightField heightfield;
    } as;
} b3_Shape;

B3_API b3_Shape b3_shape_from_box(b3_Box3D box);
B3_API b3_Shape b3_shape_from_sphere(b3_Sphere sphere);
B3_API b3_Shape b3_shape_from_capsule(b3_Capsule capsule);
B3_API b3_Shape b3_shape_from_hull(b3_ConvexHull hull);
B3_API b3_Shape b3_shape_from_compound(b3_Compound compound);
B3_API b3_Shape b3_shape_from_mesh(b3_TriangleMesh mesh);
B3_API b3_Shape b3_shape_from_heightfield(b3_HeightField heightfield);

B3_API b3_Vec3 b3_shape_center(const b3_Shape *shape);

/* Repositions/reorients `shape` in place, preserving its local
 * kind-specific data (half-extents, radius, ...). Spheres ignore
 * `orientation` (rotation-invariant). */
B3_API void b3_shape_set_pose(b3_Shape *shape, b3_Vec3 center, b3_Quat orientation);

B3_API void b3_shape_compute_aabb(const b3_Shape *shape, b3_Vec3 *out_min, b3_Vec3 *out_max);
B3_API int b3_shape_contains_point(const b3_Shape *shape, b3_Vec3 point);

/* The point on `shape`'s surface farthest along `direction`. The common
 * interface every convex shape kind implements, used both directly and
 * as the GJK/EPA support function (box3d/gjk.h). */
B3_API b3_Vec3 b3_shape_support(const b3_Shape *shape, b3_Vec3 direction);

B3_API b3_RayHit b3_shape_raycast(const b3_Shape *shape, b3_Vec3 origin, b3_Vec3 dir, b3_real max_t);

/* Box-vs-box uses the exact 15-axis SAT (box3d/box3d.h); every other pair
 * of convex shape kinds uses GJK/EPA (box3d/gjk.h). See b3_ContactInfo
 * for the v1 single-contact-point caveat. */
B3_API int b3_shape_overlap(const b3_Shape *a, const b3_Shape *b, b3_ContactInfo *out);

#ifdef __cplusplus
}
#endif

#endif /* BOX3D_SHAPE_H */
