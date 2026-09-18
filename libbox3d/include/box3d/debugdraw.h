#ifndef BOX3D_DEBUGDRAW_H
#define BOX3D_DEBUGDRAW_H

#include "box3d_export.h"
#include "box3d/vec3.h"
#include "box3d/shape.h"
#include "box3d/world.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Pure data, no rendering: a world-space line segment, meant to be fed
 * to whatever rendering backend the caller already has (this library has
 * none, by design -- see docs/limitations.md). */
typedef struct {
    b3_Vec3 a;
    b3_Vec3 b;
} b3_DebugLine;

/* Writes up to `max_lines` line segments approximating `shape`'s
 * wireframe into `out_lines`, and returns the total number of segments
 * this shape produces (may exceed `max_lines` -- same overflow-
 * truncation semantics as b3_world_query_aabb; pass NULL/0 to just get a
 * count first). Box gets its exact 12 edges; Sphere/Capsule are
 * approximated with circles (B3_DEBUG_CIRCLE_SEGMENTS-sided polygons);
 * ConvexHull draws its AABB instead of an exact wireframe (v1 has no
 * computed edge topology for an arbitrary vertex set -- see
 * box3d/hull.h); Compound recurses into each child; TriangleMesh draws
 * each triangle's 3 edges; HeightField draws its grid lines. */
B3_API int b3_shape_debug_lines(const b3_Shape *shape, b3_DebugLine *out_lines, int max_lines);

/* Number of segments b3_shape_debug_lines uses to approximate a round
 * shape's circular cross-sections. */
#define B3_DEBUG_CIRCLE_SEGMENTS 16

/* One representative world-space contact point/normal per currently-
 * overlapping body pair in `world`, from a fresh O(n^2) discrete scan --
 * independent of (and redundant with) b3_world_step's own sort-and-sweep
 * broad phase, since this is a debug/tooling query meant to be called
 * for visualization, not every step of a performance-critical
 * simulation. Unlike b3_world_step, this does *not* skip pairs connected
 * by a Filter joint -- it reports every raw geometric overlap regardless
 * of collision filtering, which is usually more useful for debugging.
 * Same overflow-truncation semantics as b3_shape_debug_lines. */
typedef struct {
    b3_Vec3 point;
    b3_Vec3 normal;
} b3_DebugContact;
B3_API int b3_world_debug_contacts(
    const b3_World *world, b3_DebugContact *out_contacts, int max_contacts
);

/* World-space anchor_a/anchor_b for every joint in `world` (in the same
 * order as index 0..joint_count-1 -- pair each with b3_world_get_joint's
 * `.kind` if you want to skip Filter/Motor/Parallel joints, whose
 * anchor_a/anchor_b fields are unused and left at the zero offset, so
 * this reports each body's own center for them rather than a meaningful
 * anchor point). Same overflow-truncation semantics. */
typedef struct {
    b3_Vec3 anchor_a;
    b3_Vec3 anchor_b;
} b3_DebugJointAnchor;
B3_API int b3_world_debug_joint_anchors(
    const b3_World *world, b3_DebugJointAnchor *out_anchors, int max_anchors
);

#ifdef __cplusplus
}
#endif

#endif /* BOX3D_DEBUGDRAW_H */
