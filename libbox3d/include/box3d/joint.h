#ifndef BOX3D_JOINT_H
#define BOX3D_JOINT_H

#include "box3d_export.h"
#include "box3d/vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

/* A rigid distance constraint between the centers of two bodies (indices
 * into a b3_World's body array): b3_world_step holds the distance between
 * their positions at `rest_length` as a bilateral constraint (a stiff rod,
 * not a spring -- no softness/limits).
 *
 * v1 scope cut: anchored at body centers only, no per-body local anchor
 * offset. Like contact resolution, this means no angular/torque
 * contribution -- see README limitations. */
typedef struct {
    int body_a_index;
    int body_b_index;
    b3_real rest_length;
} b3_DistanceJoint;

#ifdef __cplusplus
}
#endif

#endif /* BOX3D_JOINT_H */
