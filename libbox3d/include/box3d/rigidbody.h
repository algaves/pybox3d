#ifndef BOX3D_RIGIDBODY_H
#define BOX3D_RIGIDBODY_H

#include "box3d_export.h"
#include "box3d/vec3.h"
#include "box3d/quat.h"
#include "box3d/box3d.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Plain-old-data rigid body scoped to a box shape. No internal heap
 * pointers: instances live inline in a b3_World's body array, or on the
 * stack/embedded in a caller-owned struct when used standalone. There is
 * no b3_rigidbody_create/destroy pair because there is nothing to
 * allocate. */
typedef struct {
    b3_Vec3 position;
    b3_Quat orientation;
    b3_Vec3 linear_velocity;
    b3_Vec3 angular_velocity;

    b3_real mass;
    b3_real inv_mass;
    b3_real inertia_local[3];     /* diagonal inertia tensor, body-local axes */
    b3_real inv_inertia_local[3];

    b3_Vec3 force_accum;
    b3_Vec3 torque_accum;

    b3_Box3D shape; /* kept in sync with position/orientation via b3_rigidbody_sync_shape */

    b3_real restitution;
    b3_real friction;

    void *user_data; /* reserved, unused in v1 */
} b3_RigidBody;

/* Initializes `body` in place as a box of the given half-extents centered
 * at `position` with identity orientation. `mass == 0` creates a static
 * (infinite-mass) body: inv_mass and inv_inertia_local are all zero and it
 * is never moved by integration or collision response. */
B3_API void b3_rigidbody_init_box(b3_RigidBody *body, b3_Vec3 position, b3_Vec3 half_extents, b3_real mass);

/* Recomputes inv_mass and the closed-form box inertia tensor (and its
 * inverse) for the body's current shape half-extents. mass == 0 marks the
 * body static. */
B3_API void b3_rigidbody_set_mass(b3_RigidBody *body, b3_real mass);

B3_API void b3_rigidbody_apply_force(b3_RigidBody *body, b3_Vec3 force, b3_Vec3 world_point);
B3_API void b3_rigidbody_apply_impulse(b3_RigidBody *body, b3_Vec3 impulse, b3_Vec3 world_point);

B3_API void b3_rigidbody_clear_accumulators(b3_RigidBody *body);

/* Copies position/orientation into body->shape so its collision shape
 * reflects the current pose. Called automatically by b3_world_step. */
B3_API void b3_rigidbody_sync_shape(b3_RigidBody *body);

/* Semi-implicit Euler integration step: applies `gravity` and the pending
 * force/torque accumulators to velocities, then integrates position and
 * orientation from those velocities, syncs the collision shape, and
 * clears the accumulators. A no-op for static bodies (inv_mass == 0).
 * Used by b3_world_step; also usable directly for a standalone body not
 * attached to any b3_World. */
B3_API void b3_rigidbody_integrate(b3_RigidBody *body, b3_Vec3 gravity, b3_real dt);

#ifdef __cplusplus
}
#endif

#endif /* BOX3D_RIGIDBODY_H */
