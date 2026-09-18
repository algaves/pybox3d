#ifndef BOX3D_RIGIDBODY_H
#define BOX3D_RIGIDBODY_H

#include "box3d_export.h"
#include "box3d/vec3.h"
#include "box3d/quat.h"
#include "box3d/shape.h"
#include "box3d/id.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Plain-old-data rigid body. No internal heap pointers: instances live
 * inline in a b3_World's body array, or on the stack/embedded in a
 * caller-owned struct when used standalone. There is no
 * b3_rigidbody_create/destroy pair because there is nothing to
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

    b3_Shape shape; /* kept in sync with position/orientation via b3_rigidbody_sync_shape */

    b3_real restitution;
    b3_real friction;

    void *user_data; /* reserved, unused in v1 */

    /* Sleeping (see b3_World::sleeping_enabled): b3_world_step accumulates
     * sleep_timer while this body's velocity stays below
     * b3_World::sleep_linear_threshold/sleep_angular_threshold, and sets
     * is_sleeping once it reaches sleep_time_threshold. b3_rigidbody_integrate
     * then no-ops for this body exactly like a static one (inv_mass == 0)
     * -- joint/contact resolution still runs on it normally, so any
     * impulse that leaves it with an above-threshold velocity naturally
     * clears is_sleeping again on the next sleep-timer update, no explicit
     * "wake" call needed for that case. b3_rigidbody_wake() (also called
     * internally by apply_force/apply_impulse/set_mass) is for the other
     * case: code driving the body directly (e.g. a Python caller setting
     * .position/.linear_velocity) between steps. */
    b3_real sleep_timer;
    int is_sleeping;

    /* Internal bookkeeping for b3_World: the stable slot id this body was
     * handed at b3_world_add_body time (b3_id_invalid() for a standalone
     * body not attached to any World). Do not set this directly -- see
     * box3d/slotmap.h. */
    b3_Id slot_id;
} b3_RigidBody;

/* Initializes `body` in place with the given shape, centered at
 * `position` with identity orientation (the shape's own center/
 * orientation are overwritten to match). `mass == 0` creates a static
 * (infinite-mass) body: inv_mass and inv_inertia_local are all zero and
 * it is never moved by integration or collision response. Computes a
 * closed-form inertia tensor from `shape`'s kind (box/sphere/capsule).
 * b3_rigidbody_init_box/init_sphere/init_capsule are thin convenience
 * wrappers around this for each shape kind. */
B3_API void b3_rigidbody_init_shape(b3_RigidBody *body, b3_Shape shape, b3_Vec3 position, b3_real mass);

B3_API void b3_rigidbody_init_box(b3_RigidBody *body, b3_Vec3 position, b3_Vec3 half_extents, b3_real mass);
B3_API void b3_rigidbody_init_sphere(b3_RigidBody *body, b3_Vec3 position, b3_real radius, b3_real mass);
B3_API void b3_rigidbody_init_capsule(
    b3_RigidBody *body, b3_Vec3 position, b3_real radius, b3_real half_height, b3_real mass
);
B3_API void b3_rigidbody_init_hull(
    b3_RigidBody *body, b3_Vec3 position, const b3_Vec3 *local_vertices, int vertex_count, b3_real mass
);
B3_API void b3_rigidbody_init_compound(
    b3_RigidBody *body, b3_Vec3 position, const b3_CompoundChild *children, int child_count, b3_real mass
);

/* Mesh/height-field bodies are always static (no mass parameter -- see
 * docs/limitations.md): they get zero inertia and inv_mass, same as
 * b3_rigidbody_init_box/etc. with mass=0. */
B3_API void b3_rigidbody_init_mesh(
    b3_RigidBody *body, b3_Vec3 position, const b3_Vec3 *local_triangle_vertices, int triangle_count
);
B3_API void b3_rigidbody_init_heightfield(
    b3_RigidBody *body, b3_Vec3 position, b3_real cell_size,
    const b3_real *row_major_heights, int rows, int cols
);

/* Recomputes inv_mass and the closed-form inertia tensor (and its
 * inverse) for the body's current shape kind/dimensions. mass == 0 marks
 * the body static. */
B3_API void b3_rigidbody_set_mass(b3_RigidBody *body, b3_real mass);

B3_API void b3_rigidbody_apply_force(b3_RigidBody *body, b3_Vec3 force, b3_Vec3 world_point);
B3_API void b3_rigidbody_apply_impulse(b3_RigidBody *body, b3_Vec3 impulse, b3_Vec3 world_point);

/* Applies the body's diagonal body-local inverse inertia tensor to a
 * world-frame vector, accounting for the body's current orientation:
 * result = R * I_local^-1 * R^T * v. Exposed for solvers (e.g.
 * b3_world_step's contact resolution) that need a body's effective
 * angular mass to compute an impulse before applying it via
 * b3_rigidbody_apply_impulse. Zero for a static body (inv_inertia_local
 * is all zero), so it's always safe to call. */
B3_API b3_Vec3 b3_rigidbody_world_inv_inertia(const b3_RigidBody *body, b3_Vec3 world_vec);

B3_API void b3_rigidbody_clear_accumulators(b3_RigidBody *body);

/* Clears is_sleeping/sleep_timer. A no-op for an already-awake body. Not
 * called by b3_rigidbody_apply_force/apply_impulse themselves (those are
 * also the primitives b3_world_step's own solver uses internally for
 * every joint/contact impulse, including tiny converged-equilibrium
 * residuals every step -- auto-waking there would mean a sleeping body
 * touching anything never actually sleeps). The Python bindings call this
 * from the *external*, user-facing entry points instead: RigidBody's
 * apply_force()/apply_impulse() methods and its position/orientation/
 * linear_velocity/angular_velocity/mass setters (see py_rigidbody.c) --
 * acting on a body directly is taken to mean the caller wants it active.
 * Safe to call on a standalone body not attached to any World (sleeping
 * only has an observable effect via b3_world_step). */
B3_API void b3_rigidbody_wake(b3_RigidBody *body);

/* Copies position/orientation into body->shape so its collision shape
 * reflects the current pose. Called automatically by b3_world_step. */
B3_API void b3_rigidbody_sync_shape(b3_RigidBody *body);

/* Semi-implicit Euler integration step: applies `gravity` and the pending
 * force/torque accumulators to velocities, then integrates position and
 * orientation from those velocities, syncs the collision shape, and
 * clears the accumulators. A no-op for static bodies (inv_mass == 0) or
 * sleeping ones (is_sleeping != 0 -- see b3_World::sleeping_enabled).
 * Used by b3_world_step; also usable directly for a standalone body not
 * attached to any b3_World (is_sleeping is always 0 for one, since
 * nothing ever sets it outside b3_world_step). */
B3_API void b3_rigidbody_integrate(b3_RigidBody *body, b3_Vec3 gravity, b3_real dt);

#ifdef __cplusplus
}
#endif

#endif /* BOX3D_RIGIDBODY_H */
