#include "box3d/world.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

b3_Status b3_world_init(b3_World *world, b3_Vec3 gravity, int initial_capacity) {
    if (initial_capacity < 0) {
        return B3_ERR_INVALID_ARGUMENT;
    }
    if (initial_capacity == 0) {
        initial_capacity = 4;
    }

    world->bodies = (b3_RigidBody *)malloc((size_t)initial_capacity * sizeof(b3_RigidBody));
    if (world->bodies == NULL) {
        return B3_ERR_OUT_OF_MEMORY;
    }
    world->joints = (b3_Joint *)malloc((size_t)initial_capacity * sizeof(b3_Joint));
    if (world->joints == NULL) {
        free(world->bodies);
        world->bodies = NULL;
        return B3_ERR_OUT_OF_MEMORY;
    }
    if (b3_slotmap_init(&world->body_slots, initial_capacity) < 0) {
        free(world->bodies);
        free(world->joints);
        world->bodies = NULL;
        world->joints = NULL;
        return B3_ERR_OUT_OF_MEMORY;
    }
    if (b3_slotmap_init(&world->joint_slots, initial_capacity) < 0) {
        free(world->bodies);
        free(world->joints);
        b3_slotmap_destroy(&world->body_slots);
        world->bodies = NULL;
        world->joints = NULL;
        return B3_ERR_OUT_OF_MEMORY;
    }

    world->gravity = gravity;
    world->body_count = 0;
    world->body_capacity = initial_capacity;
    world->joint_count = 0;
    world->joint_capacity = initial_capacity;
    world->solver_iterations = 4;

    world->sleeping_enabled = 1;
    world->sleep_linear_threshold = 0.05f;
    world->sleep_angular_threshold = 0.05f;
    world->sleep_time_threshold = 0.5f;

    world->contact_pairs_prev = NULL;
    world->contact_pairs_prev_count = 0;
    world->contact_pairs_prev_capacity = 0;
    world->contacts_began = NULL;
    world->contacts_began_count = 0;
    world->contacts_began_capacity = 0;
    world->contacts_ended = NULL;
    world->contacts_ended_count = 0;
    world->contacts_ended_capacity = 0;

    return B3_OK;
}

void b3_world_destroy(b3_World *world) {
    free(world->bodies);
    world->bodies = NULL;
    world->body_count = 0;
    world->body_capacity = 0;
    free(world->joints);
    world->joints = NULL;
    world->joint_count = 0;
    world->joint_capacity = 0;
    b3_slotmap_destroy(&world->body_slots);
    b3_slotmap_destroy(&world->joint_slots);
    free(world->contact_pairs_prev);
    world->contact_pairs_prev = NULL;
    world->contact_pairs_prev_count = 0;
    world->contact_pairs_prev_capacity = 0;
    free(world->contacts_began);
    world->contacts_began = NULL;
    world->contacts_began_count = 0;
    world->contacts_began_capacity = 0;
    free(world->contacts_ended);
    world->contacts_ended = NULL;
    world->contacts_ended_count = 0;
    world->contacts_ended_capacity = 0;
}

b3_Status b3_world_add_body(b3_World *world, const b3_RigidBody *body, int *out_index) {
    if (body == NULL) {
        return B3_ERR_INVALID_ARGUMENT;
    }

    if (world->body_count == world->body_capacity) {
        int new_capacity = world->body_capacity > 0 ? world->body_capacity * 2 : 4;
        b3_RigidBody *grown = (b3_RigidBody *)realloc(
            world->bodies, (size_t)new_capacity * sizeof(b3_RigidBody)
        );
        if (grown == NULL) {
            return B3_ERR_OUT_OF_MEMORY;
        }
        world->bodies = grown;
        world->body_capacity = new_capacity;
    }

    int index = world->body_count;
    world->bodies[index] = *body;

    b3_Id id;
    if (b3_slotmap_alloc(&world->body_slots, index, &id) < 0) {
        return B3_ERR_OUT_OF_MEMORY;
    }
    world->bodies[index].slot_id = id;

    if (out_index != NULL) {
        *out_index = index;
    }
    world->body_count += 1;
    return B3_OK;
}

b3_RigidBody *b3_world_get_body(b3_World *world, int index) {
    if (index < 0 || index >= world->body_count) {
        return NULL;
    }
    return &world->bodies[index];
}

b3_RigidBody *b3_world_get_body_by_id(b3_World *world, b3_BodyId id) {
    int index = b3_slotmap_resolve(&world->body_slots, id);
    if (index < 0) {
        return NULL;
    }
    return &world->bodies[index];
}

b3_BodyId b3_world_body_id(const b3_World *world, int index) {
    if (index < 0 || index >= world->body_count) {
        return b3_id_invalid();
    }
    return world->bodies[index].slot_id;
}

b3_Status b3_world_remove_body(b3_World *world, int index) {
    if (index < 0 || index >= world->body_count) {
        return B3_ERR_INDEX_OUT_OF_RANGE;
    }

    b3_Id removed_id = world->bodies[index].slot_id;

    int last = world->body_count - 1;
    if (index != last) {
        world->bodies[index] = world->bodies[last];
        b3_slotmap_update_index(&world->body_slots, world->bodies[index].slot_id.index, index);
    }
    world->body_count -= 1;
    b3_slotmap_release(&world->body_slots, removed_id.index);

    /* Drop every joint that referenced the now-removed body, rather than
     * leaving it dangling/misdirected. Iterating backwards is safe with
     * swap-remove: any element relocated into slot i by a removal came
     * from a higher position we've already visited (and kept) in this
     * same backward sweep. */
    for (int i = world->joint_count - 1; i >= 0; i--) {
        b3_Joint *joint = &world->joints[i];
        if (b3_id_equal(joint->body_a_id, removed_id) || b3_id_equal(joint->body_b_id, removed_id)) {
            b3_world_remove_joint(world, i);
        }
    }

    return B3_OK;
}

/* Shared allocation/validation for every joint kind: grows world->joints
 * if needed, validates the two body ids, and zero-initializes the new
 * b3_Joint's anchors/params (every kind-specific default beyond "zeroed"
 * is set by the caller right after this returns). */
static b3_Status world_add_joint_of_kind(
    b3_World *world, b3_JointKind kind, b3_BodyId body_a_id, b3_BodyId body_b_id, int *out_index
) {
    if (b3_slotmap_resolve(&world->body_slots, body_a_id) < 0 ||
        b3_slotmap_resolve(&world->body_slots, body_b_id) < 0) {
        return B3_ERR_INDEX_OUT_OF_RANGE;
    }
    if (b3_id_equal(body_a_id, body_b_id)) {
        return B3_ERR_INVALID_ARGUMENT;
    }

    if (world->joint_count == world->joint_capacity) {
        int new_capacity = world->joint_capacity > 0 ? world->joint_capacity * 2 : 4;
        b3_Joint *grown = (b3_Joint *)realloc(world->joints, (size_t)new_capacity * sizeof(b3_Joint));
        if (grown == NULL) {
            return B3_ERR_OUT_OF_MEMORY;
        }
        world->joints = grown;
        world->joint_capacity = new_capacity;
    }

    int index = world->joint_count;
    b3_Joint *joint = &world->joints[index];
    joint->body_a_id = body_a_id;
    joint->body_b_id = body_b_id;
    joint->kind = kind;
    joint->anchor_a = b3_vec3_zero();
    joint->anchor_b = b3_vec3_zero();
    memset(&joint->params, 0, sizeof(joint->params));

    b3_Id id;
    if (b3_slotmap_alloc(&world->joint_slots, index, &id) < 0) {
        return B3_ERR_OUT_OF_MEMORY;
    }
    joint->slot_id = id;

    if (out_index != NULL) {
        *out_index = index;
    }
    world->joint_count += 1;
    return B3_OK;
}

b3_Status b3_world_add_joint(
    b3_World *world, b3_BodyId body_a_id, b3_BodyId body_b_id, b3_real rest_length, int *out_index
) {
    int index;
    b3_Status status = world_add_joint_of_kind(world, B3_JOINT_DISTANCE, body_a_id, body_b_id, &index);
    if (status != B3_OK) {
        return status;
    }
    b3_Joint *joint = &world->joints[index];
    joint->params.distance.rest_length = rest_length;
    joint->params.distance.min_length = 0.0f;
    joint->params.distance.max_length = rest_length;
    joint->params.distance.has_limits = 0;
    joint->params.distance.stiffness = 0.0f;
    joint->params.distance.damping = 0.0f;
    if (out_index != NULL) {
        *out_index = index;
    }
    return B3_OK;
}

b3_Status b3_world_add_spherical_joint(
    b3_World *world, b3_BodyId body_a_id, b3_BodyId body_b_id, int *out_index
) {
    return world_add_joint_of_kind(world, B3_JOINT_SPHERICAL, body_a_id, body_b_id, out_index);
}

b3_Status b3_world_add_revolute_joint(
    b3_World *world, b3_BodyId body_a_id, b3_BodyId body_b_id, int *out_index
) {
    int index;
    b3_Status status = world_add_joint_of_kind(world, B3_JOINT_REVOLUTE, body_a_id, body_b_id, &index);
    if (status != B3_OK) {
        return status;
    }
    world->joints[index].params.revolute.axis_a = b3_vec3_make(0.0f, 1.0f, 0.0f);
    if (out_index != NULL) {
        *out_index = index;
    }
    return B3_OK;
}

b3_Status b3_world_add_prismatic_joint(
    b3_World *world, b3_BodyId body_a_id, b3_BodyId body_b_id, int *out_index
) {
    int index;
    b3_Status status = world_add_joint_of_kind(world, B3_JOINT_PRISMATIC, body_a_id, body_b_id, &index);
    if (status != B3_OK) {
        return status;
    }
    world->joints[index].params.prismatic.axis_a = b3_vec3_make(0.0f, 1.0f, 0.0f);
    if (out_index != NULL) {
        *out_index = index;
    }
    return B3_OK;
}

b3_Status b3_world_add_weld_joint(
    b3_World *world, b3_BodyId body_a_id, b3_BodyId body_b_id, int *out_index
) {
    return world_add_joint_of_kind(world, B3_JOINT_WELD, body_a_id, body_b_id, out_index);
}

b3_Status b3_world_add_motor_joint(
    b3_World *world, b3_BodyId body_a_id, b3_BodyId body_b_id, int *out_index
) {
    int index;
    b3_Status status = world_add_joint_of_kind(world, B3_JOINT_MOTOR, body_a_id, body_b_id, &index);
    if (status != B3_OK) {
        return status;
    }
    world->joints[index].params.motor.angular_offset = b3_quat_identity();
    if (out_index != NULL) {
        *out_index = index;
    }
    return B3_OK;
}

b3_Status b3_world_add_wheel_joint(
    b3_World *world, b3_BodyId body_a_id, b3_BodyId body_b_id, int *out_index
) {
    int index;
    b3_Status status = world_add_joint_of_kind(world, B3_JOINT_WHEEL, body_a_id, body_b_id, &index);
    if (status != B3_OK) {
        return status;
    }
    world->joints[index].params.wheel.suspension_axis_a = b3_vec3_make(0.0f, 1.0f, 0.0f);
    world->joints[index].params.wheel.axle_axis_a = b3_vec3_make(1.0f, 0.0f, 0.0f);
    if (out_index != NULL) {
        *out_index = index;
    }
    return B3_OK;
}

b3_Status b3_world_add_filter_joint(
    b3_World *world, b3_BodyId body_a_id, b3_BodyId body_b_id, int *out_index
) {
    return world_add_joint_of_kind(world, B3_JOINT_FILTER, body_a_id, body_b_id, out_index);
}

b3_Status b3_world_add_parallel_joint(
    b3_World *world, b3_BodyId body_a_id, b3_BodyId body_b_id, int *out_index
) {
    return world_add_joint_of_kind(world, B3_JOINT_PARALLEL, body_a_id, body_b_id, out_index);
}

b3_Joint *b3_world_get_joint(b3_World *world, int index) {
    if (index < 0 || index >= world->joint_count) {
        return NULL;
    }
    return &world->joints[index];
}

b3_Joint *b3_world_get_joint_by_id(b3_World *world, b3_JointId id) {
    int index = b3_slotmap_resolve(&world->joint_slots, id);
    if (index < 0) {
        return NULL;
    }
    return &world->joints[index];
}

b3_JointId b3_world_joint_id(const b3_World *world, int index) {
    if (index < 0 || index >= world->joint_count) {
        return b3_id_invalid();
    }
    return world->joints[index].slot_id;
}

b3_Status b3_world_remove_joint(b3_World *world, int index) {
    if (index < 0 || index >= world->joint_count) {
        return B3_ERR_INDEX_OUT_OF_RANGE;
    }
    b3_Id removed_id = world->joints[index].slot_id;
    int last = world->joint_count - 1;
    if (index != last) {
        world->joints[index] = world->joints[last];
        b3_slotmap_update_index(&world->joint_slots, world->joints[index].slot_id.index, index);
    }
    world->joint_count -= 1;
    b3_slotmap_release(&world->joint_slots, removed_id.index);
    return B3_OK;
}

static int aabb_overlaps(b3_Vec3 min_a, b3_Vec3 max_a, b3_Vec3 min_b, b3_Vec3 max_b) {
    return min_a.x <= max_b.x && min_b.x <= max_a.x && min_a.y <= max_b.y && min_b.y <= max_a.y &&
           min_a.z <= max_b.z && min_b.z <= max_a.z;
}

int b3_world_query_aabb(
    const b3_World *world, b3_Vec3 min, b3_Vec3 max, b3_BodyId *out_ids, int max_results
) {
    int matched = 0;
    for (int i = 0; i < world->body_count; i++) {
        b3_Vec3 body_min, body_max;
        b3_shape_compute_aabb(&world->bodies[i].shape, &body_min, &body_max);
        if (!aabb_overlaps(min, max, body_min, body_max)) {
            continue;
        }
        if (matched < max_results) {
            out_ids[matched] = world->bodies[i].slot_id;
        }
        matched++;
    }
    return matched;
}

int b3_world_raycast_all(
    const b3_World *world, b3_Vec3 origin, b3_Vec3 dir, b3_real max_distance, b3_WorldRayHit *out_hits,
    int max_results
) {
    int matched = 0;
    for (int i = 0; i < world->body_count; i++) {
        b3_RayHit hit = b3_shape_raycast(&world->bodies[i].shape, origin, dir, max_distance);
        if (!hit.hit) {
            continue;
        }
        if (matched < max_results) {
            out_hits[matched].body_id = world->bodies[i].slot_id;
            out_hits[matched].hit = hit;
        }
        matched++;
    }
    return matched;
}

b3_Status b3_world_snapshot(const b3_World *world, b3_BodySnapshot *out_snapshots, int max_snapshots, int *out_count) {
    if (max_snapshots < world->body_count) {
        return B3_ERR_INVALID_ARGUMENT;
    }
    for (int i = 0; i < world->body_count; i++) {
        const b3_RigidBody *body = &world->bodies[i];
        out_snapshots[i].id = body->slot_id;
        out_snapshots[i].position = body->position;
        out_snapshots[i].orientation = body->orientation;
        out_snapshots[i].linear_velocity = body->linear_velocity;
        out_snapshots[i].angular_velocity = body->angular_velocity;
    }
    if (out_count != NULL) {
        *out_count = world->body_count;
    }
    return B3_OK;
}

void b3_world_restore(b3_World *world, const b3_BodySnapshot *snapshots, int count) {
    for (int i = 0; i < count; i++) {
        b3_RigidBody *body = b3_world_get_body_by_id(world, snapshots[i].id);
        if (body == NULL) {
            continue;
        }
        body->position = snapshots[i].position;
        body->orientation = snapshots[i].orientation;
        body->linear_velocity = snapshots[i].linear_velocity;
        body->angular_velocity = snapshots[i].angular_velocity;
        b3_rigidbody_clear_accumulators(body);
        b3_rigidbody_wake(body);
        b3_rigidbody_sync_shape(body);
    }
}

int b3_world_contacts_began_count(const b3_World *world) {
    return world->contacts_began_count;
}
b3_ContactPairIds b3_world_contacts_began(const b3_World *world, int index) {
    return world->contacts_began[index];
}
int b3_world_contacts_ended_count(const b3_World *world) {
    return world->contacts_ended_count;
}
b3_ContactPairIds b3_world_contacts_ended(const b3_World *world, int index) {
    return world->contacts_ended[index];
}

static b3_Vec3 joint_anchor_world(const b3_RigidBody *body, b3_Vec3 local_anchor) {
    return b3_vec3_add(body->position, b3_quat_rotate_vec3(body->orientation, local_anchor));
}

/* This pair's angular effective-mass contribution to the impulse
 * denominator along `dir`, given each body's own lever arm to the point
 * the impulse is applied at. Forward-declared here, shared with
 * resolve_contact_velocity below (same math, same translation unit). */
static b3_real angular_effective_mass(
    const b3_RigidBody *a, b3_Vec3 r_a, const b3_RigidBody *b, b3_Vec3 r_b, b3_Vec3 dir
);

/* Positional correction only -- run `solver_iterations` times per step,
 * before the velocity passes (see b3_world_step; unlike contacts, this is
 * safe/beneficial to iterate since dist is recomputed fresh from current
 * positions each call, so repeated calls converge -- see the
 * b3_world_step comment for why contacts differ). Rigid-without-limits
 * mode targets rest_length exactly; a bare limits-only "rope" and spring
 * mode only get a hard correction when they've exceeded a has_limits
 * bound (a spring's own pull/push is otherwise a velocity-phase force,
 * not a position snap -- see resolve_distance_joint_velocity). Purely
 * translates each body's center along the anchor-to-anchor direction --
 * see the b3_DistanceJoint doc comment for why this (like contact
 * positional correction) stays linear-only. */
static void resolve_distance_joint_position(b3_RigidBody *a, b3_RigidBody *b, const b3_DistanceJoint *joint) {
    b3_real inv_mass_sum = a->inv_mass + b->inv_mass;
    if (inv_mass_sum <= 0.0f) {
        return; /* both static (or massless): nothing to resolve */
    }

    b3_Vec3 anchor_a = joint_anchor_world(a, joint->anchor_a);
    b3_Vec3 anchor_b = joint_anchor_world(b, joint->anchor_b);
    b3_Vec3 delta = b3_vec3_sub(anchor_b, anchor_a);
    b3_real dist = b3_vec3_length(delta);
    if (dist < 1e-6f) {
        return; /* degenerate: coincident anchors, no well-defined axis */
    }
    b3_Vec3 dir = b3_vec3_scale(delta, 1.0f / dist);

    b3_real error;
    if (joint->params.distance.has_limits && dist < joint->params.distance.min_length) {
        error = dist - joint->params.distance.min_length;
    } else if (joint->params.distance.has_limits && dist > joint->params.distance.max_length) {
        error = dist - joint->params.distance.max_length;
    } else if (!joint->params.distance.has_limits && joint->params.distance.stiffness <= 0.0f) {
        /* Rigid mode, no limits: a stiff rod at exactly rest_length. */
        error = dist - joint->params.distance.rest_length;
    } else {
        /* Free play: either a spring (whose pull is a velocity-phase
         * force, not a position snap -- see resolve_distance_joint_velocity)
         * or a bare limits-only "rope" with no rest_length pull at all,
         * currently within its slack range. */
        return;
    }

    /* Bilateral (pulls together or pushes apart depending on the sign of
     * the error), unlike resolve_contact's one-directional push-apart.
     * Gentler `percent` than contacts to avoid a stiff rod oscillating/
     * jittering. */
    const b3_real percent = 0.2f;
    b3_Vec3 correction = b3_vec3_scale(dir, error / inv_mass_sum * percent);
    a->position = b3_vec3_add(a->position, b3_vec3_scale(correction, a->inv_mass));
    b->position = b3_vec3_sub(b->position, b3_vec3_scale(correction, b->inv_mass));
    b3_rigidbody_sync_shape(a);
    b3_rigidbody_sync_shape(b);
}

/* Velocity-only resolution, meant to be called `solver_iterations` times
 * per step over the same joint (see b3_world_step) -- geometry (anchors,
 * dir, dist) is recomputed each call since positional correction may have
 * run (once) beforehand, but each call only ever consumes the constraint
 * error already present in the current velocities, so repeated calls
 * converge rather than compound (standard sequential-impulse behavior).
 * Applied at the world-space anchors via b3_rigidbody_apply_impulse, so
 * (unlike the v1 DistanceJoint before this) this does torque each body. */
static void resolve_distance_joint_velocity(
    b3_RigidBody *a, b3_RigidBody *b, const b3_DistanceJoint *joint, b3_real dt
) {
    b3_real inv_mass_sum = a->inv_mass + b->inv_mass;
    if (inv_mass_sum <= 0.0f) {
        return;
    }

    b3_Vec3 anchor_a = joint_anchor_world(a, joint->anchor_a);
    b3_Vec3 anchor_b = joint_anchor_world(b, joint->anchor_b);
    b3_Vec3 delta = b3_vec3_sub(anchor_b, anchor_a);
    b3_real dist = b3_vec3_length(delta);
    if (dist < 1e-6f) {
        return;
    }
    b3_Vec3 dir = b3_vec3_scale(delta, 1.0f / dist);

    b3_Vec3 r_a = b3_vec3_sub(anchor_a, a->position);
    b3_Vec3 r_b = b3_vec3_sub(anchor_b, b->position);

    b3_Vec3 vel_a = b3_vec3_add(a->linear_velocity, b3_vec3_cross(a->angular_velocity, r_a));
    b3_Vec3 vel_b = b3_vec3_add(b->linear_velocity, b3_vec3_cross(b->angular_velocity, r_b));
    b3_Vec3 rel_vel = b3_vec3_sub(vel_b, vel_a);
    b3_real vel_along_dir = b3_vec3_dot(rel_vel, dir);

    int hit_min = joint->params.distance.has_limits && dist < joint->params.distance.min_length;
    int hit_max = joint->params.distance.has_limits && dist > joint->params.distance.max_length;

    b3_real j;
    if (hit_min) {
        if (vel_along_dir > 0.0f) return; /* already separating back toward min_length */
        b3_real denom = inv_mass_sum + angular_effective_mass(a, r_a, b, r_b, dir);
        if (denom <= 0.0f) return;
        j = -vel_along_dir / denom;
    } else if (hit_max) {
        if (vel_along_dir < 0.0f) return; /* already relaxing back toward max_length */
        b3_real denom = inv_mass_sum + angular_effective_mass(a, r_a, b, r_b, dir);
        if (denom <= 0.0f) return;
        j = -vel_along_dir / denom;
    } else if (joint->params.distance.stiffness > 0.0f) {
        /* Soft spring: a literal Hooke's-law force integrated over `dt`,
         * not a velocity-cancelling constraint impulse -- no effective-
         * mass denominator. `dt` is the per-iteration sub-step (see
         * b3_world_step), so repeated calls approximate substep
         * integration of the spring-damper ODE rather than re-applying
         * the same full-step force `solver_iterations` times over. */
        b3_real spring_error = dist - joint->params.distance.rest_length;
        b3_real force = -joint->params.distance.stiffness * spring_error - joint->params.distance.damping * vel_along_dir;
        j = force * dt;
    } else if (!joint->params.distance.has_limits) {
        /* Rigid, no limits: cancel relative velocity along dir entirely
         * (a stiff rod, not a spring). */
        b3_real denom = inv_mass_sum + angular_effective_mass(a, r_a, b, r_b, dir);
        if (denom <= 0.0f) return;
        j = -vel_along_dir / denom;
    } else {
        /* Limits-only "rope", currently within its slack range: no
         * constraint force at all, free play until a bound is hit. */
        return;
    }

    b3_Vec3 impulse = b3_vec3_scale(dir, j);
    b3_rigidbody_apply_impulse(a, b3_vec3_negate(impulse), anchor_a);
    b3_rigidbody_apply_impulse(b, impulse, anchor_b);
}

/* Some vector perpendicular to `v` (arbitrary but deterministic),
 * normalized. Mirrors gjk.c's own static helper of the same purpose (not
 * shared across translation units to keep each file's collision/solver
 * code self-contained). */
static b3_Vec3 any_perpendicular(b3_Vec3 v) {
    b3_Vec3 p = b3_vec3_cross(v, b3_vec3_make(0.0f, 1.0f, 0.0f));
    if (b3_vec3_length_sq(p) < 1e-10f) {
        p = b3_vec3_cross(v, b3_vec3_make(1.0f, 0.0f, 0.0f));
    }
    return b3_vec3_normalize(p);
}

/* Two vectors forming an orthonormal basis together with (a normalized)
 * `axis`, used to constrain "everything perpendicular to this axis" --
 * e.g. a hinge's non-rotation-axis angular DOF, or a slider's non-slide-
 * axis translation DOF. */
static void perpendicular_basis(b3_Vec3 axis, b3_Vec3 *perp1, b3_Vec3 *perp2) {
    *perp1 = any_perpendicular(axis);
    *perp2 = b3_vec3_normalize(b3_vec3_cross(axis, *perp1));
}

/* Applies a pure torque impulse (no linear effect): body `a` gets
 * `-angular_impulse`, body `b` gets `+angular_impulse` (an action/
 * reaction pair), each scaled through its own inverse inertia tensor.
 * Shared by every rotation-locking/driving joint kind below. */
static void apply_torque_pair(b3_RigidBody *a, b3_RigidBody *b, b3_Vec3 angular_impulse) {
    a->angular_velocity =
        b3_vec3_sub(a->angular_velocity, b3_rigidbody_world_inv_inertia(a, angular_impulse));
    b->angular_velocity =
        b3_vec3_add(b->angular_velocity, b3_rigidbody_world_inv_inertia(b, angular_impulse));
}

/* Cancels the relative angular velocity between `a` and `b` along each of
 * `axes` (world-space, normalized) -- used to lock one or more rotational
 * DOF between two bodies. Velocity-only: see b3_Joint's doc comment for
 * why there's no positional (Baumgarte) counterpart for this. */
static void resolve_angular_lock_velocity(
    b3_RigidBody *a, b3_RigidBody *b, const b3_Vec3 *axes, int axis_count
) {
    for (int i = 0; i < axis_count; i++) {
        b3_Vec3 axis = axes[i];
        b3_real rel_along = b3_vec3_dot(b3_vec3_sub(b->angular_velocity, a->angular_velocity), axis);
        b3_Vec3 ang_a = b3_rigidbody_world_inv_inertia(a, axis);
        b3_Vec3 ang_b = b3_rigidbody_world_inv_inertia(b, axis);
        b3_real denom = b3_vec3_dot(ang_a, axis) + b3_vec3_dot(ang_b, axis);
        if (denom <= 0.0f) {
            continue;
        }
        b3_real impulse_mag = -rel_along / denom;
        apply_torque_pair(a, b, b3_vec3_scale(axis, impulse_mag));
    }
}

/* Positional correction for a point-style constraint (anchor_a should
 * coincide with anchor_b) restricted to `axes` (1-3 world-space,
 * normalized directions) -- used as-is (3 axes: X/Y/Z) for a full point
 * lock (Spherical/Revolute/Weld), or with 2 axes perpendicular to a slide
 * axis for a point-on-line constraint (Prismatic/Wheel). */
static void resolve_point_position_axes(
    b3_RigidBody *a, b3_RigidBody *b, b3_Vec3 world_anchor_a, b3_Vec3 world_anchor_b,
    const b3_Vec3 *axes, int axis_count
) {
    b3_real inv_mass_sum = a->inv_mass + b->inv_mass;
    if (inv_mass_sum <= 0.0f) {
        return;
    }
    b3_Vec3 delta = b3_vec3_sub(world_anchor_b, world_anchor_a);
    b3_Vec3 error_vec = b3_vec3_zero();
    for (int i = 0; i < axis_count; i++) {
        error_vec = b3_vec3_add(error_vec, b3_vec3_scale(axes[i], b3_vec3_dot(delta, axes[i])));
    }
    if (b3_vec3_length_sq(error_vec) < 1e-16f) {
        return;
    }
    const b3_real percent = 0.2f;
    b3_Vec3 correction = b3_vec3_scale(error_vec, percent / inv_mass_sum);
    a->position = b3_vec3_add(a->position, b3_vec3_scale(correction, a->inv_mass));
    b->position = b3_vec3_sub(b->position, b3_vec3_scale(correction, b->inv_mass));
    b3_rigidbody_sync_shape(a);
    b3_rigidbody_sync_shape(b);
}

/* Velocity-phase counterpart of resolve_point_position_axes: cancels
 * relative velocity *at the anchors* along each of `axes`, full 6-DOF
 * (torques both bodies via b3_rigidbody_apply_impulse). */
static void resolve_point_velocity_axes(
    b3_RigidBody *a, b3_RigidBody *b, b3_Vec3 world_anchor_a, b3_Vec3 world_anchor_b,
    const b3_Vec3 *axes, int axis_count
) {
    b3_real inv_mass_sum = a->inv_mass + b->inv_mass;
    if (inv_mass_sum <= 0.0f) {
        return;
    }
    b3_Vec3 r_a = b3_vec3_sub(world_anchor_a, a->position);
    b3_Vec3 r_b = b3_vec3_sub(world_anchor_b, b->position);
    for (int i = 0; i < axis_count; i++) {
        b3_Vec3 axis = axes[i];
        b3_Vec3 vel_a = b3_vec3_add(a->linear_velocity, b3_vec3_cross(a->angular_velocity, r_a));
        b3_Vec3 vel_b = b3_vec3_add(b->linear_velocity, b3_vec3_cross(b->angular_velocity, r_b));
        b3_real vel_along = b3_vec3_dot(b3_vec3_sub(vel_b, vel_a), axis);
        b3_real denom = inv_mass_sum + angular_effective_mass(a, r_a, b, r_b, axis);
        if (denom <= 0.0f) {
            continue;
        }
        b3_real j = -vel_along / denom;
        b3_Vec3 impulse = b3_vec3_scale(axis, j);
        b3_rigidbody_apply_impulse(a, b3_vec3_negate(impulse), world_anchor_a);
        b3_rigidbody_apply_impulse(b, impulse, world_anchor_b);
    }
}

static const b3_Vec3 *world_axes_xyz(void) {
    static const b3_Vec3 axes[3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    return axes;
}

/* Positional correction for Prismatic/Wheel's "along the slide/suspension
 * axis" translation limits (mirrors resolve_distance_joint_position's
 * hit_min/hit_max branch, generalized to a caller-supplied fixed
 * world-space axis and translation reading instead of a scalar
 * anchor-to-anchor distance). No-op if `has_limits` is false or the
 * translation is within bounds. */
static void resolve_axis_limit_position(
    b3_RigidBody *a, b3_RigidBody *b, b3_Vec3 axis, b3_real translation, b3_real min_translation,
    b3_real max_translation, int has_limits
) {
    if (!has_limits) {
        return;
    }
    b3_real inv_mass_sum = a->inv_mass + b->inv_mass;
    if (inv_mass_sum <= 0.0f) {
        return;
    }
    b3_real error;
    if (translation < min_translation) {
        error = translation - min_translation;
    } else if (translation > max_translation) {
        error = translation - max_translation;
    } else {
        return;
    }
    const b3_real percent = 0.2f;
    b3_Vec3 correction = b3_vec3_scale(axis, error / inv_mass_sum * percent);
    a->position = b3_vec3_add(a->position, b3_vec3_scale(correction, a->inv_mass));
    b->position = b3_vec3_sub(b->position, b3_vec3_scale(correction, b->inv_mass));
    b3_rigidbody_sync_shape(a);
    b3_rigidbody_sync_shape(b);
}

static void resolve_spherical_joint_position(b3_RigidBody *a, b3_RigidBody *b, const b3_Joint *joint) {
    b3_Vec3 world_anchor_a = joint_anchor_world(a, joint->anchor_a);
    b3_Vec3 world_anchor_b = joint_anchor_world(b, joint->anchor_b);
    resolve_point_position_axes(a, b, world_anchor_a, world_anchor_b, world_axes_xyz(), 3);
}

static void resolve_spherical_joint_velocity(b3_RigidBody *a, b3_RigidBody *b, const b3_Joint *joint) {
    b3_Vec3 world_anchor_a = joint_anchor_world(a, joint->anchor_a);
    b3_Vec3 world_anchor_b = joint_anchor_world(b, joint->anchor_b);
    resolve_point_velocity_axes(a, b, world_anchor_a, world_anchor_b, world_axes_xyz(), 3);
}

static void resolve_revolute_joint_position(b3_RigidBody *a, b3_RigidBody *b, const b3_Joint *joint) {
    resolve_spherical_joint_position(a, b, joint); /* same point lock, different angular treatment */
}

static void resolve_revolute_joint_velocity(
    b3_RigidBody *a, b3_RigidBody *b, const b3_Joint *joint, b3_real dt
) {
    resolve_spherical_joint_velocity(a, b, joint);

    const b3_RevoluteJointParams *params = &joint->params.revolute;
    b3_Vec3 hinge_axis = b3_vec3_normalize(b3_quat_rotate_vec3(a->orientation, params->axis_a));
    b3_Vec3 perp1, perp2;
    perpendicular_basis(hinge_axis, &perp1, &perp2);
    b3_Vec3 lock_axes[2] = {perp1, perp2};
    resolve_angular_lock_velocity(a, b, lock_axes, 2);

    if (!params->enable_motor) {
        return;
    }
    b3_real rel_along = b3_vec3_dot(b3_vec3_sub(b->angular_velocity, a->angular_velocity), hinge_axis);
    b3_Vec3 ang_a = b3_rigidbody_world_inv_inertia(a, hinge_axis);
    b3_Vec3 ang_b = b3_rigidbody_world_inv_inertia(b, hinge_axis);
    b3_real denom = b3_vec3_dot(ang_a, hinge_axis) + b3_vec3_dot(ang_b, hinge_axis);
    if (denom <= 0.0f) {
        return;
    }
    b3_real impulse_mag = (params->motor_speed - rel_along) / denom;
    b3_real max_impulse = params->max_motor_torque * dt;
    if (impulse_mag > max_impulse) impulse_mag = max_impulse;
    if (impulse_mag < -max_impulse) impulse_mag = -max_impulse;
    apply_torque_pair(a, b, b3_vec3_scale(hinge_axis, impulse_mag));
}

static void resolve_prismatic_joint_position(b3_RigidBody *a, b3_RigidBody *b, const b3_Joint *joint) {
    const b3_PrismaticJointParams *params = &joint->params.prismatic;
    b3_Vec3 slide_axis = b3_vec3_normalize(b3_quat_rotate_vec3(a->orientation, params->axis_a));
    b3_Vec3 perp1, perp2;
    perpendicular_basis(slide_axis, &perp1, &perp2);
    b3_Vec3 point_axes[2] = {perp1, perp2};
    b3_Vec3 world_anchor_a = joint_anchor_world(a, joint->anchor_a);
    b3_Vec3 world_anchor_b = joint_anchor_world(b, joint->anchor_b);
    resolve_point_position_axes(a, b, world_anchor_a, world_anchor_b, point_axes, 2);

    b3_real translation = b3_vec3_dot(b3_vec3_sub(world_anchor_b, world_anchor_a), slide_axis);
    resolve_axis_limit_position(
        a, b, slide_axis, translation, params->min_translation, params->max_translation,
        params->has_limits
    );
}

static void resolve_prismatic_joint_velocity(
    b3_RigidBody *a, b3_RigidBody *b, const b3_Joint *joint, b3_real dt
) {
    const b3_PrismaticJointParams *params = &joint->params.prismatic;
    b3_Vec3 slide_axis = b3_vec3_normalize(b3_quat_rotate_vec3(a->orientation, params->axis_a));
    b3_Vec3 perp1, perp2;
    perpendicular_basis(slide_axis, &perp1, &perp2);
    b3_Vec3 point_axes[2] = {perp1, perp2};
    b3_Vec3 world_anchor_a = joint_anchor_world(a, joint->anchor_a);
    b3_Vec3 world_anchor_b = joint_anchor_world(b, joint->anchor_b);
    resolve_point_velocity_axes(a, b, world_anchor_a, world_anchor_b, point_axes, 2);
    resolve_angular_lock_velocity(a, b, world_axes_xyz(), 3);

    b3_real inv_mass_sum = a->inv_mass + b->inv_mass;
    if (inv_mass_sum <= 0.0f) {
        return;
    }
    b3_Vec3 r_a = b3_vec3_sub(world_anchor_a, a->position);
    b3_Vec3 r_b = b3_vec3_sub(world_anchor_b, b->position);
    b3_real translation = b3_vec3_dot(b3_vec3_sub(world_anchor_b, world_anchor_a), slide_axis);
    b3_Vec3 vel_a = b3_vec3_add(a->linear_velocity, b3_vec3_cross(a->angular_velocity, r_a));
    b3_Vec3 vel_b = b3_vec3_add(b->linear_velocity, b3_vec3_cross(b->angular_velocity, r_b));
    b3_real vel_along = b3_vec3_dot(b3_vec3_sub(vel_b, vel_a), slide_axis);

    int hit_min = params->has_limits && translation < params->min_translation;
    int hit_max = params->has_limits && translation > params->max_translation;
    b3_real denom = inv_mass_sum + angular_effective_mass(a, r_a, b, r_b, slide_axis);
    if (denom <= 0.0f) {
        return;
    }
    if (hit_min && vel_along < 0.0f) {
        b3_Vec3 impulse = b3_vec3_scale(slide_axis, -vel_along / denom);
        b3_rigidbody_apply_impulse(a, b3_vec3_negate(impulse), world_anchor_a);
        b3_rigidbody_apply_impulse(b, impulse, world_anchor_b);
    } else if (hit_max && vel_along > 0.0f) {
        b3_Vec3 impulse = b3_vec3_scale(slide_axis, -vel_along / denom);
        b3_rigidbody_apply_impulse(a, b3_vec3_negate(impulse), world_anchor_a);
        b3_rigidbody_apply_impulse(b, impulse, world_anchor_b);
    } else if (params->enable_motor) {
        b3_real impulse_mag = (params->motor_speed - vel_along) / denom;
        b3_real max_impulse = params->max_motor_force * dt;
        if (impulse_mag > max_impulse) impulse_mag = max_impulse;
        if (impulse_mag < -max_impulse) impulse_mag = -max_impulse;
        b3_Vec3 impulse = b3_vec3_scale(slide_axis, impulse_mag);
        b3_rigidbody_apply_impulse(a, b3_vec3_negate(impulse), world_anchor_a);
        b3_rigidbody_apply_impulse(b, impulse, world_anchor_b);
    }
}

static void resolve_weld_joint_position(b3_RigidBody *a, b3_RigidBody *b, const b3_Joint *joint) {
    resolve_spherical_joint_position(a, b, joint);
}

static void resolve_weld_joint_velocity(b3_RigidBody *a, b3_RigidBody *b, const b3_Joint *joint) {
    resolve_spherical_joint_velocity(a, b, joint);
    resolve_angular_lock_velocity(a, b, world_axes_xyz(), 3);
}

static void resolve_motor_joint_velocity(
    b3_RigidBody *a, b3_RigidBody *b, const b3_Joint *joint, b3_real dt
) {
    const b3_MotorJointParams *params = &joint->params.motor;

    /* Linear spring toward a target position of body B relative to body
     * A's frame -- a literal force integrated over dt (like the distance
     * joint's spring mode), applied at each body's *center* rather than
     * an anchor: two independent (uncoupled) linear/angular springs are
     * far simpler to keep stable than one 6-DOF spring, and match how
     * resolve_contact_velocity's restitution impulse deliberately avoids
     * coupling linear correction through a torque arm -- see
     * docs/limitations.md. */
    if (params->linear_stiffness > 0.0f) {
        b3_real inv_mass_sum = a->inv_mass + b->inv_mass;
        if (inv_mass_sum > 0.0f) {
            b3_Vec3 target = b3_vec3_add(a->position, b3_quat_rotate_vec3(a->orientation, params->linear_offset));
            b3_Vec3 pos_error = b3_vec3_sub(b->position, target);
            b3_Vec3 rel_vel = b3_vec3_sub(b->linear_velocity, a->linear_velocity);
            b3_Vec3 force = b3_vec3_sub(
                b3_vec3_scale(pos_error, -params->linear_stiffness),
                b3_vec3_scale(rel_vel, params->linear_damping)
            );
            b3_Vec3 impulse = b3_vec3_scale(force, dt);
            a->linear_velocity = b3_vec3_sub(a->linear_velocity, b3_vec3_scale(impulse, a->inv_mass));
            b->linear_velocity = b3_vec3_add(b->linear_velocity, b3_vec3_scale(impulse, b->inv_mass));
        }
    }

    if (params->angular_stiffness > 0.0f) {
        b3_Quat current_rel = b3_quat_mul(b3_quat_conjugate(a->orientation), b->orientation);
        b3_Quat err_q = b3_quat_mul(b3_quat_conjugate(params->angular_offset), current_rel);
        /* Small-angle axis-angle approximation (2 * the quaternion's
         * vector part), taking the shortest path (w >= 0) -- accurate
         * enough for a spring that's meant to pull small deviations back
         * toward the target, not snap across a large one instantly. */
        b3_Vec3 err_vec = b3_vec3_make(err_q.x, err_q.y, err_q.z);
        if (err_q.w < 0.0f) {
            err_vec = b3_vec3_negate(err_vec);
        }
        err_vec = b3_vec3_scale(err_vec, 2.0f);
        b3_Vec3 rel_ang_vel = b3_vec3_sub(b->angular_velocity, a->angular_velocity);
        b3_Vec3 torque = b3_vec3_sub(
            b3_vec3_scale(err_vec, -params->angular_stiffness),
            b3_vec3_scale(rel_ang_vel, params->angular_damping)
        );
        apply_torque_pair(a, b, b3_vec3_scale(torque, dt));
    }
}

static void resolve_wheel_joint_position(b3_RigidBody *a, b3_RigidBody *b, const b3_Joint *joint) {
    const b3_WheelJointParams *params = &joint->params.wheel;
    b3_Vec3 susp_axis = b3_vec3_normalize(b3_quat_rotate_vec3(a->orientation, params->suspension_axis_a));
    b3_Vec3 perp1, perp2;
    perpendicular_basis(susp_axis, &perp1, &perp2);
    b3_Vec3 point_axes[2] = {perp1, perp2};
    b3_Vec3 world_anchor_a = joint_anchor_world(a, joint->anchor_a);
    b3_Vec3 world_anchor_b = joint_anchor_world(b, joint->anchor_b);
    resolve_point_position_axes(a, b, world_anchor_a, world_anchor_b, point_axes, 2);

    b3_real translation = b3_vec3_dot(b3_vec3_sub(world_anchor_b, world_anchor_a), susp_axis);
    resolve_axis_limit_position(
        a, b, susp_axis, translation, params->min_translation, params->max_translation,
        params->has_limits
    );
}

static void resolve_wheel_joint_velocity(b3_RigidBody *a, b3_RigidBody *b, const b3_Joint *joint, b3_real dt) {
    const b3_WheelJointParams *params = &joint->params.wheel;
    b3_Vec3 susp_axis = b3_vec3_normalize(b3_quat_rotate_vec3(a->orientation, params->suspension_axis_a));
    b3_Vec3 axle_axis = b3_vec3_normalize(b3_quat_rotate_vec3(a->orientation, params->axle_axis_a));

    b3_Vec3 perp1, perp2;
    perpendicular_basis(susp_axis, &perp1, &perp2);
    b3_Vec3 point_axes[2] = {perp1, perp2};
    b3_Vec3 world_anchor_a = joint_anchor_world(a, joint->anchor_a);
    b3_Vec3 world_anchor_b = joint_anchor_world(b, joint->anchor_b);
    resolve_point_velocity_axes(a, b, world_anchor_a, world_anchor_b, point_axes, 2);

    b3_Vec3 axle_perp1, axle_perp2;
    perpendicular_basis(axle_axis, &axle_perp1, &axle_perp2);
    b3_Vec3 lock_axes[2] = {axle_perp1, axle_perp2};
    resolve_angular_lock_velocity(a, b, lock_axes, 2);

    b3_real inv_mass_sum = a->inv_mass + b->inv_mass;
    if (inv_mass_sum <= 0.0f) {
        return;
    }
    b3_Vec3 r_a = b3_vec3_sub(world_anchor_a, a->position);
    b3_Vec3 r_b = b3_vec3_sub(world_anchor_b, b->position);
    b3_real translation = b3_vec3_dot(b3_vec3_sub(world_anchor_b, world_anchor_a), susp_axis);
    b3_Vec3 vel_a = b3_vec3_add(a->linear_velocity, b3_vec3_cross(a->angular_velocity, r_a));
    b3_Vec3 vel_b = b3_vec3_add(b->linear_velocity, b3_vec3_cross(b->angular_velocity, r_b));
    b3_real vel_along = b3_vec3_dot(b3_vec3_sub(vel_b, vel_a), susp_axis);

    int hit_min = params->has_limits && translation < params->min_translation;
    int hit_max = params->has_limits && translation > params->max_translation;
    if (hit_min && vel_along < 0.0f) {
        b3_real denom = inv_mass_sum + angular_effective_mass(a, r_a, b, r_b, susp_axis);
        if (denom <= 0.0f) return;
        b3_Vec3 impulse = b3_vec3_scale(susp_axis, -vel_along / denom);
        b3_rigidbody_apply_impulse(a, b3_vec3_negate(impulse), world_anchor_a);
        b3_rigidbody_apply_impulse(b, impulse, world_anchor_b);
    } else if (hit_max && vel_along > 0.0f) {
        b3_real denom = inv_mass_sum + angular_effective_mass(a, r_a, b, r_b, susp_axis);
        if (denom <= 0.0f) return;
        b3_Vec3 impulse = b3_vec3_scale(susp_axis, -vel_along / denom);
        b3_rigidbody_apply_impulse(a, b3_vec3_negate(impulse), world_anchor_a);
        b3_rigidbody_apply_impulse(b, impulse, world_anchor_b);
    } else if (params->suspension_stiffness > 0.0f) {
        /* Free-length spring toward zero offset along the suspension
         * axis (i.e. anchor_a/anchor_b coincide along that axis at
         * rest) -- a literal force, like the distance joint's spring
         * mode, not a velocity-cancelling impulse. */
        b3_real force = -params->suspension_stiffness * translation - params->suspension_damping * vel_along;
        b3_Vec3 impulse = b3_vec3_scale(susp_axis, force * dt);
        b3_rigidbody_apply_impulse(a, b3_vec3_negate(impulse), world_anchor_a);
        b3_rigidbody_apply_impulse(b, impulse, world_anchor_b);
    }
    /* else: no limits, no spring -- free slide along the suspension axis. */
}

static void resolve_parallel_joint_velocity(b3_RigidBody *a, b3_RigidBody *b) {
    resolve_angular_lock_velocity(a, b, world_axes_xyz(), 3);
}

/* Dispatches to the right position-phase resolver for `joint->kind`.
 * Distance is handled by its own dedicated function (see above); Motor,
 * Filter, and Parallel have no positional correction at all (Motor is a
 * pure velocity-phase spring, Filter isn't a constraint, and Parallel
 * constrains only rotation, which is velocity-only for every kind -- see
 * b3_Joint's doc comment). */
static void resolve_joint_position(b3_RigidBody *a, b3_RigidBody *b, const b3_Joint *joint) {
    switch (joint->kind) {
        case B3_JOINT_DISTANCE:
            resolve_distance_joint_position(a, b, joint);
            break;
        case B3_JOINT_SPHERICAL:
            resolve_spherical_joint_position(a, b, joint);
            break;
        case B3_JOINT_REVOLUTE:
            resolve_revolute_joint_position(a, b, joint);
            break;
        case B3_JOINT_PRISMATIC:
            resolve_prismatic_joint_position(a, b, joint);
            break;
        case B3_JOINT_WELD:
            resolve_weld_joint_position(a, b, joint);
            break;
        case B3_JOINT_WHEEL:
            resolve_wheel_joint_position(a, b, joint);
            break;
        case B3_JOINT_MOTOR:
        case B3_JOINT_FILTER:
        case B3_JOINT_PARALLEL:
            break;
    }
}

/* Dispatches to the right velocity-phase resolver for `joint->kind`. */
static void resolve_joint_velocity(b3_RigidBody *a, b3_RigidBody *b, const b3_Joint *joint, b3_real dt) {
    switch (joint->kind) {
        case B3_JOINT_DISTANCE:
            resolve_distance_joint_velocity(a, b, joint, dt);
            break;
        case B3_JOINT_SPHERICAL:
            resolve_spherical_joint_velocity(a, b, joint);
            break;
        case B3_JOINT_REVOLUTE:
            resolve_revolute_joint_velocity(a, b, joint, dt);
            break;
        case B3_JOINT_PRISMATIC:
            resolve_prismatic_joint_velocity(a, b, joint, dt);
            break;
        case B3_JOINT_WELD:
            resolve_weld_joint_velocity(a, b, joint);
            break;
        case B3_JOINT_MOTOR:
            resolve_motor_joint_velocity(a, b, joint, dt);
            break;
        case B3_JOINT_WHEEL:
            resolve_wheel_joint_velocity(a, b, joint, dt);
            break;
        case B3_JOINT_FILTER:
            break;
        case B3_JOINT_PARALLEL:
            resolve_parallel_joint_velocity(a, b);
            break;
    }
}

/* True if a Filter joint connects these two bodies (in either order) --
 * consulted by b3_world_step's narrow phase to skip collision entirely
 * for the pair. O(joint_count) per candidate pair, acceptable alongside
 * the narrow phase's own O(n^2) body-pair scan for the small body/joint
 * counts this "basic" library targets. */
static int is_filtered_pair(const b3_World *world, b3_BodyId a_id, b3_BodyId b_id) {
    for (int i = 0; i < world->joint_count; i++) {
        const b3_Joint *joint = &world->joints[i];
        if (joint->kind != B3_JOINT_FILTER) {
            continue;
        }
        if ((b3_id_equal(joint->body_a_id, a_id) && b3_id_equal(joint->body_b_id, b_id)) ||
            (b3_id_equal(joint->body_a_id, b_id) && b3_id_equal(joint->body_b_id, a_id))) {
            return 1;
        }
    }
    return 0;
}

/* Standard material mixing rules (matching common simple physics
 * engines): the bouncier of the two materials wins, but friction is
 * "diluted" by a slippery surface on either side. */
static b3_real combine_restitution(b3_real a, b3_real b) {
    return a > b ? a : b;
}
static b3_real combine_friction(b3_real a, b3_real b) {
    return sqrtf(a * b);
}

/* This pair's angular effective-mass contribution to the impulse
 * denominator along `dir` for a contact at `contact_point`: how much
 * each body's own rotation resists an impulse along `dir` applied there.
 * Zero for a static body (its inv_inertia_local is all zero). Shared by
 * both the normal and friction impulse solves below -- only `dir`
 * differs between the two calls. */
static b3_real angular_effective_mass(
    const b3_RigidBody *a, b3_Vec3 r_a, const b3_RigidBody *b, b3_Vec3 r_b, b3_Vec3 dir
) {
    b3_Vec3 ra_x_dir = b3_vec3_cross(r_a, dir);
    b3_Vec3 rb_x_dir = b3_vec3_cross(r_b, dir);
    b3_Vec3 ang_a = b3_rigidbody_world_inv_inertia(a, ra_x_dir);
    b3_Vec3 ang_b = b3_rigidbody_world_inv_inertia(b, rb_x_dir);
    return b3_vec3_dot(b3_vec3_cross(ang_a, r_a), dir) + b3_vec3_dot(b3_vec3_cross(ang_b, r_b), dir);
}

/* Positional correction only -- run once per step, before any velocity
 * iterations (see b3_world_step), mirroring
 * resolve_distance_joint_position. Push the bodies apart along the
 * contact normal, proportional to each body's share of the inverse mass,
 * to counteract integration drift/sinking. A small `slop` allowance and
 * `percent < 1` avoid jitter, matching common simple physics engines.
 * Purely linear -- a positional nudge needs no angular term. */
static void resolve_contact_position(b3_RigidBody *a, b3_RigidBody *b, const b3_ContactInfo *contact) {
    b3_real inv_mass_sum = a->inv_mass + b->inv_mass;
    if (inv_mass_sum <= 0.0f) {
        return; /* both static (or massless): nothing to resolve */
    }

    const b3_real percent = 0.8f;
    const b3_real slop = 0.005f;
    b3_real correction_mag = fmaxf(contact->penetration - slop, 0.0f) / inv_mass_sum * percent;
    b3_Vec3 correction = b3_vec3_scale(contact->normal, correction_mag);
    a->position = b3_vec3_sub(a->position, b3_vec3_scale(correction, a->inv_mass));
    b->position = b3_vec3_add(b->position, b3_vec3_scale(correction, b->inv_mass));
    b3_rigidbody_sync_shape(a);
    b3_rigidbody_sync_shape(b);
}

/* Velocity-only resolution, meant to be called `solver_iterations` times
 * per step over the same detected contact (see b3_world_step): each call
 * only consumes whatever constraint error remains in the current
 * velocities (the `vel_along_normal > 0` early-out below is what makes
 * repeated calls converge instead of compounding -- standard
 * sequential-impulse behavior, same as resolve_distance_joint_velocity). */
static void resolve_contact_velocity(b3_RigidBody *a, b3_RigidBody *b, const b3_ContactInfo *contact) {
    b3_real inv_mass_sum = a->inv_mass + b->inv_mass;
    if (inv_mass_sum <= 0.0f) {
        return; /* both static (or massless): nothing to resolve */
    }

    b3_real restitution = combine_restitution(a->restitution, b->restitution);
    b3_real friction = combine_friction(a->friction, b->friction);

    /* Contact torque lever arms, used below by both the friction impulse
     * (full 6-DOF: applied via b3_rigidbody_apply_impulse, so it also
     * torques each body) and the normal impulse's effective-mass
     * calculation. The normal impulse's own *application* stays
     * linear-only -- see the comment at its apply_impulse-shaped code
     * below for why. */
    b3_Vec3 r_a = b3_vec3_sub(contact->point, a->position);
    b3_Vec3 r_b = b3_vec3_sub(contact->point, b->position);

    b3_Vec3 rel_vel = b3_vec3_sub(b->linear_velocity, a->linear_velocity);
    b3_real vel_along_normal = b3_vec3_dot(rel_vel, contact->normal);
    if (vel_along_normal > 0.0f) {
        return; /* already separating */
    }

    /* Suppress restitution below a small closing-speed threshold (a
     * standard technique -- e.g. Box2D's b2_velocityThreshold) so a
     * settled/resting contact whose normal and point drift very
     * slightly from step to step (inherent to a round shape's or a
     * mesh/height-field's approximate GJK/EPA contact -- see
     * docs/limitations.md) doesn't repeatedly "bounce" a tiny amount. */
    const b3_real velocity_threshold = 1.0f;
    b3_real effective_restitution = -vel_along_normal < velocity_threshold ? 0.0f : restitution;

    /* v1 simplification: the normal/restitution impulse is applied
     * linearly (no torque), even though its magnitude below does account
     * for each body's angular effective mass. A fully coupled 6-DOF
     * normal-constraint solve (closing velocity measured *at the contact
     * point*, torquing both bodies) is the textbook-correct approach and
     * was tried, but this project's single-pass, non-iterative, non-
     * warm-started solver isn't stable with it for a body resting on a
     * displaced/offset contact point (e.g. an upright capsule on its
     * round cap): any rotation the body already has feeds back into the
     * next step's normal closing-velocity measurement, and with no
     * iteration to damp it, small errors compound into runaway spin over
     * hundreds of steps rather than settling. Friction -- the dominant
     * real source of contact-induced spin (e.g. a ball picking up roll
     * from horizontal sliding) -- keeps its full torque below; only the
     * separating/restitution impulse is restricted to linear. See
     * docs/limitations.md. */
    b3_real normal_denom = inv_mass_sum + angular_effective_mass(a, r_a, b, r_b, contact->normal);
    b3_real j = -(1.0f + effective_restitution) * vel_along_normal / normal_denom;
    b3_Vec3 impulse = b3_vec3_scale(contact->normal, j);
    a->linear_velocity = b3_vec3_sub(a->linear_velocity, b3_vec3_scale(impulse, a->inv_mass));
    b->linear_velocity = b3_vec3_add(b->linear_velocity, b3_vec3_scale(impulse, b->inv_mass));

    /* Basic Coulomb friction: a single-pass clamp of the tangential
     * relative velocity (now including each body's post-normal-impulse
     * rotation), no iterative solver / warm-starting. */
    b3_Vec3 vel_a = b3_vec3_add(a->linear_velocity, b3_vec3_cross(a->angular_velocity, r_a));
    b3_Vec3 vel_b = b3_vec3_add(b->linear_velocity, b3_vec3_cross(b->angular_velocity, r_b));
    rel_vel = b3_vec3_sub(vel_b, vel_a);
    b3_Vec3 tangent_vel = b3_vec3_sub(
        rel_vel, b3_vec3_scale(contact->normal, b3_vec3_dot(rel_vel, contact->normal))
    );
    b3_real tangent_speed = b3_vec3_length(tangent_vel);
    if (tangent_speed > 1e-6f) {
        b3_Vec3 tangent_dir = b3_vec3_scale(tangent_vel, 1.0f / tangent_speed);
        b3_real tangent_denom = inv_mass_sum + angular_effective_mass(a, r_a, b, r_b, tangent_dir);
        b3_real jt = -b3_vec3_dot(rel_vel, tangent_dir) / tangent_denom;
        b3_real max_friction = friction * fabsf(j);
        if (jt > max_friction) jt = max_friction;
        if (jt < -max_friction) jt = -max_friction;
        b3_Vec3 friction_impulse = b3_vec3_scale(tangent_dir, jt);
        b3_rigidbody_apply_impulse(a, b3_vec3_negate(friction_impulse), contact->point);
        b3_rigidbody_apply_impulse(b, friction_impulse, contact->point);
    }
}

typedef struct {
    b3_RigidBody *a;
    b3_RigidBody *b;
    b3_ContactInfo contact;
} b3_ContactPair;

/* A dynamic body that's neither static (inv_mass == 0) nor asleep --
 * i.e. one that actually needs its contacts resolved. A pair where
 * neither body is active can be skipped entirely: nothing would move
 * either way, and skipping avoids waking a sleeping body via spurious
 * positional-correction jitter (see b3_World::sleeping_enabled). */
static int body_is_active(const b3_RigidBody *body) {
    return body->inv_mass != 0.0f && !body->is_sleeping;
}

/* One entry per body for the sort-and-sweep broad phase below: `min_x`
 * (its AABB's minimum X, the sweep/sort key) alongside `body_index` (its
 * position in world->bodies, since qsort loses the array position). */
typedef struct {
    b3_real min_x;
    int body_index;
} b3_SweepEntry;

static int sweep_entry_cmp(const void *lhs, const void *rhs) {
    const b3_SweepEntry *la = (const b3_SweepEntry *)lhs;
    const b3_SweepEntry *rb = (const b3_SweepEntry *)rhs;
    if (la->min_x < rb->min_x) return -1;
    if (la->min_x > rb->min_x) return 1;
    if (la->body_index < rb->body_index) return -1;
    if (la->body_index > rb->body_index) return 1;
    return 0;
}

static int aabb_overlaps_yz(b3_Vec3 min_a, b3_Vec3 max_a, b3_Vec3 min_b, b3_Vec3 max_b) {
    return min_a.y <= max_b.y && min_b.y <= max_a.y && min_a.z <= max_b.z && min_b.z <= max_a.z;
}

/* Grows `*arr` (an array of b3_ContactPairIds, `*capacity`-long) via
 * realloc if `needed` exceeds its current capacity. Returns 0 on success
 * (including when no growth was needed), -1 on allocation failure (in
 * which case `*arr`/`*capacity` are left unchanged, per realloc's usual
 * failure contract). Shared by b3_World's three contact-pair scratch
 * arrays (contact_pairs_prev/contacts_began/contacts_ended). */
static int ensure_pair_capacity(b3_ContactPairIds **arr, int *capacity, int needed) {
    if (needed <= *capacity) {
        return 0;
    }
    int new_capacity = *capacity > 0 ? *capacity * 2 : 4;
    while (new_capacity < needed) {
        new_capacity *= 2;
    }
    b3_ContactPairIds *grown =
        (b3_ContactPairIds *)realloc(*arr, (size_t)new_capacity * sizeof(b3_ContactPairIds));
    if (grown == NULL) {
        return -1;
    }
    *arr = grown;
    *capacity = new_capacity;
    return 0;
}

/* Orders a pair's two ids by index so the same logical pair always
 * compares equal regardless of which body was named "a" vs "b" when it
 * was detected -- needed since b3_world_step's contact-began/ended
 * diffing (below) compares this step's pairs against last step's by
 * value. */
static void normalize_pair_ids(b3_BodyId *a, b3_BodyId *b) {
    if (b->index < a->index) {
        b3_BodyId tmp = *a;
        *a = *b;
        *b = tmp;
    }
}

static int pair_ids_equal(b3_ContactPairIds p, b3_ContactPairIds q) {
    return b3_id_equal(p.a, q.a) && b3_id_equal(p.b, q.b);
}

static int contains_pair_ids(const b3_ContactPairIds *arr, int count, b3_ContactPairIds pair) {
    for (int i = 0; i < count; i++) {
        if (pair_ids_equal(arr[i], pair)) {
            return 1;
        }
    }
    return 0;
}

void b3_world_step(b3_World *world, b3_real dt) {
    for (int i = 0; i < world->body_count; i++) {
        b3_rigidbody_integrate(&world->bodies[i], world->gravity, dt);
    }

    int iterations = world->solver_iterations > 0 ? world->solver_iterations : 1;
    b3_real sub_dt = dt / (b3_real)iterations;

    /* Joints: run `iterations` Gauss-Seidel passes of positional
     * correction (not just once) before the velocity passes below -- a
     * chain's positional error otherwise only propagates one joint per
     * *step*, since correcting joint i's position error only feeds into
     * joint i+1's error calculation on the *next* pass over the list.
     * Iterating this loop lets a correction ripple all the way down (or
     * up) a chain within a single step, which is what actually fixes the
     * documented joint-chain-sag limitation -- solving velocity alone
     * (previously the only thing this iterated) helps far less, since
     * sag is predominantly a positional-drift phenomenon here. */
    for (int iter = 0; iter < iterations; iter++) {
        for (int i = 0; i < world->joint_count; i++) {
            b3_Joint *joint = &world->joints[i];
            b3_RigidBody *a = b3_world_get_body_by_id(world, joint->body_a_id);
            b3_RigidBody *b = b3_world_get_body_by_id(world, joint->body_b_id);
            if (a == NULL || b == NULL) {
                continue; /* defensive: b3_world_remove_body already prevents this */
            }
            resolve_joint_position(a, b, joint);
        }
    }
    for (int iter = 0; iter < iterations; iter++) {
        for (int i = 0; i < world->joint_count; i++) {
            b3_Joint *joint = &world->joints[i];
            b3_RigidBody *a = b3_world_get_body_by_id(world, joint->body_a_id);
            b3_RigidBody *b = b3_world_get_body_by_id(world, joint->body_b_id);
            if (a == NULL || b == NULL) {
                continue;
            }
            resolve_joint_velocity(a, b, joint, sub_dt);
        }
    }

    /* Sort-and-sweep broad phase: sort bodies by AABB min.x, then sweep
     * to find candidate pairs whose AABBs actually overlap on all three
     * axes, rather than testing every one of the O(n^2) possible body
     * pairs against the (much more expensive) exact narrow-phase test.
     * Still O(n^2) in the pathological case where every AABB overlaps
     * (unavoidable -- that many pairs genuinely are candidates), but the
     * common case of a handful of nearby bodies out of many is now
     * O(n log n) instead. Contacts are detected once (bodies don't move
     * again until the loop below's position pass), buffered,
     * position-corrected once, then velocity-resolved `iterations` times
     * -- mirroring the joint solve above. */
    b3_Vec3 *aabb_min = world->body_count > 0 ? (b3_Vec3 *)malloc((size_t)world->body_count * sizeof(b3_Vec3)) : NULL;
    b3_Vec3 *aabb_max = world->body_count > 0 ? (b3_Vec3 *)malloc((size_t)world->body_count * sizeof(b3_Vec3)) : NULL;
    b3_SweepEntry *sweep =
        world->body_count > 0 ? (b3_SweepEntry *)malloc((size_t)world->body_count * sizeof(b3_SweepEntry)) : NULL;
    int max_pairs = world->body_count > 1 ? (world->body_count * (world->body_count - 1)) / 2 : 0;
    b3_ContactPair *pairs = max_pairs > 0 ? (b3_ContactPair *)malloc((size_t)max_pairs * sizeof(b3_ContactPair)) : NULL;
    int pair_count = 0;

    if (aabb_min != NULL && aabb_max != NULL && sweep != NULL) {
        for (int i = 0; i < world->body_count; i++) {
            b3_shape_compute_aabb(&world->bodies[i].shape, &aabb_min[i], &aabb_max[i]);
            sweep[i].min_x = aabb_min[i].x;
            sweep[i].body_index = i;
        }
        qsort(sweep, (size_t)world->body_count, sizeof(b3_SweepEntry), sweep_entry_cmp);

        for (int ii = 0; ii < world->body_count; ii++) {
            int i = sweep[ii].body_index;
            for (int jj = ii + 1; jj < world->body_count; jj++) {
                int j = sweep[jj].body_index;
                if (sweep[jj].min_x > aabb_max[i].x) {
                    break; /* sorted by min_x: nothing further can overlap `i` on X either */
                }
                if (!aabb_overlaps_yz(aabb_min[i], aabb_max[i], aabb_min[j], aabb_max[j])) {
                    continue;
                }

                b3_RigidBody *a = &world->bodies[i];
                b3_RigidBody *b = &world->bodies[j];
                if (!body_is_active(a) && !body_is_active(b)) {
                    continue;
                }
                if (is_filtered_pair(world, a->slot_id, b->slot_id)) {
                    continue;
                }
                b3_ContactInfo contact;
                if (!b3_shape_overlap(&a->shape, &b->shape, &contact)) {
                    continue;
                }
                if (pairs != NULL) {
                    pairs[pair_count].a = a;
                    pairs[pair_count].b = b;
                    pairs[pair_count].contact = contact;
                    pair_count++;
                }
                /* Position correction always runs once immediately, even
                 * if the pairs buffer couldn't be allocated (rare OOM) --
                 * in that degraded case velocity resolution below also
                 * just runs once inline instead of `iterations` times. */
                resolve_contact_position(a, b, &contact);
                if (pairs == NULL) {
                    resolve_contact_velocity(a, b, &contact);
                }
            }
        }
    }
    free(aabb_min);
    free(aabb_max);
    free(sweep);

    if (pairs != NULL) {
        for (int iter = 0; iter < iterations; iter++) {
            for (int p = 0; p < pair_count; p++) {
                resolve_contact_velocity(pairs[p].a, pairs[p].b, &pairs[p].contact);
            }
        }
    }

    /* Contact begin/end events: diff this step's contact-pair set
     * (normalized to a stable, order-independent id per pair) against
     * last step's. O(pair_count * prev_count), fine for the small body
     * counts this library targets (same tradeoff as is_filtered_pair
     * above). Skipped (events simply aren't tracked this step) if any
     * allocation along the way fails -- a rare-OOM degraded path, not a
     * correctness issue for the rest of the step. */
    if (pairs != NULL) {
        b3_ContactPairIds *current =
            pair_count > 0 ? (b3_ContactPairIds *)malloc((size_t)pair_count * sizeof(b3_ContactPairIds)) : NULL;
        if (pair_count == 0 || current != NULL) {
            for (int p = 0; p < pair_count; p++) {
                current[p].a = pairs[p].a->slot_id;
                current[p].b = pairs[p].b->slot_id;
                normalize_pair_ids(&current[p].a, &current[p].b);
            }

            world->contacts_began_count = 0;
            for (int p = 0; p < pair_count; p++) {
                if (contains_pair_ids(world->contact_pairs_prev, world->contact_pairs_prev_count, current[p])) {
                    continue;
                }
                if (ensure_pair_capacity(
                        &world->contacts_began, &world->contacts_began_capacity,
                        world->contacts_began_count + 1
                    ) == 0) {
                    world->contacts_began[world->contacts_began_count++] = current[p];
                }
            }

            world->contacts_ended_count = 0;
            for (int p = 0; p < world->contact_pairs_prev_count; p++) {
                if (contains_pair_ids(current, pair_count, world->contact_pairs_prev[p])) {
                    continue;
                }
                if (ensure_pair_capacity(
                        &world->contacts_ended, &world->contacts_ended_capacity,
                        world->contacts_ended_count + 1
                    ) == 0) {
                    world->contacts_ended[world->contacts_ended_count++] = world->contact_pairs_prev[p];
                }
            }

            if (ensure_pair_capacity(&world->contact_pairs_prev, &world->contact_pairs_prev_capacity, pair_count) ==
                    0 ||
                pair_count == 0) {
                for (int p = 0; p < pair_count; p++) {
                    world->contact_pairs_prev[p] = current[p];
                }
                world->contact_pairs_prev_count = pair_count;
            }
        }
        free(current);
        free(pairs);
    }

    /* Sleeping: update each dynamic body's sleep timer from its final
     * (post-resolution) velocity -- see b3_World::sleeping_enabled. A
     * body that just crossed back above the threshold (e.g. from a
     * contact/joint impulse) has is_sleeping cleared here too, which is
     * what lets a sleeping body woken by a collision actually resume
     * moving on the *next* step's integration (this step's integration
     * already ran, at the top, before that impulse existed). */
    if (world->sleeping_enabled) {
        for (int i = 0; i < world->body_count; i++) {
            b3_RigidBody *body = &world->bodies[i];
            if (body->inv_mass == 0.0f) {
                continue; /* static bodies have no sleep state to update */
            }
            b3_real lin_speed = b3_vec3_length(body->linear_velocity);
            b3_real ang_speed = b3_vec3_length(body->angular_velocity);
            if (lin_speed < world->sleep_linear_threshold && ang_speed < world->sleep_angular_threshold) {
                body->sleep_timer += dt;
                if (body->sleep_timer >= world->sleep_time_threshold) {
                    body->is_sleeping = 1;
                    body->linear_velocity = b3_vec3_zero();
                    body->angular_velocity = b3_vec3_zero();
                }
            } else {
                body->sleep_timer = 0.0f;
                body->is_sleeping = 0;
            }
        }
    }
}
