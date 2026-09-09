#include "box3d/world.h"

#include <stdlib.h>
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

    world->gravity = gravity;
    world->body_count = 0;
    world->body_capacity = initial_capacity;
    world->default_restitution = 0.3f;
    world->default_friction = 0.5f;
    return B3_OK;
}

void b3_world_destroy(b3_World *world) {
    free(world->bodies);
    world->bodies = NULL;
    world->body_count = 0;
    world->body_capacity = 0;
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

    world->bodies[world->body_count] = *body;
    if (out_index != NULL) {
        *out_index = world->body_count;
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

b3_Status b3_world_remove_body(b3_World *world, int index) {
    if (index < 0 || index >= world->body_count) {
        return B3_ERR_INDEX_OUT_OF_RANGE;
    }
    int last = world->body_count - 1;
    if (index != last) {
        world->bodies[index] = world->bodies[last];
    }
    world->body_count -= 1;
    return B3_OK;
}

static void resolve_contact(
    b3_RigidBody *a, b3_RigidBody *b, const b3_ContactInfo *contact,
    b3_real restitution, b3_real friction
) {
    b3_real inv_mass_sum = a->inv_mass + b->inv_mass;
    if (inv_mass_sum <= 0.0f) {
        return; /* both static (or massless): nothing to resolve */
    }

    /* Positional correction: push the bodies apart along the contact
     * normal, proportional to each body's share of the inverse mass, to
     * counteract integration drift/sinking. A small `slop` allowance and
     * `percent < 1` avoid jitter, matching common simple physics engines. */
    const b3_real percent = 0.8f;
    const b3_real slop = 0.005f;
    b3_real correction_mag = fmaxf(contact->penetration - slop, 0.0f) / inv_mass_sum * percent;
    b3_Vec3 correction = b3_vec3_scale(contact->normal, correction_mag);
    a->position = b3_vec3_sub(a->position, b3_vec3_scale(correction, a->inv_mass));
    b->position = b3_vec3_add(b->position, b3_vec3_scale(correction, b->inv_mass));
    b3_rigidbody_sync_shape(a);
    b3_rigidbody_sync_shape(b);

    /* Impulse-based velocity resolution along the contact normal
     * (linear-only in v1 -- no angular contribution to the contact
     * impulse; see README limitations). */
    b3_Vec3 rel_vel = b3_vec3_sub(b->linear_velocity, a->linear_velocity);
    b3_real vel_along_normal = b3_vec3_dot(rel_vel, contact->normal);
    if (vel_along_normal > 0.0f) {
        return; /* already separating */
    }

    b3_real j = -(1.0f + restitution) * vel_along_normal / inv_mass_sum;
    b3_Vec3 impulse = b3_vec3_scale(contact->normal, j);
    a->linear_velocity = b3_vec3_sub(a->linear_velocity, b3_vec3_scale(impulse, a->inv_mass));
    b->linear_velocity = b3_vec3_add(b->linear_velocity, b3_vec3_scale(impulse, b->inv_mass));

    /* Basic Coulomb friction: a single-pass clamp of the tangential
     * relative velocity, no iterative solver / warm-starting. */
    rel_vel = b3_vec3_sub(b->linear_velocity, a->linear_velocity);
    b3_Vec3 tangent_vel = b3_vec3_sub(
        rel_vel, b3_vec3_scale(contact->normal, b3_vec3_dot(rel_vel, contact->normal))
    );
    b3_real tangent_speed = b3_vec3_length(tangent_vel);
    if (tangent_speed > 1e-6f) {
        b3_Vec3 tangent_dir = b3_vec3_scale(tangent_vel, 1.0f / tangent_speed);
        b3_real jt = -b3_vec3_dot(rel_vel, tangent_dir) / inv_mass_sum;
        b3_real max_friction = friction * fabsf(j);
        if (jt > max_friction) jt = max_friction;
        if (jt < -max_friction) jt = -max_friction;
        b3_Vec3 friction_impulse = b3_vec3_scale(tangent_dir, jt);
        a->linear_velocity = b3_vec3_sub(a->linear_velocity, b3_vec3_scale(friction_impulse, a->inv_mass));
        b->linear_velocity = b3_vec3_add(b->linear_velocity, b3_vec3_scale(friction_impulse, b->inv_mass));
    }
}

void b3_world_step(b3_World *world, b3_real dt) {
    for (int i = 0; i < world->body_count; i++) {
        b3_rigidbody_integrate(&world->bodies[i], world->gravity, dt);
    }

    /* Naive O(n^2) broad+narrow phase, adequate for the small body counts
     * this "basic" library targets. */
    for (int i = 0; i < world->body_count; i++) {
        for (int j = i + 1; j < world->body_count; j++) {
            b3_RigidBody *a = &world->bodies[i];
            b3_RigidBody *b = &world->bodies[j];
            if (a->inv_mass == 0.0f && b->inv_mass == 0.0f) {
                continue;
            }
            b3_ContactInfo contact;
            if (b3_box3d_overlap(&a->shape, &b->shape, &contact)) {
                resolve_contact(a, b, &contact, world->default_restitution, world->default_friction);
            }
        }
    }
}
