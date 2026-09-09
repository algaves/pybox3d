#include "box3d/rigidbody.h"

#include <stddef.h>

void b3_rigidbody_init_box(b3_RigidBody *body, b3_Vec3 position, b3_Vec3 half_extents, b3_real mass) {
    body->position = position;
    body->orientation = b3_quat_identity();
    body->linear_velocity = b3_vec3_zero();
    body->angular_velocity = b3_vec3_zero();

    body->force_accum = b3_vec3_zero();
    body->torque_accum = b3_vec3_zero();

    body->shape = b3_box3d_make_obb(position, half_extents, body->orientation);

    body->restitution = 0.2f;
    body->friction = 0.5f;
    body->user_data = NULL;

    b3_rigidbody_set_mass(body, mass);
}

void b3_rigidbody_set_mass(b3_RigidBody *body, b3_real mass) {
    body->mass = mass;
    if (mass <= 0.0f) {
        body->mass = 0.0f;
        body->inv_mass = 0.0f;
        body->inertia_local[0] = body->inertia_local[1] = body->inertia_local[2] = 0.0f;
        body->inv_inertia_local[0] = body->inv_inertia_local[1] = body->inv_inertia_local[2] = 0.0f;
        return;
    }

    body->inv_mass = 1.0f / mass;

    b3_real hx = body->shape.half_extents.x;
    b3_real hy = body->shape.half_extents.y;
    b3_real hz = body->shape.half_extents.z;
    b3_real dx = 2.0f * hx, dy = 2.0f * hy, dz = 2.0f * hz;

    body->inertia_local[0] = (mass / 12.0f) * (dy * dy + dz * dz);
    body->inertia_local[1] = (mass / 12.0f) * (dx * dx + dz * dz);
    body->inertia_local[2] = (mass / 12.0f) * (dx * dx + dy * dy);

    for (int i = 0; i < 3; i++) {
        body->inv_inertia_local[i] =
            body->inertia_local[i] > 1e-12f ? 1.0f / body->inertia_local[i] : 0.0f;
    }
}

/* Applies the body's diagonal body-local inverse inertia tensor to a
 * world-frame vector, accounting for the body's current orientation:
 * result = R * I_local^-1 * R^T * v. */
static b3_Vec3 rigidbody_world_inv_inertia(const b3_RigidBody *body, b3_Vec3 world_vec) {
    b3_Quat inv_orientation = b3_quat_conjugate(body->orientation);
    b3_Vec3 local = b3_quat_rotate_vec3(inv_orientation, world_vec);
    b3_Vec3 scaled = b3_vec3_make(
        local.x * body->inv_inertia_local[0],
        local.y * body->inv_inertia_local[1],
        local.z * body->inv_inertia_local[2]
    );
    return b3_quat_rotate_vec3(body->orientation, scaled);
}

void b3_rigidbody_apply_force(b3_RigidBody *body, b3_Vec3 force, b3_Vec3 world_point) {
    if (body->inv_mass == 0.0f) {
        return;
    }
    body->force_accum = b3_vec3_add(body->force_accum, force);
    b3_Vec3 r = b3_vec3_sub(world_point, body->position);
    body->torque_accum = b3_vec3_add(body->torque_accum, b3_vec3_cross(r, force));
}

void b3_rigidbody_apply_impulse(b3_RigidBody *body, b3_Vec3 impulse, b3_Vec3 world_point) {
    if (body->inv_mass == 0.0f) {
        return;
    }
    body->linear_velocity = b3_vec3_add(body->linear_velocity, b3_vec3_scale(impulse, body->inv_mass));

    b3_Vec3 r = b3_vec3_sub(world_point, body->position);
    b3_Vec3 angular_impulse = b3_vec3_cross(r, impulse);
    b3_Vec3 delta_angular = rigidbody_world_inv_inertia(body, angular_impulse);
    body->angular_velocity = b3_vec3_add(body->angular_velocity, delta_angular);
}

void b3_rigidbody_clear_accumulators(b3_RigidBody *body) {
    body->force_accum = b3_vec3_zero();
    body->torque_accum = b3_vec3_zero();
}

void b3_rigidbody_sync_shape(b3_RigidBody *body) {
    body->shape.center = body->position;
    body->shape.orientation = body->orientation;
}

void b3_rigidbody_integrate(b3_RigidBody *body, b3_Vec3 gravity, b3_real dt) {
    if (body->inv_mass == 0.0f) {
        b3_rigidbody_clear_accumulators(body);
        return;
    }

    b3_Vec3 linear_accel = b3_vec3_add(gravity, b3_vec3_scale(body->force_accum, body->inv_mass));
    body->linear_velocity = b3_vec3_add(body->linear_velocity, b3_vec3_scale(linear_accel, dt));

    b3_Vec3 angular_accel = rigidbody_world_inv_inertia(body, body->torque_accum);
    body->angular_velocity = b3_vec3_add(body->angular_velocity, b3_vec3_scale(angular_accel, dt));

    body->position = b3_vec3_add(body->position, b3_vec3_scale(body->linear_velocity, dt));
    body->orientation = b3_quat_integrate(body->orientation, body->angular_velocity, dt);

    b3_rigidbody_sync_shape(body);
    b3_rigidbody_clear_accumulators(body);
}
