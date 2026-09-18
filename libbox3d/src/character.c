#include "box3d/character.h"

void b3_character_init(b3_CharacterMover *mover, b3_Vec3 position, b3_Shape shape) {
    mover->position = position;
    mover->shape = shape;
    b3_shape_set_pose(&mover->shape, position, b3_quat_identity());
    mover->velocity = b3_vec3_zero();
    mover->is_grounded = 0;
    mover->skin_width = 0.01f;
    mover->max_slide_iterations = 4;
    mover->ground_normal_min_y = 0.5f;
}

void b3_character_move(b3_CharacterMover *mover, const b3_World *world, b3_Vec3 displacement) {
    mover->position = b3_vec3_add(mover->position, displacement);
    b3_shape_set_pose(&mover->shape, mover->position, b3_quat_identity());
    mover->is_grounded = 0;

    int iterations = mover->max_slide_iterations > 0 ? mover->max_slide_iterations : 1;
    for (int iter = 0; iter < iterations; iter++) {
        b3_real worst_penetration = 0.0f;
        b3_ContactInfo worst_contact = {0};
        int found = 0;
        for (int i = 0; i < world->body_count; i++) {
            b3_ContactInfo contact;
            if (!b3_shape_overlap(&mover->shape, &world->bodies[i].shape, &contact)) {
                continue;
            }
            if (!found || contact.penetration > worst_penetration) {
                worst_penetration = contact.penetration;
                worst_contact = contact;
                found = 1;
            }
        }
        if (!found) {
            break;
        }

        /* worst_contact.normal points from mover->shape ("a") toward the
         * world body ("b") -- push the character the other way, out of
         * the body. */
        b3_Vec3 push =
            b3_vec3_scale(worst_contact.normal, -(worst_contact.penetration + mover->skin_width));
        mover->position = b3_vec3_add(mover->position, push);
        b3_shape_set_pose(&mover->shape, mover->position, b3_quat_identity());

        b3_Vec3 out_normal = b3_vec3_negate(worst_contact.normal); /* obstacle -> character */
        b3_real vel_into_surface = b3_vec3_dot(mover->velocity, out_normal);
        if (vel_into_surface < 0.0f) {
            mover->velocity = b3_vec3_sub(mover->velocity, b3_vec3_scale(out_normal, vel_into_surface));
        }
        if (out_normal.y >= mover->ground_normal_min_y) {
            mover->is_grounded = 1;
        }
    }
}
