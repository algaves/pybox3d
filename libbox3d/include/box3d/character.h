#ifndef BOX3D_CHARACTER_H
#define BOX3D_CHARACTER_H

#include "box3d_export.h"
#include "box3d/vec3.h"
#include "box3d/shape.h"
#include "box3d/world.h"

#ifdef __cplusplus
extern "C" {
#endif

/* A kinematic move-and-slide character controller, built on the same
 * b3_shape_overlap discrete test everything else in this library uses --
 * there's no continuous/swept collision (see b3_character_move), so a
 * fast-moving character or a thin obstacle can still tunnel through in a
 * single large step, same caveat as b3_World's own discrete contacts.
 * Not a b3_RigidBody and not added to a b3_World: it's driven directly by
 * the caller each step, tested against a b3_World's bodies for
 * collision, but plays no part in that World's own physics (gravity,
 * joints, contact response between the character and anything else all
 * stay entirely up to the caller). */
typedef struct {
    b3_Vec3 position;
    b3_Shape shape; /* kept in sync with position via b3_character_move/b3_character_set_position */
    b3_Vec3 velocity;
    int is_grounded;

    b3_real skin_width;          /* depenetration margin beyond exact contact; default 0.01 */
    int max_slide_iterations;    /* discrete push-out passes per move() call; default 4 */
    b3_real ground_normal_min_y; /* dot(resolved contact normal, +Y) threshold for is_grounded; default 0.5 */
} b3_CharacterMover;

/* Initializes `mover` at `position` with `shape` (copied in; its own
 * center/orientation are overwritten to match `position`, no rotation --
 * a character controller's shape doesn't tip over). Sets the default
 * skin_width/max_slide_iterations/ground_normal_min_y named above. */
B3_API void b3_character_init(b3_CharacterMover *mover, b3_Vec3 position, b3_Shape shape);

/* Moves `mover` by `displacement` (the caller's own responsibility to
 * fold in gravity/input for this step -- there's no separate integration
 * here, unlike b3_rigidbody_integrate), then iteratively resolves any
 * resulting overlaps against `world`'s bodies: each of up to
 * `max_slide_iterations` passes finds the single worst-penetrating
 * overlap, pushes the character out along its normal by `penetration +
 * skin_width`, and removes that normal's into-the-surface component from
 * `velocity` (so a caller deriving next step's displacement from
 * `velocity` naturally slides along the obstacle rather than pushing
 * back into it). Sets `is_grounded` from whether any pass this call
 * resolved a contact whose normal pointed "up" enough
 * (>= ground_normal_min_y) -- not sticky between calls, so call this
 * every step (even with a zero displacement) if you need is_grounded to
 * stay current. */
B3_API void b3_character_move(b3_CharacterMover *mover, const b3_World *world, b3_Vec3 displacement);

#ifdef __cplusplus
}
#endif

#endif /* BOX3D_CHARACTER_H */
