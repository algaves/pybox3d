#ifndef BOX3D_WORLD_H
#define BOX3D_WORLD_H

#include "box3d_export.h"
#include "box3d/vec3.h"
#include "box3d/rigidbody.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    B3_OK = 0,
    B3_ERR_OUT_OF_MEMORY,
    B3_ERR_INVALID_ARGUMENT,
    B3_ERR_CAPACITY_EXCEEDED,
    B3_ERR_INDEX_OUT_OF_RANGE
} b3_Status;

/* NOTE on capacity: `body_capacity` is a soft starting hint, not a hard
 * ceiling -- b3_world_add_body grows the array (realloc, doubling) as
 * needed and only ever returns B3_ERR_CAPACITY_EXCEEDED if that realloc
 * itself fails (folded into B3_ERR_OUT_OF_MEMORY in practice). It is kept
 * as a distinct status per the original design for callers that want to
 * enforce their own hard limits by checking body_count themselves. */
typedef struct {
    b3_Vec3 gravity;
    b3_RigidBody *bodies; /* world-owned heap array, grows via realloc */
    int body_count;
    int body_capacity;

    /* v1 simplification: collision resolution uses these two *global*
     * material values for every contact rather than per-body mixing
     * rules. Each b3_RigidBody still carries its own restitution/friction
     * fields (for forward compatibility / informational use), but
     * b3_world_step does not consult them -- see README for rationale. */
    b3_real default_restitution;
    b3_real default_friction;
} b3_World;

B3_API b3_Status b3_world_init(b3_World *world, b3_Vec3 gravity, int initial_capacity);
B3_API void b3_world_destroy(b3_World *world);

/* Deep-copies `body` into the world's array and writes its index to
 * `out_index` (may be NULL). The array may be reallocated (and thus
 * moved) as a result -- do not hold raw b3_RigidBody* pointers across
 * calls to b3_world_add_body/b3_world_remove_body; use b3_world_get_body
 * with the returned index instead. */
B3_API b3_Status b3_world_add_body(b3_World *world, const b3_RigidBody *body, int *out_index);

/* Returns NULL if `index` is out of range. The returned pointer is only
 * valid until the next add/remove call on this world. */
B3_API b3_RigidBody *b3_world_get_body(b3_World *world, int index);

/* Swap-remove: the body at `index` is replaced by the last body in the
 * array (if it wasn't already last), which shrinks body_count by one.
 * Any index for the previously-last body now refers to a different one. */
B3_API b3_Status b3_world_remove_body(b3_World *world, int index);

/* One simulation step: integrate all dynamic bodies (semi-implicit
 * Euler), then run a naive O(n^2) broad+narrow phase (b3_box3d_overlap)
 * over every body pair and resolve any contacts with an impulse-based
 * response plus positional correction. */
B3_API void b3_world_step(b3_World *world, b3_real dt);

#ifdef __cplusplus
}
#endif

#endif /* BOX3D_WORLD_H */
