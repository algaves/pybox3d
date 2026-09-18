#ifndef BOX3D_WORLD_H
#define BOX3D_WORLD_H

#include "box3d_export.h"
#include "box3d/vec3.h"
#include "box3d/rigidbody.h"
#include "box3d/joint.h"
#include "box3d/slotmap.h"
#include "box3d/collision.h"

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

/* A body pair, named by stable id, used for both b3_world_query_aabb's
 * results and b3_world_contacts_began/ended's diffed event pairs. */
typedef struct {
    b3_BodyId a;
    b3_BodyId b;
} b3_ContactPairIds;

/* One raycast hit against a specific body, as returned by
 * b3_world_raycast_all. */
typedef struct {
    b3_BodyId body_id;
    b3_RayHit hit;
} b3_WorldRayHit;

/* One body's transform/velocity at the moment b3_world_snapshot was
 * called, as returned in bulk by that function and consumed in bulk by
 * b3_world_restore. */
typedef struct {
    b3_BodyId id;
    b3_Vec3 position;
    b3_Quat orientation;
    b3_Vec3 linear_velocity;
    b3_Vec3 angular_velocity;
} b3_BodySnapshot;

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
    b3_SlotMap body_slots; /* stable b3_BodyId -> current dense index */

    b3_Joint *joints; /* world-owned heap array, grows via realloc */
    int joint_count;
    int joint_capacity;
    b3_SlotMap joint_slots; /* stable b3_JointId -> current dense index */

    /* Number of velocity-resolution passes b3_world_step runs per step
     * over the same detected joint/contact set (position correction still
     * runs once). >= 1; values <= 0 are treated as 1. Defaults to 4.
     * Raising this improves joint-chain/stacking convergence within a
     * single step (each iteration lets a correction propagate one more
     * hop through a chain) at a roughly linear cost in step time. */
    int solver_iterations;

    /* Sleeping (see b3_RigidBody::is_sleeping): when sleeping_enabled is
     * nonzero (the default), b3_world_step puts a dynamic body to sleep
     * once its linear/angular speed has stayed below
     * sleep_linear_threshold/sleep_angular_threshold for
     * sleep_time_threshold seconds -- skipping its integration (and, once
     * every body in a contact pair is asleep or static, that pair's
     * narrow phase too) until something (a joint/contact impulse, or an
     * explicit b3_rigidbody_wake()) gives it an above-threshold velocity
     * again. Defaults roughly match Box2D's: 0.05 m/s, 0.05 rad/s, 0.5s. */
    int sleeping_enabled;
    b3_real sleep_linear_threshold;
    b3_real sleep_angular_threshold;
    b3_real sleep_time_threshold;

    /* This step's contact-pair set, diffed against the previous step's to
     * populate contacts_began/contacts_ended (see b3_world_contacts_began/
     * b3_world_contacts_ended) -- world-owned scratch storage, grown via
     * realloc as needed. Not meant to be read directly; use the accessor
     * functions below, which are stable regardless of how this is laid
     * out. */
    b3_ContactPairIds *contact_pairs_prev;
    int contact_pairs_prev_count;
    int contact_pairs_prev_capacity;
    b3_ContactPairIds *contacts_began;
    int contacts_began_count;
    int contacts_began_capacity;
    b3_ContactPairIds *contacts_ended;
    int contacts_ended_count;
    int contacts_ended_capacity;
} b3_World;

B3_API b3_Status b3_world_init(b3_World *world, b3_Vec3 gravity, int initial_capacity);
B3_API void b3_world_destroy(b3_World *world);

/* Deep-copies `body` into the world's array, assigns it a stable
 * b3_BodyId (see b3_world_body_id), and writes its dense-array index to
 * `out_index` (may be NULL). The array may be reallocated (and thus
 * moved) as a result -- do not hold raw b3_RigidBody* pointers across
 * calls to b3_world_add_body/b3_world_remove_body; use b3_world_get_body
 * with the returned index (or b3_world_get_body_by_id with its id)
 * instead. */
B3_API b3_Status b3_world_add_body(b3_World *world, const b3_RigidBody *body, int *out_index);

/* Returns NULL if `index` is out of range. The returned pointer is only
 * valid until the next add/remove call on this world. */
B3_API b3_RigidBody *b3_world_get_body(b3_World *world, int index);

/* Id-based lookup: returns NULL if `id` is stale (its body was removed)
 * or was never issued by this world. Unlike b3_world_get_body(index),
 * this keeps resolving to the *same* body even if unrelated
 * add/remove calls have since moved it to a different dense index. */
B3_API b3_RigidBody *b3_world_get_body_by_id(b3_World *world, b3_BodyId id);

/* The stable id currently naming the body at dense position `index` (use
 * right after b3_world_add_body's out_index, or after b3_world_get_body,
 * to obtain a handle that survives future swap-removes). Returns
 * b3_id_invalid() if index is out of range. */
B3_API b3_BodyId b3_world_body_id(const b3_World *world, int index);

/* Swap-remove: the body at `index` is replaced by the last body in the
 * array (if it wasn't already last), which shrinks body_count by one.
 * Any *raw index* for the previously-last body now refers to a different
 * one -- but a b3_BodyId obtained via b3_world_body_id for that body
 * keeps resolving correctly (see b3_world_get_body_by_id). Also removes
 * every joint that referenced the removed body, so no joint is left
 * dangling/misdirected. */
B3_API b3_Status b3_world_remove_body(b3_World *world, int index);

/* Adds a distance joint (b3_JointKind B3_JOINT_DISTANCE) between the
 * bodies named by `body_a_id`/`body_b_id` (e.g. from b3_world_body_id),
 * writing its dense-array index to `out_index` (may be NULL). Returns
 * B3_ERR_INDEX_OUT_OF_RANGE if either id doesn't currently resolve, or
 * B3_ERR_INVALID_ARGUMENT if the two ids are equal. Same realloc/index-
 * stability caveats as b3_world_add_body apply to the returned index.
 * anchor_a/anchor_b default to each body's center (b3_vec3_zero());
 * min_length/max_length/has_limits/stiffness/damping default to a rigid
 * joint with no limits -- set them directly on the returned b3_Joint
 * (via b3_world_get_joint(world, *out_index)) to change them. */
B3_API b3_Status b3_world_add_joint(
    b3_World *world, b3_BodyId body_a_id, b3_BodyId body_b_id, b3_real rest_length, int *out_index
);

/* The rest of the joint kinds (see b3_JointKind), added the same way:
 * body_a_id/body_b_id must both currently resolve and be distinct (same
 * error cases as b3_world_add_joint above). anchor_a/anchor_b and every
 * kind-specific param (b3_Joint::params) start zeroed/defaulted -- set
 * them directly on the returned b3_Joint. Revolute/Prismatic/Wheel's
 * axis fields default to zero, which is *not* a usable axis (it fails to
 * normalize) -- callers must set params.<kind>.axis_a (and, for Wheel,
 * also axle_axis_a) before the first b3_world_step. */
B3_API b3_Status b3_world_add_spherical_joint(
    b3_World *world, b3_BodyId body_a_id, b3_BodyId body_b_id, int *out_index
);
B3_API b3_Status b3_world_add_revolute_joint(
    b3_World *world, b3_BodyId body_a_id, b3_BodyId body_b_id, int *out_index
);
B3_API b3_Status b3_world_add_prismatic_joint(
    b3_World *world, b3_BodyId body_a_id, b3_BodyId body_b_id, int *out_index
);
B3_API b3_Status b3_world_add_weld_joint(
    b3_World *world, b3_BodyId body_a_id, b3_BodyId body_b_id, int *out_index
);
B3_API b3_Status b3_world_add_motor_joint(
    b3_World *world, b3_BodyId body_a_id, b3_BodyId body_b_id, int *out_index
);
B3_API b3_Status b3_world_add_wheel_joint(
    b3_World *world, b3_BodyId body_a_id, b3_BodyId body_b_id, int *out_index
);
/* A Filter joint isn't a real constraint -- it's a marker consulted
 * during b3_world_step's narrow phase to skip collision entirely between
 * the two named bodies (in either order). It contributes no position/
 * velocity resolution. */
B3_API b3_Status b3_world_add_filter_joint(
    b3_World *world, b3_BodyId body_a_id, b3_BodyId body_b_id, int *out_index
);
/* Angular-only Weld: locks the two bodies' relative orientation (same
 * velocity-only lock as Weld -- see b3_Joint's doc comment) without any
 * translation constraint at all; anchor_a/anchor_b are unused. */
B3_API b3_Status b3_world_add_parallel_joint(
    b3_World *world, b3_BodyId body_a_id, b3_BodyId body_b_id, int *out_index
);

/* Returns NULL if `index` is out of range. The returned pointer is only
 * valid until the next add/remove joint call on this world. */
B3_API b3_Joint *b3_world_get_joint(b3_World *world, int index);

/* Id-based lookup, mirroring b3_world_get_body_by_id. */
B3_API b3_Joint *b3_world_get_joint_by_id(b3_World *world, b3_JointId id);

/* The stable id currently naming the joint at dense position `index`,
 * mirroring b3_world_body_id. */
B3_API b3_JointId b3_world_joint_id(const b3_World *world, int index);

/* Swap-remove, mirroring b3_world_remove_body (joints don't reference
 * other joints, so no further cascade is needed). */
B3_API b3_Status b3_world_remove_joint(b3_World *world, int index);

/* Writes up to `max_results` ids of bodies whose AABB overlaps
 * [min, max] into `out_ids`, and returns how many matched in total (which
 * may be more than `max_results` -- excess matches past `max_results`
 * are simply not written; pass a buffer sized to b3_World::body_count if
 * you want every match). `out_ids` may be NULL iff `max_results` is 0
 * (e.g. to just get a count). */
B3_API int b3_world_query_aabb(
    const b3_World *world, b3_Vec3 min, b3_Vec3 max, b3_BodyId *out_ids, int max_results
);

/* Raycasts against every body's shape (via b3_shape_raycast), writing up
 * to `max_results` hits (unsorted) into `out_hits`, and returning how
 * many matched in total (same overflow semantics as
 * b3_world_query_aabb). `dir` need not be normalized; `max_distance` is
 * in units of `dir`'s own length (i.e. a hit's `.hit.t` is the fraction
 * of `dir` traveled, matching b3_shape_raycast). */
B3_API int b3_world_raycast_all(
    const b3_World *world, b3_Vec3 origin, b3_Vec3 dir, b3_real max_distance, b3_WorldRayHit *out_hits,
    int max_results
);

/* Writes every current body's transform/velocity into `out_snapshots`
 * (sized to at least b3_World::body_count -- returns
 * B3_ERR_INVALID_ARGUMENT if `max_snapshots` is too small) and writes the
 * number of bodies snapshotted to `out_count`. Pair with
 * b3_world_restore() for simple state recording/replay: call this once
 * per step you want to be able to rewind to, store the results (e.g. in a
 * growable buffer of your own), and hand any of them back to
 * b3_world_restore() later. */
B3_API b3_Status b3_world_snapshot(
    const b3_World *world, b3_BodySnapshot *out_snapshots, int max_snapshots, int *out_count
);

/* Restores every body named in `snapshots` (by id) to its recorded
 * transform/velocity, zeroing force/torque accumulators and waking it
 * (see b3_rigidbody_wake) -- ids that don't currently resolve (e.g. the
 * body was removed since the snapshot was taken) are silently skipped,
 * and bodies added since the snapshot are left untouched (not removed).
 * Does not itself run a physics step; call b3_world_step afterwards as
 * usual to resume simulating from the restored state. */
B3_API void b3_world_restore(b3_World *world, const b3_BodySnapshot *snapshots, int count);

/* This step's newly-formed contact pairs (bodies that started overlapping
 * this step, having not overlapped last step) -- populated by
 * b3_world_step, valid until the next call to it. Index must be in
 * [0, b3_world_contacts_began_count(world)). */
B3_API int b3_world_contacts_began_count(const b3_World *world);
B3_API b3_ContactPairIds b3_world_contacts_began(const b3_World *world, int index);

/* This step's newly-separated contact pairs (bodies that stopped
 * overlapping this step, having overlapped last step) -- also covers a
 * pair where one body was removed from the world entirely, *and* a pair
 * where both bodies became static/asleep at the same time (they're
 * excluded from collision entirely once that's true, for the performance
 * b3_World::sleeping_enabled exists for -- so from the event tracking's
 * perspective they "end" even though they're still physically touching;
 * they'll "begin" again the moment either one wakes and they're
 * re-detected as overlapping). Same validity/indexing rules as
 * b3_world_contacts_began. */
B3_API int b3_world_contacts_ended_count(const b3_World *world);
B3_API b3_ContactPairIds b3_world_contacts_ended(const b3_World *world, int index);

/* One simulation step: integrate all dynamic bodies (semi-implicit
 * Euler; a no-op for a static or sleeping body -- see
 * b3_World::sleeping_enabled), solve every joint (see b3_JointKind), then
 * run a sort-and-sweep broad phase (bodies' AABBs sorted and swept along
 * X, a real broad-phase algorithm rather than this library's original
 * naive O(n^2) body-pair scan, though still O(n^2) in the pathological
 * case where every AABB overlaps) followed by narrow phase
 * (b3_shape_overlap) over each candidate pair -- skipping any pair
 * connected by a Filter joint, or where both bodies are static/asleep --
 * and resolving any contacts with a full 6-DOF impulse-based response
 * (using each pair's own restitution/friction, combined via standard
 * mixing rules -- max and geometric mean respectively).
 *
 * Joints and contacts are each detected/position-corrected once, then
 * their velocity constraints are resolved `solver_iterations` times
 * (sequential impulse) before bodies are considered settled for this
 * step -- see b3_World::solver_iterations. Each dynamic body's sleep
 * timer is then updated from its final velocity (see
 * b3_World::sleeping_enabled), and this step's contact-pair set is
 * diffed against last step's to populate b3_world_contacts_began/ended. */
B3_API void b3_world_step(b3_World *world, b3_real dt);

#ifdef __cplusplus
}
#endif

#endif /* BOX3D_WORLD_H */
