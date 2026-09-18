#ifndef BOX3D_SLOTMAP_H
#define BOX3D_SLOTMAP_H

#include "box3d_export.h"
#include "box3d/id.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Generation-checked indirection from a stable b3_Id to a dense array
 * position, shared by b3_World's body and joint storage (see world.c).
 * The dense array itself (and its swap-remove logic) lives in the owner;
 * a b3_SlotMap only tracks, per allocated "slot": which dense index it
 * currently maps to, and a generation counter bumped on release so a
 * stale id is reliably rejected instead of silently resolving to
 * whatever now occupies that dense index. Deliberately returns plain int
 * status codes (0/-1) rather than b3_Status, so this header has no
 * dependency on world.h. */
typedef struct {
    int *id_to_index;
    unsigned int *generation;
    int *free_list;
    int free_count;
    int slot_count;
    int slot_capacity;
} b3_SlotMap;

B3_API int b3_slotmap_init(b3_SlotMap *map, int initial_capacity);
B3_API void b3_slotmap_destroy(b3_SlotMap *map);

/* Allocates a new id-slot bound to `dense_index` (or reuses a released
 * one), writing the resulting id to *out_id. Returns 0 on success, -1 on
 * allocation failure. */
B3_API int b3_slotmap_alloc(b3_SlotMap *map, int dense_index, b3_Id *out_id);

/* Returns the current dense index for `id`, or -1 if it's out of range or
 * stale (its slot was since released). */
B3_API int b3_slotmap_resolve(const b3_SlotMap *map, b3_Id id);

/* Repoints id-slot `slot`'s dense index -- call this after a swap-remove
 * relocates the element owning `slot` to `new_dense_index`. */
B3_API void b3_slotmap_update_index(b3_SlotMap *map, int slot, int new_dense_index);

/* Invalidates every outstanding id for `slot` (bumps its generation) and
 * returns it to the free list for reuse. */
B3_API void b3_slotmap_release(b3_SlotMap *map, int slot);

#ifdef __cplusplus
}
#endif

#endif /* BOX3D_SLOTMAP_H */
