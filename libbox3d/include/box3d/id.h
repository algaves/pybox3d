#ifndef BOX3D_ID_H
#define BOX3D_ID_H

#include "box3d_export.h"

#ifdef __cplusplus
extern "C" {
#endif

/* A generation-checked slot reference: `index` names a slot in some
 * b3_SlotMap-backed collection (a b3_World's bodies or joints), and
 * `generation` must match that slot's current generation for the id to
 * still be considered live. Unlike a raw array index, an id keeps
 * resolving to the *same* logical entity even if unrelated swap-removes
 * relocate it to a different dense array position, and correctly reports
 * staleness once the entity it named has actually been removed -- see
 * box3d/slotmap.h. */
typedef struct {
    int index;
    unsigned int generation;
} b3_Id;

typedef b3_Id b3_BodyId;
typedef b3_Id b3_JointId;

/* Trivial enough to inline rather than add a translation unit for. */
static inline b3_Id b3_id_invalid(void) {
    b3_Id id;
    id.index = -1;
    id.generation = 0;
    return id;
}

static inline int b3_id_is_valid(b3_Id id) {
    return id.index >= 0;
}

static inline int b3_id_equal(b3_Id a, b3_Id b) {
    return a.index == b.index && a.generation == b.generation;
}

#ifdef __cplusplus
}
#endif

#endif /* BOX3D_ID_H */
