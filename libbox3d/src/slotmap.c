#include "box3d/slotmap.h"

#include <stdlib.h>

int b3_slotmap_init(b3_SlotMap *map, int initial_capacity) {
    if (initial_capacity <= 0) {
        initial_capacity = 4;
    }

    map->id_to_index = (int *)malloc((size_t)initial_capacity * sizeof(int));
    map->generation = (unsigned int *)malloc((size_t)initial_capacity * sizeof(unsigned int));
    map->free_list = (int *)malloc((size_t)initial_capacity * sizeof(int));
    if (map->id_to_index == NULL || map->generation == NULL || map->free_list == NULL) {
        free(map->id_to_index);
        free(map->generation);
        free(map->free_list);
        map->id_to_index = NULL;
        map->generation = NULL;
        map->free_list = NULL;
        return -1;
    }

    map->free_count = 0;
    map->slot_count = 0;
    map->slot_capacity = initial_capacity;
    return 0;
}

void b3_slotmap_destroy(b3_SlotMap *map) {
    free(map->id_to_index);
    free(map->generation);
    free(map->free_list);
    map->id_to_index = NULL;
    map->generation = NULL;
    map->free_list = NULL;
    map->free_count = 0;
    map->slot_count = 0;
    map->slot_capacity = 0;
}

static int slotmap_grow(b3_SlotMap *map) {
    int new_capacity = map->slot_capacity > 0 ? map->slot_capacity * 2 : 4;
    int *new_id_to_index = (int *)realloc(map->id_to_index, (size_t)new_capacity * sizeof(int));
    if (new_id_to_index == NULL) {
        return -1;
    }
    map->id_to_index = new_id_to_index;

    unsigned int *new_generation =
        (unsigned int *)realloc(map->generation, (size_t)new_capacity * sizeof(unsigned int));
    if (new_generation == NULL) {
        return -1;
    }
    map->generation = new_generation;

    int *new_free_list = (int *)realloc(map->free_list, (size_t)new_capacity * sizeof(int));
    if (new_free_list == NULL) {
        return -1;
    }
    map->free_list = new_free_list;

    map->slot_capacity = new_capacity;
    return 0;
}

int b3_slotmap_alloc(b3_SlotMap *map, int dense_index, b3_Id *out_id) {
    int slot;
    if (map->free_count > 0) {
        slot = map->free_list[--map->free_count];
    } else {
        if (map->slot_count == map->slot_capacity) {
            if (slotmap_grow(map) < 0) {
                return -1;
            }
        }
        slot = map->slot_count++;
        map->generation[slot] = 0;
    }

    map->id_to_index[slot] = dense_index;
    out_id->index = slot;
    out_id->generation = map->generation[slot];
    return 0;
}

int b3_slotmap_resolve(const b3_SlotMap *map, b3_Id id) {
    if (id.index < 0 || id.index >= map->slot_count) {
        return -1;
    }
    if (map->generation[id.index] != id.generation) {
        return -1;
    }
    return map->id_to_index[id.index];
}

void b3_slotmap_update_index(b3_SlotMap *map, int slot, int new_dense_index) {
    map->id_to_index[slot] = new_dense_index;
}

void b3_slotmap_release(b3_SlotMap *map, int slot) {
    map->generation[slot] += 1;
    map->free_list[map->free_count++] = slot;
}
