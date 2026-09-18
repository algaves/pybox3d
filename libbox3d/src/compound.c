#include "box3d/compound.h"

#include <math.h>
#include <string.h>

b3_CompoundChild b3_compound_child_box(b3_Vec3 local_position, b3_Quat local_orientation, b3_Box3D box) {
    b3_CompoundChild child;
    child.local_position = local_position;
    child.local_orientation = b3_quat_normalize(local_orientation);
    child.kind = B3_COMPOUND_CHILD_BOX;
    child.as.box = box;
    return child;
}

b3_CompoundChild b3_compound_child_sphere(b3_Vec3 local_position, b3_Sphere sphere) {
    b3_CompoundChild child;
    child.local_position = local_position;
    child.local_orientation = b3_quat_identity();
    child.kind = B3_COMPOUND_CHILD_SPHERE;
    child.as.sphere = sphere;
    return child;
}

b3_CompoundChild b3_compound_child_capsule(
    b3_Vec3 local_position, b3_Quat local_orientation, b3_Capsule capsule
) {
    b3_CompoundChild child;
    child.local_position = local_position;
    child.local_orientation = b3_quat_normalize(local_orientation);
    child.kind = B3_COMPOUND_CHILD_CAPSULE;
    child.as.capsule = capsule;
    return child;
}

b3_CompoundChild b3_compound_child_hull(
    b3_Vec3 local_position, b3_Quat local_orientation, b3_ConvexHull hull
) {
    b3_CompoundChild child;
    child.local_position = local_position;
    child.local_orientation = b3_quat_normalize(local_orientation);
    child.kind = B3_COMPOUND_CHILD_HULL;
    child.as.hull = hull;
    return child;
}

b3_Compound b3_compound_make(
    b3_Vec3 center, b3_Quat orientation, const b3_CompoundChild *children, int child_count
) {
    b3_Compound compound;
    compound.center = center;
    compound.orientation = b3_quat_normalize(orientation);
    if (child_count > B3_COMPOUND_MAX_CHILDREN) {
        child_count = B3_COMPOUND_MAX_CHILDREN;
    }
    if (child_count < 0) {
        child_count = 0;
    }
    for (int i = 0; i < child_count; i++) {
        compound.children[i] = children[i];
    }
    compound.child_count = child_count;
    return compound;
}

void b3_compound_child_world_pose(
    const b3_Compound *compound, int index, b3_Vec3 *out_center, b3_Quat *out_orientation
) {
    const b3_CompoundChild *child = &compound->children[index];
    *out_center = b3_vec3_add(
        compound->center, b3_quat_rotate_vec3(compound->orientation, child->local_position)
    );
    *out_orientation = b3_quat_mul(compound->orientation, child->local_orientation);
}

/* Each of these re-poses a copy of the child's own shape struct to its
 * current world transform, then delegates to that shape kind's existing
 * (already-tested) implementation -- no per-kind collision logic is
 * duplicated here, only the dispatch. */

static b3_Vec3 child_support(
    const b3_CompoundChild *child, b3_Vec3 world_center, b3_Quat world_orientation, b3_Vec3 direction
) {
    switch (child->kind) {
        case B3_COMPOUND_CHILD_SPHERE: {
            b3_Sphere s = child->as.sphere;
            s.center = world_center;
            return b3_sphere_support(&s, direction);
        }
        case B3_COMPOUND_CHILD_CAPSULE: {
            b3_Capsule c = child->as.capsule;
            c.center = world_center;
            c.orientation = world_orientation;
            return b3_capsule_support(&c, direction);
        }
        case B3_COMPOUND_CHILD_HULL: {
            b3_ConvexHull h = child->as.hull;
            h.center = world_center;
            h.orientation = world_orientation;
            return b3_convexhull_support(&h, direction);
        }
        case B3_COMPOUND_CHILD_BOX:
        default: {
            b3_Box3D b = child->as.box;
            b.center = world_center;
            b.orientation = world_orientation;
            b.kind = B3_BOX_KIND_OBB; /* always exact regardless of the child's original kind */
            return b3_box3d_support(&b, direction);
        }
    }
}

static void child_aabb(
    const b3_CompoundChild *child, b3_Vec3 world_center, b3_Quat world_orientation,
    b3_Vec3 *out_min, b3_Vec3 *out_max
) {
    switch (child->kind) {
        case B3_COMPOUND_CHILD_SPHERE: {
            b3_Sphere s = child->as.sphere;
            s.center = world_center;
            b3_sphere_compute_aabb(&s, out_min, out_max);
            return;
        }
        case B3_COMPOUND_CHILD_CAPSULE: {
            b3_Capsule c = child->as.capsule;
            c.center = world_center;
            c.orientation = world_orientation;
            b3_capsule_compute_aabb(&c, out_min, out_max);
            return;
        }
        case B3_COMPOUND_CHILD_HULL: {
            b3_ConvexHull h = child->as.hull;
            h.center = world_center;
            h.orientation = world_orientation;
            b3_convexhull_compute_aabb(&h, out_min, out_max);
            return;
        }
        case B3_COMPOUND_CHILD_BOX:
        default: {
            b3_Box3D b = child->as.box;
            b.center = world_center;
            b.orientation = world_orientation;
            b.kind = B3_BOX_KIND_OBB;
            b3_box3d_compute_aabb(&b, out_min, out_max);
            return;
        }
    }
}

static int child_contains(
    const b3_CompoundChild *child, b3_Vec3 world_center, b3_Quat world_orientation, b3_Vec3 point
) {
    switch (child->kind) {
        case B3_COMPOUND_CHILD_SPHERE: {
            b3_Sphere s = child->as.sphere;
            s.center = world_center;
            return b3_sphere_contains_point(&s, point);
        }
        case B3_COMPOUND_CHILD_CAPSULE: {
            b3_Capsule c = child->as.capsule;
            c.center = world_center;
            c.orientation = world_orientation;
            return b3_capsule_contains_point(&c, point);
        }
        case B3_COMPOUND_CHILD_HULL: {
            b3_ConvexHull h = child->as.hull;
            h.center = world_center;
            h.orientation = world_orientation;
            return b3_convexhull_contains_point(&h, point);
        }
        case B3_COMPOUND_CHILD_BOX:
        default: {
            b3_Box3D b = child->as.box;
            b.center = world_center;
            b.orientation = world_orientation;
            b.kind = B3_BOX_KIND_OBB;
            return b3_box3d_contains_point(&b, point);
        }
    }
}

static b3_RayHit child_raycast(
    const b3_CompoundChild *child, b3_Vec3 world_center, b3_Quat world_orientation,
    b3_Vec3 origin, b3_Vec3 dir, b3_real max_t
) {
    switch (child->kind) {
        case B3_COMPOUND_CHILD_SPHERE: {
            b3_Sphere s = child->as.sphere;
            s.center = world_center;
            return b3_sphere_raycast(&s, origin, dir, max_t);
        }
        case B3_COMPOUND_CHILD_CAPSULE: {
            b3_Capsule c = child->as.capsule;
            c.center = world_center;
            c.orientation = world_orientation;
            return b3_capsule_raycast(&c, origin, dir, max_t);
        }
        case B3_COMPOUND_CHILD_HULL: {
            b3_ConvexHull h = child->as.hull;
            h.center = world_center;
            h.orientation = world_orientation;
            return b3_convexhull_raycast(&h, origin, dir, max_t);
        }
        case B3_COMPOUND_CHILD_BOX:
        default: {
            b3_Box3D b = child->as.box;
            b.center = world_center;
            b.orientation = world_orientation;
            b.kind = B3_BOX_KIND_OBB;
            return b3_box3d_raycast(&b, origin, dir, max_t);
        }
    }
}

void b3_compound_compute_aabb(const b3_Compound *compound, b3_Vec3 *out_min, b3_Vec3 *out_max) {
    if (compound->child_count == 0) {
        *out_min = compound->center;
        *out_max = compound->center;
        return;
    }
    b3_Vec3 lo = b3_vec3_zero(), hi = b3_vec3_zero();
    for (int i = 0; i < compound->child_count; i++) {
        b3_Vec3 wc;
        b3_Quat wo;
        b3_compound_child_world_pose(compound, i, &wc, &wo);
        b3_Vec3 clo, chi;
        child_aabb(&compound->children[i], wc, wo, &clo, &chi);
        if (i == 0) {
            lo = clo;
            hi = chi;
        } else {
            lo.x = fminf(lo.x, clo.x);
            lo.y = fminf(lo.y, clo.y);
            lo.z = fminf(lo.z, clo.z);
            hi.x = fmaxf(hi.x, chi.x);
            hi.y = fmaxf(hi.y, chi.y);
            hi.z = fmaxf(hi.z, chi.z);
        }
    }
    *out_min = lo;
    *out_max = hi;
}

int b3_compound_contains_point(const b3_Compound *compound, b3_Vec3 point) {
    for (int i = 0; i < compound->child_count; i++) {
        b3_Vec3 wc;
        b3_Quat wo;
        b3_compound_child_world_pose(compound, i, &wc, &wo);
        if (child_contains(&compound->children[i], wc, wo, point)) {
            return 1;
        }
    }
    return 0;
}

b3_Vec3 b3_compound_support(const b3_Compound *compound, b3_Vec3 direction) {
    if (compound->child_count == 0) {
        return compound->center;
    }
    b3_Vec3 wc;
    b3_Quat wo;
    b3_compound_child_world_pose(compound, 0, &wc, &wo);
    b3_Vec3 best = child_support(&compound->children[0], wc, wo, direction);
    b3_real best_dot = b3_vec3_dot(best, direction);
    for (int i = 1; i < compound->child_count; i++) {
        b3_compound_child_world_pose(compound, i, &wc, &wo);
        b3_Vec3 p = child_support(&compound->children[i], wc, wo, direction);
        b3_real d = b3_vec3_dot(p, direction);
        if (d > best_dot) {
            best_dot = d;
            best = p;
        }
    }
    return best;
}

b3_RayHit b3_compound_raycast(
    const b3_Compound *compound, b3_Vec3 origin, b3_Vec3 dir, b3_real max_t
) {
    b3_RayHit best;
    memset(&best, 0, sizeof(best));
    best.hit = 0;
    b3_real best_t = max_t;

    for (int i = 0; i < compound->child_count; i++) {
        b3_Vec3 wc;
        b3_Quat wo;
        b3_compound_child_world_pose(compound, i, &wc, &wo);
        b3_RayHit hit = child_raycast(&compound->children[i], wc, wo, origin, dir, best_t);
        if (hit.hit) {
            best = hit;
            best_t = hit.t;
        }
    }
    return best;
}
