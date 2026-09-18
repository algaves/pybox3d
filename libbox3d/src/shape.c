#include "box3d/shape.h"
#include "box3d/gjk.h"

b3_Shape b3_shape_from_box(b3_Box3D box) {
    b3_Shape shape;
    shape.kind = B3_SHAPE_BOX;
    shape.as.box = box;
    return shape;
}

b3_Shape b3_shape_from_sphere(b3_Sphere sphere) {
    b3_Shape shape;
    shape.kind = B3_SHAPE_SPHERE;
    shape.as.sphere = sphere;
    return shape;
}

b3_Shape b3_shape_from_capsule(b3_Capsule capsule) {
    b3_Shape shape;
    shape.kind = B3_SHAPE_CAPSULE;
    shape.as.capsule = capsule;
    return shape;
}

b3_Shape b3_shape_from_hull(b3_ConvexHull hull) {
    b3_Shape shape;
    shape.kind = B3_SHAPE_HULL;
    shape.as.hull = hull;
    return shape;
}

b3_Shape b3_shape_from_compound(b3_Compound compound) {
    b3_Shape shape;
    shape.kind = B3_SHAPE_COMPOUND;
    shape.as.compound = compound;
    return shape;
}

b3_Shape b3_shape_from_mesh(b3_TriangleMesh mesh) {
    b3_Shape shape;
    shape.kind = B3_SHAPE_MESH;
    shape.as.mesh = mesh;
    return shape;
}

b3_Shape b3_shape_from_heightfield(b3_HeightField heightfield) {
    b3_Shape shape;
    shape.kind = B3_SHAPE_HEIGHTFIELD;
    shape.as.heightfield = heightfield;
    return shape;
}

b3_Vec3 b3_shape_center(const b3_Shape *shape) {
    switch (shape->kind) {
        case B3_SHAPE_SPHERE:
            return shape->as.sphere.center;
        case B3_SHAPE_CAPSULE:
            return shape->as.capsule.center;
        case B3_SHAPE_HULL:
            return shape->as.hull.center;
        case B3_SHAPE_COMPOUND:
            return shape->as.compound.center;
        case B3_SHAPE_MESH:
            return shape->as.mesh.center;
        case B3_SHAPE_HEIGHTFIELD:
            return shape->as.heightfield.center;
        case B3_SHAPE_BOX:
        default:
            return shape->as.box.center;
    }
}

void b3_shape_set_pose(b3_Shape *shape, b3_Vec3 center, b3_Quat orientation) {
    switch (shape->kind) {
        case B3_SHAPE_SPHERE:
            shape->as.sphere.center = center;
            break;
        case B3_SHAPE_CAPSULE:
            shape->as.capsule.center = center;
            shape->as.capsule.orientation = orientation;
            break;
        case B3_SHAPE_HULL:
            shape->as.hull.center = center;
            shape->as.hull.orientation = orientation;
            break;
        case B3_SHAPE_COMPOUND:
            shape->as.compound.center = center;
            shape->as.compound.orientation = orientation;
            break;
        case B3_SHAPE_MESH:
            shape->as.mesh.center = center;
            shape->as.mesh.orientation = orientation;
            break;
        case B3_SHAPE_HEIGHTFIELD:
            shape->as.heightfield.center = center;
            shape->as.heightfield.orientation = orientation;
            break;
        case B3_SHAPE_BOX:
        default:
            shape->as.box.center = center;
            shape->as.box.orientation = orientation;
            break;
    }
}

void b3_shape_compute_aabb(const b3_Shape *shape, b3_Vec3 *out_min, b3_Vec3 *out_max) {
    switch (shape->kind) {
        case B3_SHAPE_SPHERE:
            b3_sphere_compute_aabb(&shape->as.sphere, out_min, out_max);
            return;
        case B3_SHAPE_CAPSULE:
            b3_capsule_compute_aabb(&shape->as.capsule, out_min, out_max);
            return;
        case B3_SHAPE_HULL:
            b3_convexhull_compute_aabb(&shape->as.hull, out_min, out_max);
            return;
        case B3_SHAPE_COMPOUND:
            b3_compound_compute_aabb(&shape->as.compound, out_min, out_max);
            return;
        case B3_SHAPE_MESH:
            b3_trianglemesh_compute_aabb(&shape->as.mesh, out_min, out_max);
            return;
        case B3_SHAPE_HEIGHTFIELD:
            b3_heightfield_compute_aabb(&shape->as.heightfield, out_min, out_max);
            return;
        case B3_SHAPE_BOX:
        default:
            b3_box3d_compute_aabb(&shape->as.box, out_min, out_max);
            return;
    }
}

int b3_shape_contains_point(const b3_Shape *shape, b3_Vec3 point) {
    switch (shape->kind) {
        case B3_SHAPE_SPHERE:
            return b3_sphere_contains_point(&shape->as.sphere, point);
        case B3_SHAPE_CAPSULE:
            return b3_capsule_contains_point(&shape->as.capsule, point);
        case B3_SHAPE_HULL:
            return b3_convexhull_contains_point(&shape->as.hull, point);
        case B3_SHAPE_COMPOUND:
            return b3_compound_contains_point(&shape->as.compound, point);
        case B3_SHAPE_MESH:
            return b3_trianglemesh_contains_point(&shape->as.mesh, point);
        case B3_SHAPE_HEIGHTFIELD:
            return b3_heightfield_contains_point(&shape->as.heightfield, point);
        case B3_SHAPE_BOX:
        default:
            return b3_box3d_contains_point(&shape->as.box, point);
    }
}

/* Meaningless for the non-convex kinds (mesh/height-field) -- they never
 * reach GJK/EPA directly (b3_shape_overlap special-cases them below), so
 * this is only a harmless fallback for a caller that calls it on one
 * directly. */
b3_Vec3 b3_shape_support(const b3_Shape *shape, b3_Vec3 direction) {
    switch (shape->kind) {
        case B3_SHAPE_SPHERE:
            return b3_sphere_support(&shape->as.sphere, direction);
        case B3_SHAPE_CAPSULE:
            return b3_capsule_support(&shape->as.capsule, direction);
        case B3_SHAPE_HULL:
            return b3_convexhull_support(&shape->as.hull, direction);
        case B3_SHAPE_COMPOUND:
            return b3_compound_support(&shape->as.compound, direction);
        case B3_SHAPE_MESH:
        case B3_SHAPE_HEIGHTFIELD:
            return b3_shape_center(shape);
        case B3_SHAPE_BOX:
        default:
            return b3_box3d_support(&shape->as.box, direction);
    }
}

b3_RayHit b3_shape_raycast(const b3_Shape *shape, b3_Vec3 origin, b3_Vec3 dir, b3_real max_t) {
    switch (shape->kind) {
        case B3_SHAPE_SPHERE:
            return b3_sphere_raycast(&shape->as.sphere, origin, dir, max_t);
        case B3_SHAPE_CAPSULE:
            return b3_capsule_raycast(&shape->as.capsule, origin, dir, max_t);
        case B3_SHAPE_HULL:
            return b3_convexhull_raycast(&shape->as.hull, origin, dir, max_t);
        case B3_SHAPE_COMPOUND:
            return b3_compound_raycast(&shape->as.compound, origin, dir, max_t);
        case B3_SHAPE_MESH:
            return b3_trianglemesh_raycast(&shape->as.mesh, origin, dir, max_t);
        case B3_SHAPE_HEIGHTFIELD:
            return b3_heightfield_raycast(&shape->as.heightfield, origin, dir, max_t);
        case B3_SHAPE_BOX:
        default:
            return b3_box3d_raycast(&shape->as.box, origin, dir, max_t);
    }
}

static b3_Vec3 shape_support_adapter(const void *shape, b3_Vec3 direction) {
    return b3_shape_support((const b3_Shape *)shape, direction);
}

static int is_mesh_like(const b3_Shape *shape) {
    return shape->kind == B3_SHAPE_MESH || shape->kind == B3_SHAPE_HEIGHTFIELD;
}

int b3_shape_overlap(const b3_Shape *a, const b3_Shape *b, b3_ContactInfo *out) {
    if (a->kind == B3_SHAPE_BOX && b->kind == B3_SHAPE_BOX) {
        return b3_box3d_overlap(&a->as.box, &b->as.box, out);
    }

    int a_mesh_like = is_mesh_like(a);
    int b_mesh_like = is_mesh_like(b);

    if (a_mesh_like && b_mesh_like) {
        /* v1: mesh/height-field vs mesh/height-field isn't supported
         * (both sides are static-only anyway, so b3_world_step never
         * hits this) -- see docs/limitations.md. */
        return 0;
    }

    if (a_mesh_like || b_mesh_like) {
        const b3_Shape *mesh_side = a_mesh_like ? a : b;
        const b3_Shape *other_side = a_mesh_like ? b : a;
        b3_Vec3 other_lo, other_hi;
        b3_shape_compute_aabb(other_side, &other_lo, &other_hi);

        b3_ContactInfo contact;
        int hit;
        if (mesh_side->kind == B3_SHAPE_MESH) {
            hit = b3_trianglemesh_overlap_shape(
                &mesh_side->as.mesh, other_side, shape_support_adapter, other_lo, other_hi, &contact
            );
        } else {
            hit = b3_heightfield_overlap_shape(
                &mesh_side->as.heightfield, other_side, shape_support_adapter, other_lo, other_hi,
                &contact
            );
        }
        if (!hit) {
            return 0;
        }
        if (out) {
            *out = contact;
            if (!a_mesh_like) {
                /* The mesh/height-field side is always "a" in the
                 * internal triangle-vs-shape call, so its normal points
                 * mesh->other; flip it to match this call's a->b
                 * ordering when the caller's mesh side was "b" instead. */
                out->normal = b3_vec3_negate(out->normal);
            }
        }
        return 1;
    }

    b3_GjkInput input;
    input.shape_a = a;
    input.support_a = shape_support_adapter;
    input.shape_b = b;
    input.support_b = shape_support_adapter;
    return b3_gjk_epa_overlap(&input, out);
}
