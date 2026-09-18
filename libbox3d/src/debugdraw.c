#include "box3d/debugdraw.h"

#include <math.h>
#include <stddef.h>

#define B3_DEBUG_PI 3.14159265358979323846f

static void emit_line(b3_DebugLine *out, int max, int *count, b3_Vec3 a, b3_Vec3 b) {
    if (*count < max) {
        out[*count].a = a;
        out[*count].b = b;
    }
    *count += 1;
}

/* Mirrors world.c's own static any_perpendicular/perpendicular_basis --
 * not shared across translation units, same rationale as that file's own
 * comment on the same duplication (each collision/geometry file stays
 * self-contained). */
static b3_Vec3 debug_any_perpendicular(b3_Vec3 v) {
    b3_Vec3 p = b3_vec3_cross(v, b3_vec3_make(0.0f, 1.0f, 0.0f));
    if (b3_vec3_length_sq(p) < 1e-10f) {
        p = b3_vec3_cross(v, b3_vec3_make(1.0f, 0.0f, 0.0f));
    }
    return b3_vec3_normalize(p);
}

static void debug_perpendicular_basis(b3_Vec3 axis, b3_Vec3 *perp1, b3_Vec3 *perp2) {
    *perp1 = debug_any_perpendicular(axis);
    *perp2 = b3_vec3_normalize(b3_vec3_cross(axis, *perp1));
}

static void debug_lines_circle(
    b3_Vec3 center, b3_Vec3 axis1, b3_Vec3 axis2, b3_real radius, b3_DebugLine *out, int max,
    int *count
) {
    b3_Vec3 prev = b3_vec3_add(center, b3_vec3_scale(axis1, radius));
    for (int i = 1; i <= B3_DEBUG_CIRCLE_SEGMENTS; i++) {
        b3_real t = (b3_real)i / (b3_real)B3_DEBUG_CIRCLE_SEGMENTS * 2.0f * B3_DEBUG_PI;
        b3_Vec3 next = b3_vec3_add(
            center,
            b3_vec3_add(b3_vec3_scale(axis1, radius * cosf(t)), b3_vec3_scale(axis2, radius * sinf(t)))
        );
        emit_line(out, max, count, prev, next);
        prev = next;
    }
}

static void debug_lines_box(const b3_Box3D *box, b3_DebugLine *out, int max, int *count) {
    b3_Vec3 h = box->half_extents;
    b3_Vec3 corners[8];
    int idx = 0;
    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sy = -1; sy <= 1; sy += 2) {
            for (int sz = -1; sz <= 1; sz += 2) {
                b3_Vec3 local = b3_vec3_make((b3_real)sx * h.x, (b3_real)sy * h.y, (b3_real)sz * h.z);
                corners[idx++] =
                    b3_vec3_add(box->center, b3_quat_rotate_vec3(box->orientation, local));
            }
        }
    }
    /* corners[i] varies sx/sy/sz as the top/middle/bottom bit of i (see
     * the nested loop above) -- edges connect corners differing in
     * exactly one bit, the standard 12-edge cube graph. */
    static const int edges[12][2] = {
        {0, 1}, {0, 2}, {0, 4}, {1, 3}, {1, 5}, {2, 3},
        {2, 6}, {3, 7}, {4, 5}, {4, 6}, {5, 7}, {6, 7},
    };
    for (int i = 0; i < 12; i++) {
        emit_line(out, max, count, corners[edges[i][0]], corners[edges[i][1]]);
    }
}

static void debug_lines_aabb_box_of(b3_Vec3 min, b3_Vec3 max, b3_DebugLine *out, int max_lines, int *count) {
    b3_Box3D box = b3_box3d_make_aabb(
        b3_vec3_scale(b3_vec3_add(min, max), 0.5f), b3_vec3_scale(b3_vec3_sub(max, min), 0.5f)
    );
    debug_lines_box(&box, out, max_lines, count);
}

static void debug_lines_sphere(const b3_Sphere *sphere, b3_DebugLine *out, int max, int *count) {
    static const b3_Vec3 x = {1.0f, 0.0f, 0.0f};
    static const b3_Vec3 y = {0.0f, 1.0f, 0.0f};
    static const b3_Vec3 z = {0.0f, 0.0f, 1.0f};
    debug_lines_circle(sphere->center, x, y, sphere->radius, out, max, count);
    debug_lines_circle(sphere->center, x, z, sphere->radius, out, max, count);
    debug_lines_circle(sphere->center, y, z, sphere->radius, out, max, count);
}

static void debug_lines_capsule(const b3_Capsule *capsule, b3_DebugLine *out, int max, int *count) {
    b3_Vec3 p0, p1;
    b3_capsule_segment(capsule, &p0, &p1);
    b3_Vec3 axis = b3_vec3_normalize(b3_vec3_sub(p1, p0));
    b3_Vec3 perp1, perp2;
    debug_perpendicular_basis(axis, &perp1, &perp2);
    debug_lines_circle(p0, perp1, perp2, capsule->radius, out, max, count);
    debug_lines_circle(p1, perp1, perp2, capsule->radius, out, max, count);
    for (int k = 0; k < 4; k++) {
        b3_real t = (b3_real)k / 4.0f * 2.0f * B3_DEBUG_PI;
        b3_Vec3 offset = b3_vec3_add(
            b3_vec3_scale(perp1, capsule->radius * cosf(t)), b3_vec3_scale(perp2, capsule->radius * sinf(t))
        );
        emit_line(out, max, count, b3_vec3_add(p0, offset), b3_vec3_add(p1, offset));
    }
}

static void debug_lines_hull(const b3_ConvexHull *hull, b3_DebugLine *out, int max, int *count) {
    b3_Vec3 min, max_pt;
    b3_convexhull_compute_aabb(hull, &min, &max_pt);
    debug_lines_aabb_box_of(min, max_pt, out, max, count);
}

static void debug_lines_compound(const b3_Compound *compound, b3_DebugLine *out, int max, int *count) {
    for (int i = 0; i < compound->child_count; i++) {
        b3_Vec3 world_center;
        b3_Quat world_orientation;
        b3_compound_child_world_pose(compound, i, &world_center, &world_orientation);
        const b3_CompoundChild *child = &compound->children[i];
        switch (child->kind) {
            case B3_COMPOUND_CHILD_BOX: {
                b3_Box3D box = child->as.box;
                box.center = world_center;
                box.orientation = world_orientation;
                debug_lines_box(&box, out, max, count);
                break;
            }
            case B3_COMPOUND_CHILD_SPHERE: {
                b3_Sphere sphere = child->as.sphere;
                sphere.center = world_center;
                debug_lines_sphere(&sphere, out, max, count);
                break;
            }
            case B3_COMPOUND_CHILD_CAPSULE: {
                b3_Capsule capsule = child->as.capsule;
                capsule.center = world_center;
                capsule.orientation = world_orientation;
                debug_lines_capsule(&capsule, out, max, count);
                break;
            }
            case B3_COMPOUND_CHILD_HULL:
            default: {
                b3_ConvexHull hull = child->as.hull;
                hull.center = world_center;
                hull.orientation = world_orientation;
                debug_lines_hull(&hull, out, max, count);
                break;
            }
        }
    }
}

static void debug_lines_mesh(const b3_TriangleMesh *mesh, b3_DebugLine *out, int max, int *count) {
    for (int i = 0; i < mesh->triangle_count; i++) {
        b3_Vec3 tri[3];
        b3_trianglemesh_triangle_world(mesh, i, tri);
        emit_line(out, max, count, tri[0], tri[1]);
        emit_line(out, max, count, tri[1], tri[2]);
        emit_line(out, max, count, tri[2], tri[0]);
    }
}

static void debug_lines_heightfield(const b3_HeightField *hf, b3_DebugLine *out, int max, int *count) {
    for (int r = 0; r < hf->rows; r++) {
        for (int c = 0; c < hf->cols - 1; c++) {
            emit_line(
                out, max, count, b3_heightfield_world_vertex(hf, r, c),
                b3_heightfield_world_vertex(hf, r, c + 1)
            );
        }
    }
    for (int c = 0; c < hf->cols; c++) {
        for (int r = 0; r < hf->rows - 1; r++) {
            emit_line(
                out, max, count, b3_heightfield_world_vertex(hf, r, c),
                b3_heightfield_world_vertex(hf, r + 1, c)
            );
        }
    }
}

int b3_shape_debug_lines(const b3_Shape *shape, b3_DebugLine *out_lines, int max_lines) {
    int count = 0;
    switch (shape->kind) {
        case B3_SHAPE_BOX:
            debug_lines_box(&shape->as.box, out_lines, max_lines, &count);
            break;
        case B3_SHAPE_SPHERE:
            debug_lines_sphere(&shape->as.sphere, out_lines, max_lines, &count);
            break;
        case B3_SHAPE_CAPSULE:
            debug_lines_capsule(&shape->as.capsule, out_lines, max_lines, &count);
            break;
        case B3_SHAPE_HULL:
            debug_lines_hull(&shape->as.hull, out_lines, max_lines, &count);
            break;
        case B3_SHAPE_COMPOUND:
            debug_lines_compound(&shape->as.compound, out_lines, max_lines, &count);
            break;
        case B3_SHAPE_MESH:
            debug_lines_mesh(&shape->as.mesh, out_lines, max_lines, &count);
            break;
        case B3_SHAPE_HEIGHTFIELD:
            debug_lines_heightfield(&shape->as.heightfield, out_lines, max_lines, &count);
            break;
    }
    return count;
}

int b3_world_debug_contacts(const b3_World *world, b3_DebugContact *out_contacts, int max_contacts) {
    int count = 0;
    for (int i = 0; i < world->body_count; i++) {
        for (int j = i + 1; j < world->body_count; j++) {
            b3_ContactInfo contact;
            if (!b3_shape_overlap(&world->bodies[i].shape, &world->bodies[j].shape, &contact)) {
                continue;
            }
            if (count < max_contacts) {
                out_contacts[count].point = contact.point;
                out_contacts[count].normal = contact.normal;
            }
            count++;
        }
    }
    return count;
}

int b3_world_debug_joint_anchors(
    const b3_World *world, b3_DebugJointAnchor *out_anchors, int max_anchors
) {
    for (int i = 0; i < world->joint_count; i++) {
        const b3_Joint *joint = &world->joints[i];
        /* b3_world_get_body_by_id only needs a non-const b3_World* to
         * return a non-const b3_RigidBody* elsewhere in the API -- this
         * call itself never mutates `world`. */
        b3_RigidBody *a = b3_world_get_body_by_id((b3_World *)world, joint->body_a_id);
        b3_RigidBody *b = b3_world_get_body_by_id((b3_World *)world, joint->body_b_id);
        if (a == NULL || b == NULL) {
            continue; /* defensive: b3_world_remove_body already prevents this */
        }
        if (i < max_anchors) {
            out_anchors[i].anchor_a = b3_vec3_add(a->position, b3_quat_rotate_vec3(a->orientation, joint->anchor_a));
            out_anchors[i].anchor_b = b3_vec3_add(b->position, b3_quat_rotate_vec3(b->orientation, joint->anchor_b));
        }
    }
    return world->joint_count;
}
