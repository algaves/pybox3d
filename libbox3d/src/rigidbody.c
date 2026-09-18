#include "box3d/rigidbody.h"

#include <stddef.h>
#include <math.h>

void b3_rigidbody_init_shape(b3_RigidBody *body, b3_Shape shape, b3_Vec3 position, b3_real mass) {
    body->position = position;
    body->orientation = b3_quat_identity();
    body->linear_velocity = b3_vec3_zero();
    body->angular_velocity = b3_vec3_zero();

    body->force_accum = b3_vec3_zero();
    body->torque_accum = b3_vec3_zero();

    body->shape = shape;
    b3_shape_set_pose(&body->shape, position, body->orientation);

    body->restitution = 0.2f;
    body->friction = 0.5f;
    body->user_data = NULL;
    body->sleep_timer = 0.0f;
    body->is_sleeping = 0;
    body->slot_id = b3_id_invalid();

    b3_rigidbody_set_mass(body, mass);
}

void b3_rigidbody_init_box(b3_RigidBody *body, b3_Vec3 position, b3_Vec3 half_extents, b3_real mass) {
    b3_rigidbody_init_shape(
        body, b3_shape_from_box(b3_box3d_make_obb(position, half_extents, b3_quat_identity())),
        position, mass
    );
}

void b3_rigidbody_init_sphere(b3_RigidBody *body, b3_Vec3 position, b3_real radius, b3_real mass) {
    b3_rigidbody_init_shape(body, b3_shape_from_sphere(b3_sphere_make(position, radius)), position, mass);
}

void b3_rigidbody_init_capsule(
    b3_RigidBody *body, b3_Vec3 position, b3_real radius, b3_real half_height, b3_real mass
) {
    b3_rigidbody_init_shape(
        body,
        b3_shape_from_capsule(b3_capsule_make(position, b3_quat_identity(), radius, half_height)),
        position, mass
    );
}

void b3_rigidbody_init_hull(
    b3_RigidBody *body, b3_Vec3 position, const b3_Vec3 *local_vertices, int vertex_count, b3_real mass
) {
    b3_rigidbody_init_shape(
        body,
        b3_shape_from_hull(b3_convexhull_make(position, b3_quat_identity(), local_vertices, vertex_count)),
        position, mass
    );
}

void b3_rigidbody_init_compound(
    b3_RigidBody *body, b3_Vec3 position, const b3_CompoundChild *children, int child_count, b3_real mass
) {
    b3_rigidbody_init_shape(
        body,
        b3_shape_from_compound(b3_compound_make(position, b3_quat_identity(), children, child_count)),
        position, mass
    );
}

void b3_rigidbody_init_mesh(
    b3_RigidBody *body, b3_Vec3 position, const b3_Vec3 *local_triangle_vertices, int triangle_count
) {
    b3_rigidbody_init_shape(
        body,
        b3_shape_from_mesh(b3_trianglemesh_make(position, b3_quat_identity(), local_triangle_vertices, triangle_count)),
        position, 0.0f
    );
}

void b3_rigidbody_init_heightfield(
    b3_RigidBody *body, b3_Vec3 position, b3_real cell_size,
    const b3_real *row_major_heights, int rows, int cols
) {
    b3_rigidbody_init_shape(
        body,
        b3_shape_from_heightfield(
            b3_heightfield_make(position, b3_quat_identity(), cell_size, row_major_heights, rows, cols)
        ),
        position, 0.0f
    );
}

static void box_inertia_diag(b3_real mass, b3_Vec3 half_extents, b3_real out[3]) {
    b3_real dx = 2.0f * half_extents.x, dy = 2.0f * half_extents.y, dz = 2.0f * half_extents.z;
    out[0] = (mass / 12.0f) * (dy * dy + dz * dz);
    out[1] = (mass / 12.0f) * (dx * dx + dz * dz);
    out[2] = (mass / 12.0f) * (dx * dx + dy * dy);
}

static void sphere_inertia_diag(b3_real mass, b3_real radius, b3_real out[3]) {
    b3_real i = 0.4f * mass * radius * radius; /* solid sphere: (2/5) m r^2, same about any axis */
    out[0] = out[1] = out[2] = i;
}

/* Capsule = a cylinder (axis along the local Y this diagonal is
 * expressed in) capped by two hemispheres. Mass is split between the
 * two pieces by volume, and each piece's own closed-form inertia is
 * combined via the parallel-axis theorem back onto the capsule's
 * center. The hemisphere term uses the standard (83/320) m r^2
 * solid-hemisphere-about-its-own-centroid result and re-offsets it by
 * (half_height + 3r/8), the centroid's distance from the capsule
 * center. */
static void capsule_inertia_diag(b3_real mass, b3_real radius, b3_real half_height, b3_real out[3]) {
    b3_real r = radius;
    b3_real h = 2.0f * half_height;

    b3_real vol_cyl = (b3_real)3.14159265358979323846 * r * r * h;
    b3_real vol_sphere = (4.0f / 3.0f) * (b3_real)3.14159265358979323846 * r * r * r;
    b3_real vol_total = vol_cyl + vol_sphere;
    if (vol_total < 1e-12f) {
        out[0] = out[1] = out[2] = 0.0f;
        return;
    }

    b3_real m_cyl = mass * (vol_cyl / vol_total);
    b3_real m_hemi = mass * (vol_sphere / vol_total); /* both caps combined */

    b3_real i_axis = 0.5f * m_cyl * r * r + m_hemi * 0.4f * r * r;

    b3_real i_cyl_perp = m_cyl * (3.0f * r * r + h * h) / 12.0f;
    b3_real hemi_offset = half_height + (3.0f / 8.0f) * r;
    b3_real i_hemi_perp = m_hemi * (83.0f / 320.0f) * r * r + m_hemi * hemi_offset * hemi_offset;
    b3_real i_perp = i_cyl_perp + i_hemi_perp;

    out[0] = i_perp;
    out[1] = i_axis;
    out[2] = i_perp;
}

static void local_hull_aabb(const b3_ConvexHull *hull, b3_Vec3 *out_min, b3_Vec3 *out_max) {
    if (hull->vertex_count == 0) {
        *out_min = *out_max = b3_vec3_zero();
        return;
    }
    b3_Vec3 lo = hull->local_vertices[0], hi = lo;
    for (int i = 1; i < hull->vertex_count; i++) {
        b3_Vec3 v = hull->local_vertices[i];
        lo.x = fminf(lo.x, v.x);
        lo.y = fminf(lo.y, v.y);
        lo.z = fminf(lo.z, v.z);
        hi.x = fmaxf(hi.x, v.x);
        hi.y = fmaxf(hi.y, v.y);
        hi.z = fmaxf(hi.z, v.z);
    }
    *out_min = lo;
    *out_max = hi;
}

/* v1 approximation: treat the hull as a solid box of its own local AABB.
 * Exact convex inertia needs a tetrahedron decomposition of the hull's
 * volume, out of scope for v1 -- see docs/limitations.md. */
static void hull_inertia_diag(b3_real mass, const b3_ConvexHull *hull, b3_real out[3]) {
    b3_Vec3 lo, hi;
    local_hull_aabb(hull, &lo, &hi);
    box_inertia_diag(mass, b3_vec3_scale(b3_vec3_sub(hi, lo), 0.5f), out);
}

static void compute_box_inertia(b3_RigidBody *body, b3_real mass) {
    box_inertia_diag(mass, body->shape.as.box.half_extents, body->inertia_local);
}

static void compute_sphere_inertia(b3_RigidBody *body, b3_real mass) {
    sphere_inertia_diag(mass, body->shape.as.sphere.radius, body->inertia_local);
}

static void compute_capsule_inertia(b3_RigidBody *body, b3_real mass) {
    capsule_inertia_diag(
        mass, body->shape.as.capsule.radius, body->shape.as.capsule.half_height, body->inertia_local
    );
}

static void compute_hull_inertia(b3_RigidBody *body, b3_real mass) {
    hull_inertia_diag(mass, &body->shape.as.hull, body->inertia_local);
}

/* v1 approximation: mass is split equally across children, each child's
 * own inertia is computed as if it alone had that mass, and the result
 * is accumulated onto the diagonal via the parallel-axis theorem using
 * only the child's offset (not its local orientation) -- a rotated
 * child's true inertia tensor isn't generally diagonal in the parent's
 * axes. Exact composition needs full 3x3 tensor math, out of scope for
 * v1 -- see docs/limitations.md. */
static void compute_compound_inertia(b3_RigidBody *body, b3_real mass) {
    const b3_Compound *compound = &body->shape.as.compound;
    body->inertia_local[0] = body->inertia_local[1] = body->inertia_local[2] = 0.0f;
    if (compound->child_count == 0) {
        return;
    }

    b3_real child_mass = mass / (b3_real)compound->child_count;
    for (int i = 0; i < compound->child_count; i++) {
        const b3_CompoundChild *child = &compound->children[i];
        b3_real local_inertia[3];
        switch (child->kind) {
            case B3_COMPOUND_CHILD_SPHERE:
                sphere_inertia_diag(child_mass, child->as.sphere.radius, local_inertia);
                break;
            case B3_COMPOUND_CHILD_CAPSULE:
                capsule_inertia_diag(
                    child_mass, child->as.capsule.radius, child->as.capsule.half_height, local_inertia
                );
                break;
            case B3_COMPOUND_CHILD_HULL:
                hull_inertia_diag(child_mass, &child->as.hull, local_inertia);
                break;
            case B3_COMPOUND_CHILD_BOX:
            default:
                box_inertia_diag(child_mass, child->as.box.half_extents, local_inertia);
                break;
        }

        b3_Vec3 r = child->local_position;
        body->inertia_local[0] += local_inertia[0] + child_mass * (r.y * r.y + r.z * r.z);
        body->inertia_local[1] += local_inertia[1] + child_mass * (r.x * r.x + r.z * r.z);
        body->inertia_local[2] += local_inertia[2] + child_mass * (r.x * r.x + r.y * r.y);
    }
}

void b3_rigidbody_set_mass(b3_RigidBody *body, b3_real mass) {
    body->mass = mass;
    if (mass <= 0.0f) {
        body->mass = 0.0f;
        body->inv_mass = 0.0f;
        body->inertia_local[0] = body->inertia_local[1] = body->inertia_local[2] = 0.0f;
        body->inv_inertia_local[0] = body->inv_inertia_local[1] = body->inv_inertia_local[2] = 0.0f;
        return;
    }

    body->inv_mass = 1.0f / mass;

    switch (body->shape.kind) {
        case B3_SHAPE_SPHERE:
            compute_sphere_inertia(body, mass);
            break;
        case B3_SHAPE_CAPSULE:
            compute_capsule_inertia(body, mass);
            break;
        case B3_SHAPE_HULL:
            compute_hull_inertia(body, mass);
            break;
        case B3_SHAPE_COMPOUND:
            compute_compound_inertia(body, mass);
            break;
        case B3_SHAPE_MESH:
        case B3_SHAPE_HEIGHTFIELD:
            /* Mesh/height-field bodies are always static (see
             * b3_rigidbody_init_mesh/init_heightfield, which always pass
             * mass=0 and so never reach this branch); zero inertia is a
             * harmless fallback if mass is ever set directly anyway. */
            body->inertia_local[0] = body->inertia_local[1] = body->inertia_local[2] = 0.0f;
            break;
        case B3_SHAPE_BOX:
        default:
            compute_box_inertia(body, mass);
            break;
    }

    for (int i = 0; i < 3; i++) {
        body->inv_inertia_local[i] =
            body->inertia_local[i] > 1e-12f ? 1.0f / body->inertia_local[i] : 0.0f;
    }
}

b3_Vec3 b3_rigidbody_world_inv_inertia(const b3_RigidBody *body, b3_Vec3 world_vec) {
    b3_Quat inv_orientation = b3_quat_conjugate(body->orientation);
    b3_Vec3 local = b3_quat_rotate_vec3(inv_orientation, world_vec);
    b3_Vec3 scaled = b3_vec3_make(
        local.x * body->inv_inertia_local[0],
        local.y * body->inv_inertia_local[1],
        local.z * body->inv_inertia_local[2]
    );
    return b3_quat_rotate_vec3(body->orientation, scaled);
}

void b3_rigidbody_apply_force(b3_RigidBody *body, b3_Vec3 force, b3_Vec3 world_point) {
    if (body->inv_mass == 0.0f) {
        return;
    }
    body->force_accum = b3_vec3_add(body->force_accum, force);
    b3_Vec3 r = b3_vec3_sub(world_point, body->position);
    body->torque_accum = b3_vec3_add(body->torque_accum, b3_vec3_cross(r, force));
}

/* Deliberately does NOT call b3_rigidbody_wake(): this is also the
 * internal primitive b3_world_step's own joint/contact solver uses for
 * every impulse it applies (including tiny converged-equilibrium
 * residuals from a resting contact/joint, every step) -- auto-waking here
 * would mean a sleeping body touching anything never actually sleeps.
 * The *external*, user-facing entry points that should wake a body
 * (Python's RigidBody.apply_force()/.apply_impulse(), and the
 * position/orientation/velocity setters) call b3_rigidbody_wake()
 * themselves instead -- see py_rigidbody.c. */
void b3_rigidbody_apply_impulse(b3_RigidBody *body, b3_Vec3 impulse, b3_Vec3 world_point) {
    if (body->inv_mass == 0.0f) {
        return;
    }
    body->linear_velocity = b3_vec3_add(body->linear_velocity, b3_vec3_scale(impulse, body->inv_mass));

    b3_Vec3 r = b3_vec3_sub(world_point, body->position);
    b3_Vec3 angular_impulse = b3_vec3_cross(r, impulse);
    b3_Vec3 delta_angular = b3_rigidbody_world_inv_inertia(body, angular_impulse);
    body->angular_velocity = b3_vec3_add(body->angular_velocity, delta_angular);
}

void b3_rigidbody_clear_accumulators(b3_RigidBody *body) {
    body->force_accum = b3_vec3_zero();
    body->torque_accum = b3_vec3_zero();
}

void b3_rigidbody_wake(b3_RigidBody *body) {
    body->is_sleeping = 0;
    body->sleep_timer = 0.0f;
}

void b3_rigidbody_sync_shape(b3_RigidBody *body) {
    b3_shape_set_pose(&body->shape, body->position, body->orientation);
}

void b3_rigidbody_integrate(b3_RigidBody *body, b3_Vec3 gravity, b3_real dt) {
    if (body->inv_mass == 0.0f || body->is_sleeping) {
        b3_rigidbody_clear_accumulators(body);
        return;
    }

    b3_Vec3 linear_accel = b3_vec3_add(gravity, b3_vec3_scale(body->force_accum, body->inv_mass));
    body->linear_velocity = b3_vec3_add(body->linear_velocity, b3_vec3_scale(linear_accel, dt));

    b3_Vec3 angular_accel = b3_rigidbody_world_inv_inertia(body, body->torque_accum);
    body->angular_velocity = b3_vec3_add(body->angular_velocity, b3_vec3_scale(angular_accel, dt));

    body->position = b3_vec3_add(body->position, b3_vec3_scale(body->linear_velocity, dt));
    body->orientation = b3_quat_integrate(body->orientation, body->angular_velocity, dt);

    b3_rigidbody_sync_shape(body);
    b3_rigidbody_clear_accumulators(body);
}
