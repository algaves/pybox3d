#include "box3d/gjk.h"

#include <math.h>

/* GJK (boolean convex intersection) + EPA (penetration depth/normal
 * extraction) over the Minkowski difference of two support-function
 * shapes. Standard reference algorithms (see e.g. Ericson, "Real-Time
 * Collision Detection" ch. 9, or the widely-used "Implementing GJK"
 * write-ups) -- this file only wires them to b3_SupportFn. */

static b3_Vec3 minkowski_support(const b3_GjkInput *input, b3_Vec3 dir) {
    b3_Vec3 pa = input->support_a(input->shape_a, dir);
    b3_Vec3 pb = input->support_b(input->shape_b, b3_vec3_negate(dir));
    return b3_vec3_sub(pa, pb);
}

static int same_direction(b3_Vec3 dir, b3_Vec3 v) {
    return b3_vec3_dot(dir, v) > 0.0f;
}

static b3_Vec3 triple_cross(b3_Vec3 a, b3_Vec3 b, b3_Vec3 c) {
    return b3_vec3_cross(b3_vec3_cross(a, b), c);
}

/* Some vector perpendicular to `v` (arbitrary but deterministic). Falls
 * back to a second axis if `v` happens to be parallel to the first
 * choice. Used whenever a "perpendicular to this edge, toward the
 * origin" direction (via a double cross product) degenerates to zero --
 * which happens whenever the relevant points are exactly collinear, a
 * routine occurrence for symmetric configurations (e.g. two spheres
 * whose centers lie on a common axis), not just numerical noise. */
static b3_Vec3 any_perpendicular(b3_Vec3 v) {
    b3_Vec3 p = b3_vec3_cross(v, b3_vec3_make(0.0f, 1.0f, 0.0f));
    if (b3_vec3_length_sq(p) < 1e-10f) {
        p = b3_vec3_cross(v, b3_vec3_make(1.0f, 0.0f, 0.0f));
    }
    return p;
}

typedef struct {
    b3_Vec3 p[4]; /* p[0] is always the most recently added point */
    int n;
} b3_Simplex;

static void set_line(b3_Simplex *s, b3_Vec3 a, b3_Vec3 b, b3_Vec3 *dir) {
    b3_Vec3 ab = b3_vec3_sub(b, a);
    b3_Vec3 ao = b3_vec3_negate(a);
    if (same_direction(ab, ao)) {
        s->p[0] = a;
        s->p[1] = b;
        s->n = 2;
        b3_Vec3 d = triple_cross(ab, ao, ab);
        *dir = b3_vec3_length_sq(d) < 1e-10f ? any_perpendicular(ab) : d;
    } else {
        s->p[0] = a;
        s->n = 1;
        *dir = ao;
    }
}

static int line_case(b3_Simplex *s, b3_Vec3 *dir) {
    set_line(s, s->p[0], s->p[1], dir);
    return 0;
}

static int triangle_case(b3_Simplex *s, b3_Vec3 *dir) {
    b3_Vec3 a = s->p[0], b = s->p[1], c = s->p[2];
    b3_Vec3 ab = b3_vec3_sub(b, a), ac = b3_vec3_sub(c, a), ao = b3_vec3_negate(a);
    b3_Vec3 abc = b3_vec3_cross(ab, ac);

    if (b3_vec3_length_sq(abc) < 1e-10f) {
        /* a, b, c are (numerically) collinear -- there's no well-defined
         * triangle normal to branch on. Drop back to the line case
         * through the two most recent points; its own degenerate-case
         * handling picks a perpendicular escape direction. */
        set_line(s, a, b, dir);
        return 0;
    }

    if (same_direction(b3_vec3_cross(abc, ac), ao)) {
        if (same_direction(ac, ao)) {
            s->p[0] = a;
            s->p[1] = c;
            s->n = 2;
            *dir = triple_cross(ac, ao, ac);
        } else {
            set_line(s, a, b, dir);
        }
    } else if (same_direction(b3_vec3_cross(ab, abc), ao)) {
        set_line(s, a, b, dir);
    } else if (same_direction(abc, ao)) {
        s->p[0] = a;
        s->p[1] = b;
        s->p[2] = c;
        s->n = 3;
        *dir = abc;
    } else {
        s->p[0] = a;
        s->p[1] = c;
        s->p[2] = b;
        s->n = 3;
        *dir = b3_vec3_negate(abc);
    }
    return 0;
}

static int tetrahedron_case(b3_Simplex *s, b3_Vec3 *dir) {
    b3_Vec3 a = s->p[0], b = s->p[1], c = s->p[2], d = s->p[3];
    b3_Vec3 ab = b3_vec3_sub(b, a), ac = b3_vec3_sub(c, a), ad = b3_vec3_sub(d, a);
    b3_Vec3 ao = b3_vec3_negate(a);

    b3_Vec3 abc = b3_vec3_cross(ab, ac);
    b3_Vec3 acd = b3_vec3_cross(ac, ad);
    b3_Vec3 adb = b3_vec3_cross(ad, ab);

    if (same_direction(abc, ao)) {
        s->p[0] = a;
        s->p[1] = b;
        s->p[2] = c;
        s->n = 3;
        return triangle_case(s, dir);
    }
    if (same_direction(acd, ao)) {
        s->p[0] = a;
        s->p[1] = c;
        s->p[2] = d;
        s->n = 3;
        return triangle_case(s, dir);
    }
    if (same_direction(adb, ao)) {
        s->p[0] = a;
        s->p[1] = d;
        s->p[2] = b;
        s->n = 3;
        return triangle_case(s, dir);
    }
    return 1; /* origin is on the inner side of all four faces: enclosed */
}

#define EPA_MAX_VERTS 64
#define EPA_MAX_FACES 128

typedef struct {
    int idx[3];
    b3_Vec3 normal;
    b3_real dist; /* distance from the origin to this face's plane */
} EpaFace;

/* Outward-ish normal of triangle (a,b,c) (sign corrected by the caller).
 * Falls back to any_perpendicular when a,b,c are (numerically)
 * collinear, matching the GJK simplex cases' handling of the same
 * degeneracy -- otherwise normalize() of a near-zero cross product would
 * hand EPA a garbage (possibly zero) face normal. */
static b3_Vec3 face_normal(b3_Vec3 a, b3_Vec3 b, b3_Vec3 c) {
    b3_Vec3 ab = b3_vec3_sub(b, a);
    b3_Vec3 raw = b3_vec3_cross(ab, b3_vec3_sub(c, a));
    if (b3_vec3_length_sq(raw) < 1e-10f) {
        raw = any_perpendicular(ab);
    }
    return b3_vec3_normalize(raw);
}

static void epa_contact_point(
    const b3_GjkInput *input, b3_Vec3 normal, b3_real penetration, b3_ContactInfo *out
) {
    out->normal = normal;
    out->penetration = penetration;
    b3_Vec3 pa = input->support_a(input->shape_a, normal);
    b3_Vec3 pb = input->support_b(input->shape_b, b3_vec3_negate(normal));
    out->point = b3_vec3_scale(b3_vec3_add(pa, pb), 0.5f);
}

static int epa(const b3_GjkInput *input, const b3_Simplex *simplex, b3_ContactInfo *out) {
    b3_Vec3 verts[EPA_MAX_VERTS];
    int vert_count = 4;
    for (int i = 0; i < 4; i++) {
        verts[i] = simplex->p[i];
    }

    EpaFace faces[EPA_MAX_FACES];
    int face_count = 0;
    static const int tet_faces[4][3] = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
    for (int f = 0; f < 4; f++) {
        int ia = tet_faces[f][0], ib = tet_faces[f][1], ic = tet_faces[f][2];
        b3_Vec3 a = verts[ia], b = verts[ib], c = verts[ic];
        b3_Vec3 normal = face_normal(a, b, c);
        b3_real dist = b3_vec3_dot(normal, a);
        if (dist < 0.0f) {
            normal = b3_vec3_negate(normal);
            dist = -dist;
            int tmp = ib;
            ib = ic;
            ic = tmp;
        }
        faces[face_count].idx[0] = ia;
        faces[face_count].idx[1] = ib;
        faces[face_count].idx[2] = ic;
        faces[face_count].normal = normal;
        faces[face_count].dist = dist;
        face_count++;
    }

    int edges[EPA_MAX_FACES * 3][2];

    for (int iter = 0; iter < 128; iter++) {
        int closest = 0;
        b3_real closest_dist = faces[0].dist;
        for (int i = 1; i < face_count; i++) {
            if (faces[i].dist < closest_dist) {
                closest_dist = faces[i].dist;
                closest = i;
            }
        }

        b3_Vec3 normal = faces[closest].normal;
        b3_Vec3 support = minkowski_support(input, normal);
        b3_real d = b3_vec3_dot(normal, support);

        /* Convergence epsilon scaled to the magnitude of the values
         * involved, not a fixed absolute tolerance: shapes with large
         * coordinates (e.g. a sizeable mesh/height-field triangle) push
         * `d`/`closest_dist` up near float32's precision floor at that
         * scale, where a tiny fixed epsilon is dominated by rounding
         * noise -- sometimes terminating on a face nowhere near the true
         * minimum. Smooth/round shapes (spheres, capsules) still need
         * more polytope refinement than flat ones to converge to an
         * accurate normal, since each iteration only resolves one face
         * at a time against a curved surface. */
        b3_real eps = 1e-5f * (fabsf(d) + fabsf(closest_dist) + 1.0f);
        if (d - closest_dist < eps || vert_count >= EPA_MAX_VERTS) {
            if (out) {
                epa_contact_point(input, normal, closest_dist, out);
            }
            return 1;
        }

        int new_index = vert_count++;
        verts[new_index] = support;

        int edge_count = 0;
        for (int i = 0; i < face_count;) {
            if (b3_vec3_dot(faces[i].normal, b3_vec3_sub(support, verts[faces[i].idx[0]])) > 0.0f) {
                int tri[3] = {faces[i].idx[0], faces[i].idx[1], faces[i].idx[2]};
                for (int e = 0; e < 3; e++) {
                    int e0 = tri[e], e1 = tri[(e + 1) % 3];
                    int found = 0;
                    for (int k = 0; k < edge_count; k++) {
                        if (edges[k][0] == e1 && edges[k][1] == e0) {
                            edges[k][0] = edges[edge_count - 1][0];
                            edges[k][1] = edges[edge_count - 1][1];
                            edge_count--;
                            found = 1;
                            break;
                        }
                    }
                    if (!found && edge_count < EPA_MAX_FACES * 3) {
                        edges[edge_count][0] = e0;
                        edges[edge_count][1] = e1;
                        edge_count++;
                    }
                }
                faces[i] = faces[face_count - 1];
                face_count--;
            } else {
                i++;
            }
        }

        for (int e = 0; e < edge_count && face_count < EPA_MAX_FACES; e++) {
            int ia = edges[e][0], ib = edges[e][1];
            b3_Vec3 a = verts[ia], b = verts[ib], c = verts[new_index];
            b3_Vec3 normal2 = face_normal(a, b, c);
            b3_real dist2 = b3_vec3_dot(normal2, a);
            if (dist2 < 0.0f) {
                normal2 = b3_vec3_negate(normal2);
                dist2 = -dist2;
                int tmp = ia;
                ia = ib;
                ib = tmp;
            }
            faces[face_count].idx[0] = ia;
            faces[face_count].idx[1] = ib;
            faces[face_count].idx[2] = new_index;
            faces[face_count].normal = normal2;
            faces[face_count].dist = dist2;
            face_count++;
        }
    }

    /* Iteration budget exhausted: report the best face found so far. */
    int closest = 0;
    b3_real closest_dist = faces[0].dist;
    for (int i = 1; i < face_count; i++) {
        if (faces[i].dist < closest_dist) {
            closest_dist = faces[i].dist;
            closest = i;
        }
    }
    if (out) {
        epa_contact_point(input, faces[closest].normal, closest_dist, out);
    }
    return 1;
}

static int gjk_epa_once(const b3_GjkInput *input, b3_Vec3 initial_dir, b3_ContactInfo *out) {
    b3_Vec3 dir = initial_dir;
    b3_Simplex simplex;
    simplex.p[0] = minkowski_support(input, dir);
    simplex.n = 1;
    dir = b3_vec3_negate(simplex.p[0]);

    for (int iter = 0; iter < 64; iter++) {
        if (b3_vec3_length_sq(dir) < 1e-12f) {
            dir = initial_dir;
        }

        b3_Vec3 support = minkowski_support(input, dir);
        if (b3_vec3_dot(support, dir) < 0.0f) {
            return 0; /* found a separating axis */
        }

        for (int i = (simplex.n < 4 ? simplex.n : 3); i > 0; i--) {
            simplex.p[i] = simplex.p[i - 1];
        }
        simplex.p[0] = support;
        if (simplex.n < 4) {
            simplex.n++;
        }

        int enclosed;
        switch (simplex.n) {
            case 2:
                enclosed = line_case(&simplex, &dir);
                break;
            case 3:
                enclosed = triangle_case(&simplex, &dir);
                break;
            default:
                enclosed = tetrahedron_case(&simplex, &dir);
                break;
        }
        if (enclosed) {
            return epa(input, &simplex, out);
        }
    }
    return 0; /* didn't converge: treat as (probably grazing) non-overlap */
}

/* Deliberately non-axis-aligned, non-coplanar starting directions. Two
 * shapes centered on a common coordinate axis or plane (an extremely
 * common case -- e.g. two spheres stacked along X, or either shape
 * paired with an axis-aligned box) can otherwise make an entire run's
 * support points exactly collinear/coplanar with the origin, which the
 * degenerate-cross-product fallbacks in set_line/triangle_case/
 * face_normal recover from individually but not always perfectly --
 * hand-rolled GJK/EPA is known to be sensitive to exactly this kind of
 * symmetric input. Retrying with a different seed direction whenever the
 * result comes back with a degenerate (near-zero) normal is the standard
 * pragmatic fix. */
static const b3_Vec3 seed_directions[4] = {
    {0.57735027f, 0.57735027f, 0.57735027f},
    {0.57735027f, -0.57735027f, 0.57735027f},
    {-0.57735027f, 0.57735027f, -0.57735027f},
    {0.57735027f, 0.57735027f, -0.57735027f},
};

int b3_gjk_epa_overlap(const b3_GjkInput *input, b3_ContactInfo *out) {
    /* Run every seed and keep the LARGEST-penetration non-degenerate
     * result. EPA's polytope is built only from real Minkowski-difference
     * support points, so every face it ever reports is guaranteed to lie
     * at or inside the true boundary -- each refinement iteration can
     * only push the closest-face distance up towards the true minimum
     * penetration, never past it. An under-converged run (e.g. one that
     * got stuck on a highly symmetric shape like a cube, where several
     * faces start out near-equidistant from the origin) therefore always
     * *under*-reports penetration relative to a better-converged run; a
     * single run has no way to tell it settled early, but comparing
     * seeds does, and the largest value is the closest to correct. */
    b3_ContactInfo best;
    int found = 0;
    b3_ContactInfo fallback;
    int have_fallback = 0;
    for (int seed = 0; seed < 4; seed++) {
        b3_ContactInfo candidate;
        if (!gjk_epa_once(input, seed_directions[seed], &candidate)) {
            continue;
        }
        if (b3_vec3_length_sq(candidate.normal) < 0.5f) {
            if (!have_fallback) {
                fallback = candidate;
                have_fallback = 1;
            }
            continue; /* degenerate normal: never prefer this seed's result */
        }
        if (!found || candidate.penetration > best.penetration) {
            best = candidate;
            found = 1;
        }
    }
    if (found) {
        if (out) *out = best;
        return 1;
    }
    if (have_fallback) {
        /* Every overlapping seed was degenerate: still report overlap
         * (rather than a false negative) with whatever contact info the
         * first such seed produced. */
        if (out) *out = fallback;
        return 1;
    }
    return 0;
}
