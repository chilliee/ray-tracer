#include "ray_tracer.hpp"

bool mesh_intersect(Ray& r, const Vec3f &p0, const Vec3f &p1, const Vec3f &p2) {
    Vec3f e1 = p1 - p0;
    Vec3f e2 = p2 - p0;

    Vec3f e1_x_d = cross(e1, r.d);
    float det = dot(e1_x_d, e2);
    if (det < EPS_F && det > -EPS_F)
        return false;

    float inv_det = 1.0 / det;
    Vec3f s = r.o - p0;
    float v = dot(e1_x_d, s) * inv_det;
    if (v < 0. || v > 1.)
        return false;

    Vec3f s_x_e2 = cross(s, e2);
    float u = -dot(s_x_e2, r.d) * inv_det;
    if (u < 0. || u + v > 1.)
        return false;

    float t = -dot(s_x_e2, e1) * inv_det;
    if (t < r.min_t || t > r.max_t)
        return false;

    if (r.max_t > t)
        r.max_t = t;
    return true;
}
