#include "mathlib.hpp"

struct Ray {
    size_t depth;
    Vec3f o;
    Vec3f d;

    float min_t;
    float max_t;

    Vec3f inv_d;
    int sign[3];

    Ray(const Vec3f &o, const Vec3f &d, int depth = 0)
        : o(o), d(d), min_t(0.0), max_t(INF_F), depth(depth) {
        inv_d = Vec3f(1 / d.x, 1 / d.y, 1 / d.z);
        sign[0] = (inv_d.x < 0);
        sign[1] = (inv_d.y < 0);
        sign[2] = (inv_d.z < 0);
    }

    Ray(const Vec3f &o, const Vec3f &d, double max_t, int depth = 0)
        : o(o), d(d), min_t(0.0), max_t(max_t), depth(depth) {
        inv_d = Vec3f(1 / d.x, 1 / d.y, 1 / d.z);
        sign[0] = (inv_d.x < 0);
        sign[1] = (inv_d.y < 0);
        sign[2] = (inv_d.z < 0);
    }

    inline Vec3f at_time(float t) const { return o + t * d; }
};