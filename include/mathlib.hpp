#pragma once

#include "mat4f.hpp"

#include <limits>

#define EPS_F 0.00001f
#define INF_F (std::numeric_limits<float>::infinity())

template <typename T> inline T clamp(T x, T min, T max) { return std::min(std::max(x, min), max); }

template <> inline Vec3f clamp(Vec3f v, Vec3f min, Vec3f max) {
    return Vec3f(clamp(v.x, min.x, max.x), clamp(v.y, min.y, max.y), clamp(v.z, min.z, max.z));
}

template <typename T> T lerp(T start, T end, float t) { return start + (end - start) * t; }

inline float sign(float x) { return x > 0.0f ? 1.0f : x < 0.0f ? -1.0f : 0.0f; }

inline float frac(float x) { return x - (long long)x; }

inline float smoothstep(float e0, float e1, float x) {
    float t = clamp((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}