#pragma once

#include <cmath>
#include <ostream>

struct Vec3f {
    Vec3f() { x = y = z = 0.f; }
    explicit Vec3f(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {};
    explicit Vec3f(float f) { x= y = z = f; }

    Vec3f(const Vec3f &) = default;
    Vec3f &operator=(const Vec3f &) = default;
    ~Vec3f() = default;

    float &operator[](int idx) {
        return data[idx];
    }

    float operator[](int idx) const {
        return data[idx];
    }

    Vec3f operator+=(const Vec3f &v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vec3f operator-=(const Vec3f &v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    Vec3f operator*=(const Vec3f &v) { x *= v.x; y *= v.y; z *= v.z; return *this; }
    Vec3f operator/=(const Vec3f &v) { x /= v.x; y /= v.y; z /= v.z; return *this; }

    Vec3f operator+=(const float &s) { x += s; y += s; z += s; return *this; }
    Vec3f operator-=(const float &s) { x -= s; y -= s; z -= s; return *this; }
    Vec3f operator*=(const float &s) { x *= s; y *= s; z *= s; return *this; }
    Vec3f operator/=(const float &s) { x /= s; y /= s; z /= s; return *this; }

    Vec3f operator+(Vec3f &v) const { return Vec3f(x + v.x, y + v.y, z + v.z); }
    Vec3f operator-(Vec3f &v) const { return Vec3f(x - v.x, y - v.y, z - v.z); }
    Vec3f operator*(Vec3f &v) const { return Vec3f(x * v.x, y * v.y, z * v.z); }
    Vec3f operator/(Vec3f &v) const { return Vec3f(x / v.x, y / v.y, z / v.z); }

    Vec3f operator+(const float &s) const { return Vec3f(x + s, y + s, z + s); }
    Vec3f operator-(const float &s) const { return Vec3f(x - s, y - s, z - s); }
    Vec3f operator*(const float &s) const { return Vec3f(x * s, y * s, z * s); }
    Vec3f operator/(const float &s) const { return Vec3f(x / s, y / s, z / s); }

    bool operator==(Vec3f v) const { return x == v.x && y == v.y && z == v.z; }
    bool operator!=(Vec3f v) const { return x != v.x || y != v.y || z != v.z; }

    Vec3f abs() const { return Vec3f(std::abs(x), std::abs(y), std::abs(z)); }
    Vec3f operator-() const { return Vec3f(-x, -y, -z); }
    bool valid() const {
        return !(std::isinf(x) || std::isinf(y) || std::isinf(z) || std::isnan(x) ||
                 std::isnan(y) || std::isnan(z));
    }

    float norm_squared() const { return x * x + y * y + z * z; }
    float norm() const { return std::sqrt(norm_squared()); }
    Vec3f unit() const {
        float n = norm();
        return Vec3f(x / n, y / n, z / n);
    }

    Vec3f normalize() {
        float n = norm();
        x /= n;
        y /= n;
        z /= n;
        return *this;
    }

    Vec3f range(float min, float max) const {
        if (!valid())
            return Vec3f();
        Vec3f r = *this;
        float range = max - min;
        while (r.x < min)
            r.x += range;
        while (r.x >= max)
            r.x -= range;
        while (r.y < min)
            r.y += range;
        while (r.y >= max)
            r.y -= range;
        while (r.z < min)
            r.z += range;
        while (r.z >= max)
            r.z -= range;
        return r;
    }

    // Real data
    union {
        struct {
            float x;
            float y;
            float z;
        };
        float data[3] = {};
    };
};

inline Vec3f operator+(float s, Vec3f v) { return Vec3f(v.x + s, v.y + s, v.z + s); }
inline Vec3f operator-(float s, Vec3f v) { return Vec3f(v.x - s, v.y - s, v.z - s); }
inline Vec3f operator*(float s, Vec3f v) { return Vec3f(v.x * s, v.y * s, v.z * s); }
inline Vec3f operator/(float s, Vec3f v) { return Vec3f(s / v.x, s / v.y, s / v.z); }

// Take minimum of each component
inline Vec3f hmin(Vec3f l, Vec3f r) {
    return Vec3f(std::min(l.x, r.x), std::min(l.y, r.y), std::min(l.z, r.z));
}

// Take maximum of each component
inline Vec3f hmax(Vec3f l, Vec3f r) {
    return Vec3f(std::max(l.x, r.x), std::max(l.y, r.y), std::max(l.z, r.z));
}

// 3D dot product
inline float dot(Vec3f l, Vec3f r) { return l.x * r.x + l.y * r.y + l.z * r.z; }

// 3D cross product
inline Vec3f cross(Vec3f l, Vec3f r) {
    return Vec3f(l.y * r.z - l.z * r.y, l.z * r.x - l.x * r.z, l.x * r.y - l.y * r.x);
}

inline std::ostream &operator<<(std::ostream &os, Vec3f v) {
    os << "{" << v.x << "," << v.y << "," << v.z << "}";
    return os;
}