
#pragma once

#include "vec3f.hpp"

struct Vec4f {

    Vec4f() {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        w = 0.0f;
    }
    explicit Vec4f(float _x, float _y, float _z, float _w) {
        x = _x;
        y = _y;
        z = _z;
        w = _w;
    }
    explicit Vec4f(float f) { x = y = z = w = f; }
    explicit Vec4f(int _x, int _y, int _z, int _w) {
        x = (float)_x;
        y = (float)_y;
        z = (float)_z;
        w = (float)_w;
    }
    explicit Vec4f(Vec3f xyz, float _w) {
        x = xyz.x;
        y = xyz.y;
        z = xyz.z;
        w = _w;
    }

    Vec4f(const Vec4f &) = default;
    Vec4f &operator=(const Vec4f &) = default;
    ~Vec4f() = default;

    float &operator[](int idx) {
        return data[idx];
    }
    float operator[](int idx) const {
        return data[idx];
    }

    Vec4f operator+=(Vec4f v) {
        x += v.x;
        y += v.y;
        z += v.z;
        w += v.w;
        return *this;
    }
    Vec4f operator-=(Vec4f v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        w -= v.w;
        return *this;
    }
    Vec4f operator*=(Vec4f v) {
        x *= v.x;
        y *= v.y;
        z *= v.z;
        w *= v.w;
        return *this;
    }
    Vec4f operator/=(Vec4f v) {
        x /= v.x;
        y /= v.y;
        z /= v.z;
        w /= v.w;
        return *this;
    }

    Vec4f operator+=(float s) {
        x += s;
        y += s;
        z += s;
        w += s;
        return *this;
    }
    Vec4f operator-=(float s) {
        x -= s;
        y -= s;
        z -= s;
        w -= s;
        return *this;
    }
    Vec4f operator*=(float s) {
        x *= s;
        y *= s;
        z *= s;
        w *= s;
        return *this;
    }
    Vec4f operator/=(float s) {
        x /= s;
        y /= s;
        z /= s;
        w /= s;
        return *this;
    }

    Vec4f operator+(Vec4f v) const { return Vec4f(x + v.x, y + v.y, z + v.z, w + v.w); }
    Vec4f operator-(Vec4f v) const { return Vec4f(x - v.x, y - v.y, z - v.z, w - v.w); }
    Vec4f operator*(Vec4f v) const { return Vec4f(x * v.x, y * v.y, z * v.z, w * v.w); }
    Vec4f operator/(Vec4f v) const { return Vec4f(x / v.x, y / v.y, z / v.z, w / v.w); }

    Vec4f operator+(float s) const { return Vec4f(x + s, y + s, z + s, w + s); }
    Vec4f operator-(float s) const { return Vec4f(x - s, y - s, z - s, w - s); }
    Vec4f operator*(float s) const { return Vec4f(x * s, y * s, z * s, w * s); }
    Vec4f operator/(float s) const { return Vec4f(x / s, y / s, z / s, w / s); }

    bool operator==(Vec4f v) const { return x == v.x && y == v.y && z == v.z && w == v.w; }
    bool operator!=(Vec4f v) const { return x != v.x || y != v.y || z != v.z || w != v.w; }

    /// Absolute value
    Vec4f abs() const { return Vec4f(std::abs(x), std::abs(y), std::abs(z), std::abs(w)); }
    /// Negation
    Vec4f operator-() const { return Vec4f(-x, -y, -z, -w); }
    /// Are all members real numbers?
    bool valid() const {
        return !(std::isinf(x) || std::isinf(y) || std::isinf(z) || std::isinf(w) ||
                 std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(w));
    }

    /// Modify vec to have unit length
    Vec4f normalize() {
        float n = norm();
        x /= n;
        y /= n;
        z /= n;
        w /= n;
        return *this;
    }
    /// Return unit length vec in the same direction
    Vec4f unit() const {
        float n = norm();
        return Vec4f(x / n, y / n, z / n, w / n);
    }

    float norm_squared() const { return x * x + y * y + z * z + w * w; }
    float norm() const { return std::sqrt(norm_squared()); }

    /// Returns first three components
    Vec3f xyz() const { return Vec3f(x, y, z); }
    /// Performs perspective division (xyz/w)
    Vec3f project() const { return Vec3f(x / w, y / w, z / w); }

    union {
        struct {
            float x;
            float y;
            float z;
            float w;
        };
        float data[4] = {};
    };
};

inline Vec4f operator+(float s, Vec4f v) { return Vec4f(v.x + s, v.y + s, v.z + s, v.w + s); }
inline Vec4f operator-(float s, Vec4f v) { return Vec4f(v.x - s, v.y - s, v.z - s, v.w - s); }
inline Vec4f operator*(float s, Vec4f v) { return Vec4f(v.x * s, v.y * s, v.z * s, v.w * s); }
inline Vec4f operator/(float s, Vec4f v) { return Vec4f(s / v.x, s / v.y, s / v.z, s / v.w); }

/// Take minimum of each component
inline Vec4f hmin(Vec4f l, Vec4f r) {
    return Vec4f(std::min(l.x, r.x), std::min(l.y, r.y), std::min(l.z, r.z), std::min(l.w, r.w));
}
/// Take maximum of each component
inline Vec4f hmax(Vec4f l, Vec4f r) {
    return Vec4f(std::max(l.x, r.x), std::max(l.y, r.y), std::max(l.z, r.z), std::max(l.w, r.w));
}

/// 4D dot product
inline float dot(Vec4f l, Vec4f r) { return l.x * r.x + l.y * r.y + l.z * r.z + l.w * r.w; }

inline std::ostream &operator<<(std::ostream &out, Vec4f v) {
    out << "{" << v.x << "," << v.y << "," << v.z << "," << v.w << "}";
    return out;
}
