
#pragma once

#include <cfloat>
#include "vec4f.hpp"


#define PI_F 3.14159265358979323846264338327950288f
#define Degrees(v) ((v) * (180.0f / PI_F))
#define Radians(v) ((v) * (PI_F / 180.0f))

struct Mat4f {

    /// Identity matrix
    static const Mat4f I;
    /// Zero matrix
    static const Mat4f Zero;

    /// Return transposed matrix
    static Mat4f transpose(const Mat4f &m);
    /// Return inverse matrix (will be NaN if m is not invertible)
    static Mat4f inverse(const Mat4f &m);
    /// Return transformation matrix for given translation vector
    static Mat4f translate(Vec3f t);
    /// Return transformation matrix for given angle (degrees) and axis
    static Mat4f rotate(float t, Vec3f axis);
    /// Return transformation matrix for given XYZ Euler angle rotation
    static Mat4f euler(Vec3f angles);
    /// Return transformation matrix that rotates the Y axis to dir
    static Mat4f rotate_to(Vec3f dir);
    /// Return transformation matrix that rotates the -Z axis to dir
    static Mat4f rotate_z_to(Vec3f dir);
    /// Return transformation matrix for given scale factors
    static Mat4f scale(Vec3f s);
    /// Return transformation matrix with given axes
    static Mat4f axes(Vec3f x, Vec3f y, Vec3f z);

    /// Return transformation matrix for viewing a scene from $pos looking at $at,
    /// where straight up is defined as $up
    static Mat4f look_at(Vec3f pos, Vec3f at, Vec3f up = Vec3f{0.0f, 1.0f, 0.0f});
    /// Return orthogonal projection matrix with given left, right, bottom, top,
    /// near, and far planes.
    static Mat4f ortho(float l, float r, float b, float t, float n, float f);

    /// Return perspective projection matrix with given field of view, aspect ratio,
    /// and near plane. The far plane is assumed to be at infinity. This projection
    /// also outputs n/z for better precision with floating point depth buffers, so we use
    /// a depth mapping where 0 is the far plane (infinity) and 1 is the near plane, and
    /// an object is closer if is depth is greater.
    static Mat4f project(float fov, float ar, float n);

    Mat4f() { *this = I; }
    explicit Mat4f(Vec4f x, Vec4f y, Vec4f z, Vec4f w) {
        cols[0] = x;
        cols[1] = y;
        cols[2] = z;
        cols[3] = w;
    }

    Mat4f(const Mat4f &) = default;
    Mat4f &operator=(const Mat4f &) = default;
    ~Mat4f() = default;

    Vec4f &operator[](int idx) {
        return cols[idx];
    }
    Vec4f operator[](int idx) const {
        return cols[idx];
    }

    Mat4f operator+=(const Mat4f &m) {
        for (int i = 0; i < 4; i++)
            cols[i] += m.cols[i];
        return *this;
    }
    Mat4f operator-=(const Mat4f &m) {
        for (int i = 0; i < 4; i++)
            cols[i] -= m.cols[i];
        return *this;
    }

    Mat4f operator+=(float s) {
        for (int i = 0; i < 4; i++)
            cols[i] += s;
        return *this;
    }
    Mat4f operator-=(float s) {
        for (int i = 0; i < 4; i++)
            cols[i] -= s;
        return *this;
    }
    Mat4f operator*=(float s) {
        for (int i = 0; i < 4; i++)
            cols[i] *= s;
        return *this;
    }
    Mat4f operator/=(float s) {
        for (int i = 0; i < 4; i++)
            cols[i] /= s;
        return *this;
    }

    Mat4f operator+(const Mat4f &m) const {
        Mat4f r;
        for (int i = 0; i < 4; i++)
            r.cols[i] = cols[i] + m.cols[i];
        return r;
    }
    Mat4f operator-(const Mat4f &m) const {
        Mat4f r;
        for (int i = 0; i < 4; i++)
            r.cols[i] = cols[i] - m.cols[i];
        return r;
    }

    Mat4f operator+(float s) const {
        Mat4f r;
        for (int i = 0; i < 4; i++)
            r.cols[i] = cols[i] + s;
        return r;
    }
    Mat4f operator-(float s) const {
        Mat4f r;
        for (int i = 0; i < 4; i++)
            r.cols[i] = cols[i] - s;
        return r;
    }
    Mat4f operator*(float s) const {
        Mat4f r;
        for (int i = 0; i < 4; i++)
            r.cols[i] = cols[i] * s;
        return r;
    }
    Mat4f operator/(float s) const {
        Mat4f r;
        for (int i = 0; i < 4; i++)
            r.cols[i] = cols[i] / s;
        return r;
    }

    Mat4f operator*=(const Mat4f &v) {
        *this = *this * v;
        return *this;
    }
    Mat4f operator*(const Mat4f &m) const {
        Mat4f ret;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                ret[i][j] = 0.0f;
                for (int k = 0; k < 4; k++) {
                    ret[i][j] += m[i][k] * cols[k][j];
                }
            }
        }
        return ret;
    }

    Vec4f operator*(Vec4f v) const {
        return v[0] * cols[0] + v[1] * cols[1] + v[2] * cols[2] + v[3] * cols[3];
    }

    /// Expands v to Vec4f(v, 1.0), multiplies, and projects back to 3D
    Vec3f operator*(Vec3f v) const { return operator*(Vec4f(v, 1.0f)).project(); }
    /// Expands v to Vec4f(v, 0.0), multiplies, and projects back to 3D
    Vec3f rotate(Vec3f v) const { return operator*(Vec4f(v, 0.0f)).xyz(); }

    /// Converts rotation (orthonormal 3x3) matrix to equivalent Euler angles
    Vec3f to_euler() const {

        bool single = true;
        static const float singularity[] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f,
                                            0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0};
        for (int i = 0; i < 12 && single; i++) {
            single = single && std::abs(data[i] - singularity[i]) < 16.0f * FLT_EPSILON;
        }
        if (single)
            return Vec3f{0.0f, 0.0f, 180.0f};

        Vec3f eul1, eul2;

        float cy = std::hypotf(cols[0][0], cols[0][1]);
        if (cy > 16.0f * FLT_EPSILON) {
            eul1[0] = std::atan2(cols[1][2], cols[2][2]);
            eul1[1] = std::atan2(-cols[0][2], cy);
            eul1[2] = std::atan2(cols[0][1], cols[0][0]);

            eul2[0] = std::atan2(-cols[1][2], -cols[2][2]);
            eul2[1] = std::atan2(-cols[0][2], -cy);
            eul2[2] = std::atan2(-cols[0][1], -cols[0][0]);
        } else {
            eul1[0] = std::atan2(-cols[2][1], cols[1][1]);
            eul1[1] = std::atan2(-cols[0][2], cy);
            eul1[2] = 0;
            eul2 = eul1;
        }
        float d1 = std::abs(eul1[0]) + std::abs(eul1[1]) + std::abs(eul1[2]);
        float d2 = std::abs(eul2[0]) + std::abs(eul2[1]) + std::abs(eul2[2]);
        if (d1 > d2)
            return Degrees(eul2);
        else
            return Degrees(eul1);
    }

    /// Returns matrix transpose
    Mat4f T() const { return transpose(*this); }
    /// Returns matrix inverse (will be NaN if m is not invertible)
    Mat4f inverse() const { return inverse(*this); }

    /// Returns determinant (brute force).
    float det() const {
        return cols[0][3] * cols[1][2] * cols[2][1] * cols[3][0] -
               cols[0][2] * cols[1][3] * cols[2][1] * cols[3][0] -
               cols[0][3] * cols[1][1] * cols[2][2] * cols[3][0] +
               cols[0][1] * cols[1][3] * cols[2][2] * cols[3][0] +
               cols[0][2] * cols[1][1] * cols[2][3] * cols[3][0] -
               cols[0][1] * cols[1][2] * cols[2][3] * cols[3][0] -
               cols[0][3] * cols[1][2] * cols[2][0] * cols[3][1] +
               cols[0][2] * cols[1][3] * cols[2][0] * cols[3][1] +
               cols[0][3] * cols[1][0] * cols[2][2] * cols[3][1] -
               cols[0][0] * cols[1][3] * cols[2][2] * cols[3][1] -
               cols[0][2] * cols[1][0] * cols[2][3] * cols[3][1] +
               cols[0][0] * cols[1][2] * cols[2][3] * cols[3][1] +
               cols[0][3] * cols[1][1] * cols[2][0] * cols[3][2] -
               cols[0][1] * cols[1][3] * cols[2][0] * cols[3][2] -
               cols[0][3] * cols[1][0] * cols[2][1] * cols[3][2] +
               cols[0][0] * cols[1][3] * cols[2][1] * cols[3][2] +
               cols[0][1] * cols[1][0] * cols[2][3] * cols[3][2] -
               cols[0][0] * cols[1][1] * cols[2][3] * cols[3][2] -
               cols[0][2] * cols[1][1] * cols[2][0] * cols[3][3] +
               cols[0][1] * cols[1][2] * cols[2][0] * cols[3][3] +
               cols[0][2] * cols[1][0] * cols[2][1] * cols[3][3] -
               cols[0][0] * cols[1][2] * cols[2][1] * cols[3][3] -
               cols[0][1] * cols[1][0] * cols[2][2] * cols[3][3] +
               cols[0][0] * cols[1][1] * cols[2][2] * cols[3][3];
    }

    union {
        Vec4f cols[4];
        float data[16] = {};
    };
};

inline bool operator==(const Mat4f &l, const Mat4f &r) {
    for (int i = 0; i < 16; i++)
        if (l.data[i] != r.data[i])
            return false;
    return true;
}

inline bool operator!=(const Mat4f &l, const Mat4f &r) {
    for (int i = 0; i < 16; i++)
        if (l.data[i] != r.data[i])
            return true;
    return false;
}

inline Mat4f operator+(float s, const Mat4f &m) {
    Mat4f r;
    for (int i = 0; i < 4; i++)
        r.cols[i] = m.cols[i] + s;
    return r;
}
inline Mat4f operator-(float s, const Mat4f &m) {
    Mat4f r;
    for (int i = 0; i < 4; i++)
        r.cols[i] = m.cols[i] - s;
    return r;
}
inline Mat4f operator*(float s, const Mat4f &m) {
    Mat4f r;
    for (int i = 0; i < 4; i++)
        r.cols[i] = m.cols[i] * s;
    return r;
}
inline Mat4f operator/(float s, const Mat4f &m) {
    Mat4f r;
    for (int i = 0; i < 4; i++)
        r.cols[i] = m.cols[i] / s;
    return r;
}

const inline Mat4f Mat4f::I = Mat4f{Vec4f{1.0f, 0.0f, 0.0f, 0.0f}, Vec4f{0.0f, 1.0f, 0.0f, 0.0f},
                                 Vec4f{0.0f, 0.0f, 1.0f, 0.0f}, Vec4f{0.0f, 0.0f, 0.0f, 1.0f}};
const inline Mat4f Mat4f::Zero = Mat4f{Vec4f{0.0f, 0.0f, 0.0f, 0.0f}, Vec4f{0.0f, 0.0f, 0.0f, 0.0f},
                                    Vec4f{0.0f, 0.0f, 0.0f, 0.0f}, Vec4f{0.0f, 0.0f, 0.0f, 0.0f}};

inline Mat4f outer(Vec4f u, Vec4f v) {
    Mat4f B;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            B[i][j] = u[i] * v[j];
    return B;
}

inline Mat4f Mat4f::transpose(const Mat4f &m) {
    Mat4f r;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            r[i][j] = m[j][i];
        }
    }
    return r;
}

inline Mat4f Mat4f::inverse(const Mat4f &m) {
    Mat4f r;
    r[0][0] = m[1][2] * m[2][3] * m[3][1] - m[1][3] * m[2][2] * m[3][1] +
              m[1][3] * m[2][1] * m[3][2] - m[1][1] * m[2][3] * m[3][2] -
              m[1][2] * m[2][1] * m[3][3] + m[1][1] * m[2][2] * m[3][3];
    r[0][1] = m[0][3] * m[2][2] * m[3][1] - m[0][2] * m[2][3] * m[3][1] -
              m[0][3] * m[2][1] * m[3][2] + m[0][1] * m[2][3] * m[3][2] +
              m[0][2] * m[2][1] * m[3][3] - m[0][1] * m[2][2] * m[3][3];
    r[0][2] = m[0][2] * m[1][3] * m[3][1] - m[0][3] * m[1][2] * m[3][1] +
              m[0][3] * m[1][1] * m[3][2] - m[0][1] * m[1][3] * m[3][2] -
              m[0][2] * m[1][1] * m[3][3] + m[0][1] * m[1][2] * m[3][3];
    r[0][3] = m[0][3] * m[1][2] * m[2][1] - m[0][2] * m[1][3] * m[2][1] -
              m[0][3] * m[1][1] * m[2][2] + m[0][1] * m[1][3] * m[2][2] +
              m[0][2] * m[1][1] * m[2][3] - m[0][1] * m[1][2] * m[2][3];
    r[1][0] = m[1][3] * m[2][2] * m[3][0] - m[1][2] * m[2][3] * m[3][0] -
              m[1][3] * m[2][0] * m[3][2] + m[1][0] * m[2][3] * m[3][2] +
              m[1][2] * m[2][0] * m[3][3] - m[1][0] * m[2][2] * m[3][3];
    r[1][1] = m[0][2] * m[2][3] * m[3][0] - m[0][3] * m[2][2] * m[3][0] +
              m[0][3] * m[2][0] * m[3][2] - m[0][0] * m[2][3] * m[3][2] -
              m[0][2] * m[2][0] * m[3][3] + m[0][0] * m[2][2] * m[3][3];
    r[1][2] = m[0][3] * m[1][2] * m[3][0] - m[0][2] * m[1][3] * m[3][0] -
              m[0][3] * m[1][0] * m[3][2] + m[0][0] * m[1][3] * m[3][2] +
              m[0][2] * m[1][0] * m[3][3] - m[0][0] * m[1][2] * m[3][3];
    r[1][3] = m[0][2] * m[1][3] * m[2][0] - m[0][3] * m[1][2] * m[2][0] +
              m[0][3] * m[1][0] * m[2][2] - m[0][0] * m[1][3] * m[2][2] -
              m[0][2] * m[1][0] * m[2][3] + m[0][0] * m[1][2] * m[2][3];
    r[2][0] = m[1][1] * m[2][3] * m[3][0] - m[1][3] * m[2][1] * m[3][0] +
              m[1][3] * m[2][0] * m[3][1] - m[1][0] * m[2][3] * m[3][1] -
              m[1][1] * m[2][0] * m[3][3] + m[1][0] * m[2][1] * m[3][3];
    r[2][1] = m[0][3] * m[2][1] * m[3][0] - m[0][1] * m[2][3] * m[3][0] -
              m[0][3] * m[2][0] * m[3][1] + m[0][0] * m[2][3] * m[3][1] +
              m[0][1] * m[2][0] * m[3][3] - m[0][0] * m[2][1] * m[3][3];
    r[2][2] = m[0][1] * m[1][3] * m[3][0] - m[0][3] * m[1][1] * m[3][0] +
              m[0][3] * m[1][0] * m[3][1] - m[0][0] * m[1][3] * m[3][1] -
              m[0][1] * m[1][0] * m[3][3] + m[0][0] * m[1][1] * m[3][3];
    r[2][3] = m[0][3] * m[1][1] * m[2][0] - m[0][1] * m[1][3] * m[2][0] -
              m[0][3] * m[1][0] * m[2][1] + m[0][0] * m[1][3] * m[2][1] +
              m[0][1] * m[1][0] * m[2][3] - m[0][0] * m[1][1] * m[2][3];
    r[3][0] = m[1][2] * m[2][1] * m[3][0] - m[1][1] * m[2][2] * m[3][0] -
              m[1][2] * m[2][0] * m[3][1] + m[1][0] * m[2][2] * m[3][1] +
              m[1][1] * m[2][0] * m[3][2] - m[1][0] * m[2][1] * m[3][2];
    r[3][1] = m[0][1] * m[2][2] * m[3][0] - m[0][2] * m[2][1] * m[3][0] +
              m[0][2] * m[2][0] * m[3][1] - m[0][0] * m[2][2] * m[3][1] -
              m[0][1] * m[2][0] * m[3][2] + m[0][0] * m[2][1] * m[3][2];
    r[3][2] = m[0][2] * m[1][1] * m[3][0] - m[0][1] * m[1][2] * m[3][0] -
              m[0][2] * m[1][0] * m[3][1] + m[0][0] * m[1][2] * m[3][1] +
              m[0][1] * m[1][0] * m[3][2] - m[0][0] * m[1][1] * m[3][2];
    r[3][3] = m[0][1] * m[1][2] * m[2][0] - m[0][2] * m[1][1] * m[2][0] +
              m[0][2] * m[1][0] * m[2][1] - m[0][0] * m[1][2] * m[2][1] -
              m[0][1] * m[1][0] * m[2][2] + m[0][0] * m[1][1] * m[2][2];
    r /= m.det();
    return r;
}

inline Mat4f Mat4f::rotate_to(Vec3f dir) {

    dir.normalize();

    if (dir.y == 1.0f)
        return Mat4f::I;
    else if (dir.y == -1.0f)
        return Mat4f{Vec4f{1.0f, 0.0f, 0.0f, 0.0f}, Vec4f{0.0f, -1.0f, 0.0f, 0.0f},
                    Vec4f{0.0f, 0.0f, 1.0f, 0.0}, Vec4f{0.0f, 0.0f, 0.0f, 1.0f}};
    else {
        Vec3f x = cross(dir, Vec3f{0.0f, 1.0f, 0.0f}).unit();
        Vec3f z = cross(x, dir).unit();
        return Mat4f{Vec4f{x, 0.0f}, Vec4f{dir, 0.0f}, Vec4f{z, 0.0f}, Vec4f{0.0f, 0.0f, 0.0f, 1.0f}};
    }
}

inline Mat4f Mat4f::rotate_z_to(Vec3f dir) {
    Mat4f y = rotate_to(dir);
    Vec4f _y = y[1];
    Vec4f _z = y[2];
    y[1] = _z;
    y[2] = -_y;
    return y;
}

inline Mat4f Mat4f::axes(Vec3f x, Vec3f y, Vec3f z) {
    return Mat4f{Vec4f{x, 0.0f}, Vec4f{y, 0.0f}, Vec4f{z, 0.0f}, Vec4f{0.0f, 0.0f, 0.0f, 1.0f}};
}

inline Mat4f Mat4f::translate(Vec3f t) {
    Mat4f r;
    r[3] = Vec4f(t, 1.0f);
    return r;
}

inline Mat4f Mat4f::euler(Vec3f angles) {
    return Mat4f::rotate(angles.z, Vec3f{0.0f, 0.0f, 1.0f}) *
           Mat4f::rotate(angles.y, Vec3f{0.0f, 1.0f, 0.0f}) *
           Mat4f::rotate(angles.x, Vec3f{1.0f, 0.0f, 0.0f});
}

inline Mat4f Mat4f::rotate(float t, Vec3f axis) {
    Mat4f ret;
    float c = std::cos(Radians(t));
    float s = std::sin(Radians(t));
    axis.normalize();
    Vec3f temp = axis * (1.0f - c);
    ret[0][0] = c + temp[0] * axis[0];
    ret[0][1] = temp[0] * axis[1] + s * axis[2];
    ret[0][2] = temp[0] * axis[2] - s * axis[1];
    ret[1][0] = temp[1] * axis[0] - s * axis[2];
    ret[1][1] = c + temp[1] * axis[1];
    ret[1][2] = temp[1] * axis[2] + s * axis[0];
    ret[2][0] = temp[2] * axis[0] + s * axis[1];
    ret[2][1] = temp[2] * axis[1] - s * axis[0];
    ret[2][2] = c + temp[2] * axis[2];
    return ret;
}

inline Mat4f Mat4f::scale(Vec3f s) {
    Mat4f r;
    r[0][0] = s.x;
    r[1][1] = s.y;
    r[2][2] = s.z;
    return r;
}

inline Mat4f Mat4f::ortho(float l, float r, float b, float t, float n, float f) {
    Mat4f rs;
    rs[0][0] = 2.0f / (r - l);
    rs[1][1] = 2.0f / (t - b);
    rs[2][2] = 2.0f / (n - f);
    rs[3][0] = (-l - r) / (r - l);
    rs[3][1] = (-b - t) / (t - b);
    rs[3][2] = -n / (f - n);
    return rs;
}

inline Mat4f Mat4f::project(float fov, float ar, float n) {
    float f = 1.0f / std::tan(Radians(fov) / 2.0f);
    Mat4f r;
    r[0][0] = f / ar;
    r[1][1] = f;
    r[2][2] = 0.0f;
    r[3][3] = 0.0f;
    r[3][2] = n;
    r[2][3] = -1.0f;
    return r;
}

inline Mat4f Mat4f::look_at(Vec3f pos, Vec3f at, Vec3f up) {
    Mat4f r = Mat4f::Zero;
    Vec3f F = (at - pos).unit();
    Vec3f S = cross(F, up).unit();
    Vec3f U = cross(S, F).unit();
    r[0][0] = S.x;
    r[0][1] = U.x;
    r[0][2] = -F.x;
    r[1][0] = S.y;
    r[1][1] = U.y;
    r[1][2] = -F.y;
    r[2][0] = S.z;
    r[2][1] = U.z;
    r[2][2] = -F.z;
    r[3][0] = -dot(S, pos);
    r[3][1] = -dot(U, pos);
    r[3][2] = dot(F, pos);
    r[3][3] = 1.0f;
    return r;
}

inline std::ostream &operator<<(std::ostream &out, Mat4f m) {
    out << "{" << m[0] << "," << m[1] << "," << m[2] << "," << m[3] << "}";
    return out;
}
