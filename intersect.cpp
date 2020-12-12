#include "ray_tracer.hpp"

bool intersect8(Ray& r, const Vec3f &p00, const Vec3f &p01, const Vec3f &p02,
const Vec3f &p10, const Vec3f &p11, const Vec3f &p12, const Vec3f &p20,
const Vec3f &p21, const Vec3f &p22, const Vec3f &p30, const Vec3f &p31,
const Vec3f &p32, const Vec3f &p40, const Vec3f &p41, const Vec3f &p42,
const Vec3f &p50, const Vec3f &p51, const Vec3f &p52, const Vec3f &p60,
const Vec3f &p61, const Vec3f &p62, const Vec3f &p70, const Vec3f &p71,
const Vec3f &p72) {
    Vec3f e11 = p01 - p00;
    Vec3f e12 = p02 - p00;
    Vec3f e21 = p11 - p10;
    Vec3f e22 = p12 - p10;
    Vec3f e31 = p21 - p20;
    Vec3f e32 = p22 - p20;
    Vec3f e41 = p31 - p30;
    Vec3f e42 = p32 - p30;
    Vec3f e51 = p41 - p40;
    Vec3f e52 = p42 - p40;
    Vec3f e61 = p51 - p50;
    Vec3f e62 = p52 - p50;
    Vec3f e71 = p61 - p60;
    Vec3f e72 = p62 - p60;
    Vec3f e81 = p71 - p70;
    Vec3f e82 = p72 - p70;
    bool b1 = true, b2 = true, b3 = true, b4 = true;
    bool b5 = true, b6 = true, b7 = true, b8 = true;
    __m256 temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8, temp9;
    float *res0, *res1, *res2;

    // vertex 1
    Vec3f e11_x_d = cross(e11, r.d);
    Vec3f e21_x_d = cross(e21, r.d);
    Vec3f e31_x_d = cross(e31, r.d);
    Vec3f e41_x_d = cross(e41, r.d);
    Vec3f e51_x_d = cross(e51, r.d);
    Vec3f e61_x_d = cross(e61, r.d);
    Vec3f e71_x_d = cross(e71, r.d);
    Vec3f e81_x_d = cross(e81, r.d);
    // double det1 = dot(e11_x_d, e12);
    temp1 = _mm256_set_ps(e11_x_d[0], e21_x_d[0], e31_x_d[0], e41_x_d[0], e51_x_d[0], e61_x_d[0], e71_x_d[0], e81_x_d[0]);
    temp2 = _mm256_set_ps(e12[0], e22[0], e32[0], e42[0], e52[0], e62[0], e72[0], e82[0]);
    temp3 = _mm256_mul_ps(temp1, temp2);
    res0 = (float*)&temp3;
    temp4 = _mm256_set_ps(e11_x_d[1], e21_x_d[1], e31_x_d[1], e41_x_d[1], e51_x_d[1], e61_x_d[1], e71_x_d[1], e81_x_d[1]);
    temp5 = _mm256_set_ps(e12[1], e22[1], e32[1], e42[1], e52[1], e62[1], e72[1], e82[1]);
    temp6 = _mm256_mul_ps(temp4, temp5);
    res1 = (float*)&temp6;
    temp7 = _mm256_set_ps(e11_x_d[2], e21_x_d[2], e31_x_d[2], e41_x_d[2], e51_x_d[2], e61_x_d[2], e71_x_d[2], e81_x_d[2]);
    temp8 = _mm256_set_ps(e12[2], e22[2], e32[2], e42[2], e52[2], e62[2], e72[2], e82[2]);
    temp9 = _mm256_mul_ps(temp7, temp8);
    res2 = (float*)&temp9;
    double det1 = *res0 + *res1 + *res2;
    double det2 = *(res0+1) + *(res1+1) + *(res2+1);
    double det3 = *(res0+2) + *(res1+2) + *(res2+2);
    double det4 = *(res0+3) + *(res1+3) + *(res2+3);
    double det5 = *(res0+4) + *(res1+4) + *(res2+4);
    double det6 = *(res0+5) + *(res1+5) + *(res2+5);
    double det7 = *(res0+6) + *(res1+6) + *(res2+6);
    double det8 = *(res0+7) + *(res1+7) + *(res2+7);
    if (det1 < EPS_F && det1 > -EPS_F) b1 = false;

    // vertex 2
    // double det2 = dot(e21_x_d, e22);
    if (det2 < EPS_F && det2 > -EPS_F) b2 = false;

    // vertex 3
    // double det3 = dot(e31_x_d, e32);
    if (det3 < EPS_F && det3 > -EPS_F) b3 = false;

    // vertex 4
    // double det4 = dot(e41_x_d, e42);
    if (det4 < EPS_F && det4 > -EPS_F) b4 = false;
    if (det5 < EPS_F && det5 > -EPS_F) b5 = false;
    if (det6 < EPS_F && det6 > -EPS_F) b6 = false;
    if (det7 < EPS_F && det7 > -EPS_F) b7 = false;
    if (det8 < EPS_F && det8 > -EPS_F) b8 = false;

    double inv_det1 = 1.0 / det1;
    double inv_det2 = 1.0 / det2;
    double inv_det3 = 1.0 / det3;
    double inv_det4 = 1.0 / det4;
    double inv_det5 = 1.0 / det5;
    double inv_det6 = 1.0 / det6;
    double inv_det7 = 1.0 / det7;
    double inv_det8 = 1.0 / det8;
    Vec3f s1 = r.o - p00;
    Vec3f s2 = r.o - p10;
    Vec3f s3 = r.o - p20;
    Vec3f s4 = r.o - p30;
    Vec3f s5 = r.o - p40;
    Vec3f s6 = r.o - p50;
    Vec3f s7 = r.o - p60;
    Vec3f s8 = r.o - p70;
    // temp1 = _mm_set_ps(e11_x_d[0], e21_x_d[0], e31_x_d[0], e41_x_d[0]);
    temp2 = _mm256_set_ps(s1[0], s2[0], s3[0], s4[0], s5[0], s6[0], s7[0], s8[0]);
    temp3 = _mm256_mul_ps(temp1, temp2);
    res0 = (float*)&temp3;
    // temp4 = _mm_set_ps(e11_x_d[1], e21_x_d[1], e31_x_d[1], e41_x_d[1]);
    temp5 = _mm256_set_ps(s1[1], s2[1], s3[1], s4[1], s5[1], s6[1], s7[1], s8[1]);
    temp6 = _mm256_mul_ps(temp4, temp5);
    res1 = (float*)&temp6;
    // temp7 = _mm_set_ps(e11_x_d[2], e21_x_d[2], e31_x_d[2], e41_x_d[2]);
    temp8 = _mm256_set_ps(s1[2], s2[2], s3[2], s4[2], s5[2], s6[2], s7[2], s8[2]);
    temp9 = _mm256_mul_ps(temp7, temp8);
    res2 = (float*)&temp9;
    double v1 = (*res0 + *res1 + *res2) * inv_det1;
    double v2 = (*(res0+1) + *(res1+1) + *(res2+1)) * inv_det2;
    double v3 = (*(res0+2) + *(res1+2) + *(res2+2)) * inv_det3;
    double v4 = (*(res0+3) + *(res1+3) + *(res2+3)) * inv_det4;
    double v5 = (*(res0+4) + *(res1+4) + *(res2+4)) * inv_det5;
    double v6 = (*(res0+5) + *(res1+5) + *(res2+5)) * inv_det6;
    double v7 = (*(res0+6) + *(res1+6) + *(res2+6)) * inv_det7;
    double v8 = (*(res0+7) + *(res1+7) + *(res2+7)) * inv_det8;
    // double v1 = dot(e11_x_d, s1) * inv_det1;
    // double v2 = dot(e21_x_d, s2) * inv_det2;
    // double v3 = dot(e31_x_d, s3) * inv_det3;
    // double v4 = dot(e41_x_d, s4) * inv_det4;
    if (v1 < 0. || v1 > 1.) b1 = false;
    if (v2 < 0. || v2 > 1.) b2 = false;
    if (v3 < 0. || v3 > 1.) b3 = false;
    if (v4 < 0. || v4 > 1.) b4 = false;
    if (v5 < 0. || v5 > 1.) b5 = false;
    if (v6 < 0. || v6 > 1.) b6 = false;
    if (v7 < 0. || v7 > 1.) b7 = false;
    if (v8 < 0. || v8 > 1.) b8 = false;

    Vec3f s1_x_e2 = cross(s1, e12);
    Vec3f s2_x_e2 = cross(s2, e22);
    Vec3f s3_x_e2 = cross(s3, e32);
    Vec3f s4_x_e2 = cross(s4, e42);
    Vec3f s5_x_e2 = cross(s5, e52);
    Vec3f s6_x_e2 = cross(s6, e62);
    Vec3f s7_x_e2 = cross(s7, e72);
    Vec3f s8_x_e2 = cross(s8, e82);
    temp1 = _mm256_set_ps(s1_x_e2[0], s2_x_e2[0], s3_x_e2[0], s4_x_e2[0], s5_x_e2[0], s6_x_e2[0], s7_x_e2[0], s8_x_e2[0]);
    temp2 = _mm256_broadcast_ss(&(r.d[0]));
    temp3 = _mm256_mul_ps(temp1, temp2);
    res0 = (float*)&temp3;
    temp4 = _mm256_set_ps(s1_x_e2[1], s2_x_e2[1], s3_x_e2[1], s4_x_e2[1], s5_x_e2[1], s6_x_e2[1], s7_x_e2[1], s8_x_e2[1]);
    temp5 = _mm256_broadcast_ss(&r.d[1]);
    temp6 = _mm256_mul_ps(temp4, temp5);
    res1 = (float*)&temp6;
    temp7 = _mm256_set_ps(s1_x_e2[2], s2_x_e2[2], s3_x_e2[2], s4_x_e2[2], s5_x_e2[2], s6_x_e2[2], s7_x_e2[2], s8_x_e2[2]);
    temp8 = _mm256_broadcast_ss(&r.d[2]);
    temp9 = _mm256_mul_ps(temp7, temp8);
    res2 = (float*)&temp9;
    double u1 = -(*res0 + *res1 + *res2) * inv_det1;
    double u2 = -(*(res0+1) + *(res1+1) + *(res2+1)) * inv_det2;
    double u3 = -(*(res0+2) + *(res1+2) + *(res2+2)) * inv_det3;
    double u4 = -(*(res0+3) + *(res1+3) + *(res2+3)) * inv_det4;
    double u5 = -(*(res0+4) + *(res1+4) + *(res2+4)) * inv_det5;
    double u6 = -(*(res0+5) + *(res1+5) + *(res2+5)) * inv_det6;
    double u7 = -(*(res0+6) + *(res1+6) + *(res2+6)) * inv_det7;
    double u8 = -(*(res0+7) + *(res1+7) + *(res2+7)) * inv_det8;
    // double u1 = -dot(s1_x_e2, r.d) * inv_det1;
    // double u2 = -dot(s2_x_e2, r.d) * inv_det2;
    // double u3 = -dot(s3_x_e2, r.d) * inv_det3;
    // double u4 = -dot(s4_x_e2, r.d) * inv_det4;
    if (u1 < 0. || u1 + v1 > 1.) b1 = false;
    if (u2 < 0. || u2 + v2 > 1.) b2 = false;
    if (u3 < 0. || u3 + v3 > 1.) b3 = false;
    if (u4 < 0. || u4 + v4 > 1.) b4 = false;
    if (u5 < 0. || u5 + v5 > 1.) b5 = false;
    if (u6 < 0. || u6 + v6 > 1.) b6 = false;
    if (u7 < 0. || u7 + v7 > 1.) b7 = false;
    if (u8 < 0. || u8 + v8 > 1.) b8 = false;

    // temp1 = _mm_set_ps(e11_x_d[0], e21_x_d[0], e31_x_d[0], e41_x_d[0]);
    temp2 = _mm256_set_ps(e11[0], e21[0], e31[0], e41[0], e51[0], e61[0], e71[0], e81[0]);
    temp3 = _mm256_mul_ps(temp1, temp2);
    res0 = (float*)&temp3;
    // temp4 = _mm_set_ps(e11_x_d[1], e21_x_d[1], e31_x_d[1], e41_x_d[1]);
    temp5 = _mm256_set_ps(e11[1], e21[1], e31[1], e41[1], e51[1], e61[1], e71[1], e81[1]);
    temp6 = _mm256_mul_ps(temp4, temp5);
    res1 = (float*)&temp6;
    // temp7 = _mm_set_ps(e11_x_d[2], e21_x_d[2], e31_x_d[2], e41_x_d[2]);
    temp8 = _mm256_set_ps(e11[2], e21[2], e31[2], e41[2], e51[2], e61[2], e71[2], e81[2]);
    temp9 = _mm256_mul_ps(temp7, temp8);
    res2 = (float*)&temp9;
    double t1 = -(*res0 + *res1 + *res2) * inv_det1;
    double t2 = -(*(res0+1) + *(res1+1) + *(res2+1)) * inv_det2;
    double t3 = -(*(res0+2) + *(res1+2) + *(res2+2)) * inv_det3;
    double t4 = -(*(res0+3) + *(res1+3) + *(res2+3)) * inv_det4;
    double t5 = -(*(res0+4) + *(res1+4) + *(res2+4)) * inv_det5;
    double t6 = -(*(res0+5) + *(res1+5) + *(res2+5)) * inv_det6;
    double t7 = -(*(res0+6) + *(res1+6) + *(res2+6)) * inv_det7;
    double t8 = -(*(res0+7) + *(res1+7) + *(res2+7)) * inv_det8;

    // double t1 = -dot(s1_x_e2, e11) * inv_det1;
    // double t2 = -dot(s2_x_e2, e21) * inv_det2;
    // double t3 = -dot(s3_x_e2, e31) * inv_det3;
    // double t4 = -dot(s4_x_e2, e41) * inv_det4;
    if (t1 < r.min_t || t1 > r.max_t) b1 = false;
    if (t2 < r.min_t || t2 > r.max_t) b2 = false;
    if (t3 < r.min_t || t3 > r.max_t) b3 = false;
    if (t4 < r.min_t || t4 > r.max_t) b4 = false;
    if (t5 < r.min_t || t5 > r.max_t) b5 = false;
    if (t6 < r.min_t || t6 > r.max_t) b6 = false;
    if (t7 < r.min_t || t7 > r.max_t) b7 = false;
    if (t8 < r.min_t || t8 > r.max_t) b8 = false;

    if (b1 && r.max_t > t1) r.max_t = t1;
    if (b2 && r.max_t > t2) r.max_t = t2;
    if (b3 && r.max_t > t3) r.max_t = t3;
    if (b4 && r.max_t > t4) r.max_t = t4;
    if (b5 && r.max_t > t5) r.max_t = t5;
    if (b6 && r.max_t > t6) r.max_t = t6;
    if (b7 && r.max_t > t7) r.max_t = t7;
    if (b8 && r.max_t > t8) r.max_t = t8;

    return (b1 || b2 || b3 || b4 || b5 || b6 || b7 || b8);
}

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

    r.max_t = t;
    return true;
}
