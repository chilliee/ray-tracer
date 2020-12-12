#pragma once

#include "spectrum.hpp"
#include "ray.hpp"
#include "bvh.hpp"
#include <immintrin.h>
#include "scene.hpp"

bool mesh_intersect(Ray& r, const Vec3f &p0, const Vec3f &p1, const Vec3f &p2);
int simpleTrace(Scene &scene, std::vector<Vec3f> &framebuffer, size_t width, size_t height);
int simdTrace(Scene &scene, std::vector<Vec3f> &framebuffer, size_t width, size_t height);
int BVHTrace(Scene &scene, std::vector<Vec3f> &framebuffer, size_t width, size_t height, int light_per_pixel=4);

bool intersect8(Ray& r, const Vec3f &p00, const Vec3f &p01, const Vec3f &p02,
const Vec3f &p10, const Vec3f &p11, const Vec3f &p12, const Vec3f &p20,
const Vec3f &p21, const Vec3f &p22, const Vec3f &p30, const Vec3f &p31,
const Vec3f &p32, const Vec3f &p40, const Vec3f &p41, const Vec3f &p42,
const Vec3f &p50, const Vec3f &p51, const Vec3f &p52, const Vec3f &p60,
const Vec3f &p61, const Vec3f &p62, const Vec3f &p70, const Vec3f &p71,
const Vec3f &p72);
