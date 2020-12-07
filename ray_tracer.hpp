#pragma once

#include "spectrum.hpp"
#include "ray.hpp"

#include "scene.hpp"

bool mesh_intersect(Ray& r, const Vec3f &p0, const Vec3f &p1, const Vec3f &p2);
int simpleTrace(Scene &scene, std::vector<Vec3f> &framebuffer, size_t width, size_t height);