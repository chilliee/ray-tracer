#pragma once

#include "spectrum.hpp"
#include "ray.hpp"

#include "scene.hpp"

int simpleTrace(Scene &scene, std::vector<Vec3f> &framebuffer, size_t width, size_t height);