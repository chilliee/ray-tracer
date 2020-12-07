#pragma once

#include "ray_tracer.hpp"
#include "bbox.hpp"
#include <string>
#include <vector>
#include <array>






struct Camera {
    void setup(float ar_new);
    Ray generate_ray(float x, float y);

    Mat4f cam_transform;

    // In world space
    Vec3f pos;

    float hfov, ar;
    float scale_w, scale_h;
};

struct Light {
    Mat4f li_transform;
    Spectrum spectrum;
    float intensity;

    // In light space
    Vec3f pos;
};

struct Material {
    Spectrum albedo = Spectrum(1.0f);
    Spectrum reflectance = Spectrum(1.0f);
    Spectrum transmittance = Spectrum(1.0f);
    Spectrum emissive = Spectrum(1.0f);
    float intensity = 1.0f;
    float ior = 1.2f;
};

class Scene {
public:
    int load(const std::string &scene_file);

    Camera cam;
    std::vector<Vec3f> verts;
    std::vector<std::array<size_t, 3>> meshes;
    std::vector<std::pair<size_t, Material>> materials;
    std::vector<Light> lights;
};