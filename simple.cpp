#include "ray_tracer.hpp"
#include "CLI/CLI.hpp"

#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>

int simpleTrace(Scene &scene, std::vector<Vec3f> &framebuffer, size_t width, size_t height) {
    // Setup camera
    std::vector<Ray> rayholder;
    std::vector<float> pixelholder;

    // Generate all rays
    rayholder.reserve(height * width);
    for (size_t y = 0; y < height; y++) {
        float ys = (y + 0.5) / height;
        for (size_t x = 0; x < width; x++) {
            float xs = (x + 0.5) / width;
            Ray r = scene.cam.generate_ray(xs, ys);
            rayholder.push_back(r);
        }
    }

    pixelholder.resize(height * width, INF_F);
    auto start_t = std::chrono::high_resolution_clock::now();
    #pragma omp parallel for
    for (size_t k = 0; k<rayholder.size(); k++) {
        Ray r = rayholder[k];
        bool hit = false;

        //std::cout << r.d << " " << r.o << std::endl;
        for (auto &it : scene.meshes) {
            Vec3f p0 = scene.verts[it[0]];
            Vec3f p1 = scene.verts[it[1]];
            Vec3f p2 = scene.verts[it[2]];

            bool result = mesh_intersect(r, p0, p1, p2);
            if (result) hit = true;
        }

        if (hit) pixelholder[k] = r.max_t;
    }
    auto end_t = std::chrono::high_resolution_clock::now();

    // time in milli second
    std::chrono::duration<double, std::milli> time_elapsed = end_t - start_t;
    // time_elapsed.count shows how long it takes
    std::cout << "Time elapse: " << time_elapsed.count() << "\n";

    // Get image
    auto min_it = std::min_element(pixelholder.begin(), pixelholder.end());
    for (size_t k = 0; k < framebuffer.size(); k++)
        framebuffer[k] = Vec3f(*min_it / pixelholder[k]);

    return 0;
}