#include "ray_tracer.hpp"
#include "CLI/CLI.hpp"

//#include <intrin.h>

#include <omp.h>

#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <cstdlib>

int simdTrace(Scene &scene, std::vector<Vec3f> &framebuffer, size_t width, size_t height) {
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
//  #pragma omp parallel for
  for (size_t k = 0; k<rayholder.size(); k++) {
      Ray r = rayholder[k];
      bool hit = false;
      //std::cout << r.d << " " << r.o << std::endl;
      //for (auto &it : scene.meshes) {
      for(int i = 0; i<scene.meshes.size()/8;i++){

          Vec3f p00 = scene.verts[scene.meshes[8*i][0]];
          //Vec3f p0 = scene.verts[it[0]];

          Vec3f p01 = scene.verts[scene.meshes[8*i][1]];

          Vec3f p02 = scene.verts[scene.meshes[8*i][2]];

          Vec3f p10 = scene.verts[scene.meshes[8*i+1][0]];
          //Vec3f p0 = scene.verts[it[0]];

          Vec3f p11 = scene.verts[scene.meshes[8*i+1][1]];

          Vec3f p12 = scene.verts[scene.meshes[8*i+1][2]];

          Vec3f p20 = scene.verts[scene.meshes[8*i+2][0]];
          //Vec3f p0 = scene.verts[it[0]];

          Vec3f p21 = scene.verts[scene.meshes[8*i+2][1]];

          Vec3f p22 = scene.verts[scene.meshes[8*i+2][2]];

          Vec3f p30 = scene.verts[scene.meshes[8*i+3][0]];
          //Vec3f p0 = scene.verts[it[0]];

          Vec3f p31 = scene.verts[scene.meshes[8*i+3][1]];

          Vec3f p32 = scene.verts[scene.meshes[8*i+3][2]];

          Vec3f p40 = scene.verts[scene.meshes[8*i+4][0]];
          //Vec3f p0 = scene.verts[it[0]];

          Vec3f p41 = scene.verts[scene.meshes[8*i+4][1]];

          Vec3f p42 = scene.verts[scene.meshes[8*i+4][2]];

          Vec3f p50 = scene.verts[scene.meshes[8*i+5][0]];
          //Vec3f p0 = scene.verts[it[0]];

          Vec3f p51 = scene.verts[scene.meshes[8*i+5][1]];

          Vec3f p52 = scene.verts[scene.meshes[8*i+5][2]];

          Vec3f p60 = scene.verts[scene.meshes[8*i+6][0]];
          //Vec3f p0 = scene.verts[it[0]];

          Vec3f p61 = scene.verts[scene.meshes[8*i+6][1]];

          Vec3f p62 = scene.verts[scene.meshes[8*i+6][2]];

          Vec3f p70 = scene.verts[scene.meshes[8*i+7][0]];
          //Vec3f p0 = scene.verts[it[0]];

          Vec3f p71 = scene.verts[scene.meshes[8*i+7][1]];

          Vec3f p72 = scene.verts[scene.meshes[8*i+7][2]];

        
          bool result = intersect8(r,  p00, p00, p00, p00, p00, p00, p00, p00, p00, p00, p00, p00, p00, p00, p00, p00, p00, p00, p00, p00, p00, p00, p00, p00);
          //if (result1 ||result2||result3||result4||result5 ||result6||result7||result8) hit = true;
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

  //  #pragma omp parallel for
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

int BVHTrace(Scene &scene, std::vector<Vec3f> &framebuffer, size_t width, size_t height, int light_per_pixel) {
    // Setup camera
    std::vector<Ray> rayholder;
    std::vector<float> pixelholder;

    // Generate all rays
    rayholder.reserve(height * width * light_per_pixel);
    for (size_t y = 0; y < height; y++) {
        for (size_t x = 0; x < width; x++) {
            for (int n = 0; n < light_per_pixel; n++) {
                float ys = (y + (float) std::rand() / RAND_MAX) / height;
                float xs = (x + (float) std::rand() / RAND_MAX) / width;
                Ray r = scene.cam.generate_ray(xs, ys);
                rayholder.push_back(r);
            }
        }
    }

    // Construct BVH
    BVHAccel bvh(scene.verts, scene.meshes);

    pixelholder.resize(height * width * light_per_pixel, INF_F);
    auto start_t = std::chrono::high_resolution_clock::now();

    #pragma omp parallel for num_threads(12) schedule(static,1)
    for (size_t k = 0; k < rayholder.size(); k++) {

        int tid = omp_get_thread_num();
        Ray r = rayholder[k];
        if (bvh.intersect(r))
            pixelholder[k] = r.max_t;
    }
    auto end_t = std::chrono::high_resolution_clock::now();

    // time in milli second
    std::chrono::duration<double, std::milli> time_elapsed = end_t - start_t;
    // time_elapsed.count shows how long it takes
    std::cout << "Time elapse: " << time_elapsed.count() << "\n";

    // Get image
    auto min_it = std::min_element(pixelholder.begin(), pixelholder.end());
    for (size_t k = 0; k < framebuffer.size(); k++) {
        float pixel = 0.f;
        for (int n = 0; n < light_per_pixel; n++)
            pixel += *min_it / pixelholder[light_per_pixel * k + n];
        framebuffer[k] = Vec3f(pixel / light_per_pixel);
    }

    return 0;
}
