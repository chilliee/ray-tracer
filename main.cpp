#include "ray_tracer.hpp"
#include "CLI/CLI.hpp"

#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>

bool intersect(Ray& r, const Vec3f &p0, const Vec3f &p1, const Vec3f &p2) {
  Vec3f e1 = p1 - p0;
  Vec3f e2 = p2 - p0;

  Vec3f e1_x_d = cross(e1, r.d);
  double det = dot(e1_x_d, e2);
  if (det < EPS_F && det > -EPS_F) return false;

  double inv_det = 1.0 / det;
  Vec3f s = r.o - p0;
  double v = dot(e1_x_d, s) * inv_det;
  if (v < 0. || v > 1.) return false;

  Vec3f s_x_e2 = cross(s, e2);
  double u = -dot(s_x_e2, r.d) * inv_det;
  if (u < 0. || u + v > 1.) return false;

  double t = -dot(s_x_e2, e1) * inv_det;
  if (t < r.min_t || t > r.max_t) return false;

  if (r.max_t > t)
    r.max_t = t;
  return true;
}

int main(int argc, char const *argv[])
{
    std::string scene_file;
    size_t width=640, height=480;

    // CLI args
    CLI::App args{"A tiny ray-tracer"};
    args.add_option("-s,--scene", scene_file, "Scene file");
    args.add_option("-x,--width", width, "image width");
    args.add_option("-y,--height", height, "image height");
    CLI11_PARSE(args, argc, argv); // Now parse!

    Scene scene;
    if (scene.load(scene_file) < 0)
        return -1;

    std::cout << "Verteces Num: " << scene.verts.size() << std::endl;
    std::cout << "Meshes Num: " << scene.meshes.size() << std::endl;
    std::cout << "Materal Num: " << scene.materials.size() << std::endl;
    std::cout << "Camera Position: " << scene.cam.pos << " FOV: " << scene.cam.hfov << std::endl;
    std::cout << "Lights Num: " << scene.lights.size() << std::endl;
    for (auto &li : scene.lights)
        std::cout << "  light@" << li.li_transform * li.pos << std::endl;

    // Setup camera
    scene.cam.setup((float) ((double) width / (double) height));

    std::vector<Vec3f> framebuffer(width*height);

    // Intersect test start here
    for (size_t y = 0; y < height; y++) {
        float ys = ((double) y + 0.5) / (double) height;
        for (size_t x = 0; x < width; x++) {
            float xs = ((double) x + 0.5) / (double) width;
            Ray r = scene.cam.generate_ray(xs, ys);
            bool hit = false;

            //std::cout << r.d << " " << r.o << std::endl;

            for (auto &it : scene.meshes) {
                Vec3f p0 = scene.verts[it[0]];
                Vec3f p1 = scene.verts[it[1]];
                Vec3f p2 = scene.verts[it[2]];

                bool result = intersect(r, p0, p1, p2);
                // std::cout << "  " << p0 << " " << p1 << " " << p2 << " result: " << result << std::endl;
                if (result) hit = true;
            }

            if (hit) {
                // std::cout << r.max_t << std::endl;
                framebuffer[x+y*width] = Vec3f(1.0, 1.0, 1.0);
            }
        }
    }

    std::ofstream ofs; // save the framebuffer to file
    ofs.open("./out.ppm");
    ofs << "P6\n" << width << " " << height << "\n255\n";
    for (size_t i = 0; i < height*width; ++i) {
        for (size_t j = 0; j<3; j++) {
            ofs << (char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
        }
    }
    ofs.close();

    return 0;
}
