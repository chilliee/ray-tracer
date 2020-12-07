#include "ray_tracer.hpp"
#include "CLI/CLI.hpp"

#include <string>
#include <iostream>
#include <iomanip>
#include <fstream>

int main(int argc, char const *argv[]) {
    std::string scene_file, output_file="out.ppm";
    size_t width=640, height=480;

    // CLI args
    CLI::App args{"A tiny ray-tracer"};
    args.add_option("-s,--scene", scene_file, "Scene file");
    args.add_option("-x,--width", width, "Image width");
    args.add_option("-y,--height", height, "Image height");
    args.add_option("-o,--output", output_file, "Output file");
    CLI11_PARSE(args, argc, argv); // Now parse!

    Scene scene;
    if (scene.load(scene_file) < 0)
        return -1;

    std::cout << "Verteces Num: " << scene.verts.size() << std::endl;
    std::cout << "Meshes Num: " << scene.meshes.size() << std::endl;
    // std::cout << "Materal Num: " << scene.materials.size() << std::endl;
    std::cout << "Camera Position: " << scene.cam.pos << "\nCamera FOV: " << scene.cam.hfov / PI_F * 180 << std::endl;
    //std::cout << "Lights Num: " << scene.lights.size() << std::endl;
    //for (auto &li : scene.lights)
    //    std::cout << "  light@" << li.li_transform * li.pos << std::endl;

    // Setup camera
    scene.cam.setup((float) ((double) width / (double) height));

    std::vector<Vec3f> framebuffer(width*height);

    simpleTrace(scene, framebuffer, width, height);

    std::ofstream ofs; // save the framebuffer to file
    ofs.open(output_file);
    ofs << "P6\n" << width << " " << height << "\n255\n";
    for (size_t i = 0; i < height*width; ++i) {
        for (size_t j = 0; j<3; j++) {
            ofs << (char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
        }
    }
    ofs.close();

    return 0;
}
