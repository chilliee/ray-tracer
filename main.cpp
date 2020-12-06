#include "ray_tracer.hpp"
#include "CLI/CLI.hpp"

#include <string>
#include <iostream>
#include <iomanip>

int main(int argc, char const *argv[])
{
    std::string scene_file;

    // CLI args
    CLI::App args{"A tiny ray-tracer"};
    args.add_option("-s,--scene", scene_file, "Scene file");
    CLI11_PARSE(args, argc, argv); // Now parse!

    Scene scene;
    scene.load(scene_file);

    std::cout << "Verteces Num: " << scene.verts.size() << std::endl;
    std::cout << "Meshes Num: " << scene.meshes.size() << std::endl;
    std::cout << "Materal Num: " << scene.materials.size() << std::endl;

    std::cout << "Camera Position: " << scene.cam.cam_transform * scene.cam.pos << std::endl;

    std::cout << "Lights Num: " << scene.lights.size() << std::endl;
    for (auto &li : scene.lights)
        std::cout << "  light@" << li.li_transform * li.pos << std::endl;
    
    

    return 0;
}
