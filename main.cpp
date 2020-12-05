#include "ray_tracer.hpp"
#include "CLI/CLI.hpp"

#include <string>
#include <iostream>

int main(int argc, char const *argv[])
{
    std::string scene_file;

    // Initialize Google’s logging library.
    google::InitGoogleLogging(argv[0]);

    // CLI args
    CLI::App args{"A tiny ray-tracer"};
    args.add_option("-s,--scene", scene_file, "Scene file");
    CLI11_PARSE(args, argc, argv); // Now parse!

    

    return 0;
}
