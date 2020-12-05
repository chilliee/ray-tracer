#pragma once

#include <string>

class Scene
{
private:
    /* data */
public:
    Scene();
    ~Scene();

    int load(std::string scene_file);
};
