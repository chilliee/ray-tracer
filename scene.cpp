#include "scene.hpp"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include "ray_tracer.hpp"
#include "vec3f.hpp"
#include <vector>

static void load_node(Scene &scobj, const aiScene *scene, aiNode *node) {
    for (unsigned int i = 0; i < node->mNumMeshes; i++) {
        const aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];

        std::vector<Vec3f> verts;
        for (unsigned int j = 0; j < mesh->mNumVertices; j++) {
            const aiVector3D &pos = mesh->mVertices[j];
            verts.push_back(Vec3f(pos.x, pos.y, pos.z));
        }

        std::vector<std::vector<size_t>> polys;
        for (unsigned int j = 0; j < mesh->mNumFaces; j++) {
            const aiFace &face = mesh->mFaces[j];
            if (face.mNumIndices < 3) continue;
            std::vector<size_t> poly;
            for (unsigned int k = 0; k < face.mNumIndices; k++) {
                poly.push_back(face.mIndices[k]);
            }
            polys.push_back(poly);
        }
    }
}

int Scene::load(std::string scene_file) {
    // Now read scene file
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(
        scene_file,
        aiProcess_OptimizeMeshes |
        aiProcess_ValidateDataStructure |
        aiProcess_FindInvalidData |
        aiProcess_FindInstances |
        aiProcess_FindDegenerates |
        aiProcess_DropNormals |
        aiProcess_JoinIdenticalVertices);
    
    if (!scene) {
        LOG(FATAL) << "Parsing scend " << scene_file << ": "<< importer.GetErrorString();
        return -1;
    }
}