#include "scene.hpp"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include "ray_tracer.hpp"

#include "material.hpp"

#include <vector>

static Vec3f aiVec(aiVector3D aiv) { return Vec3f(aiv.x, aiv.y, aiv.z); }
static Spectrum aiSpec(aiColor3D aiv) { return Spectrum(aiv.r, aiv.g, aiv.b); }

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

        Material::Options material;
        const aiMaterial &ai_mat = *scene->mMaterials[mesh->mMaterialIndex];

        aiColor3D albedo;
        ai_mat.Get(AI_MATKEY_COLOR_DIFFUSE, albedo);
        material.albedo = aiSpec(albedo);

        aiColor3D emissive;
        ai_mat.Get(AI_MATKEY_COLOR_EMISSIVE, emissive);
        material.emissive = aiSpec(emissive);

         aiColor3D reflectance;
        ai_mat.Get(AI_MATKEY_COLOR_REFLECTIVE, reflectance);
        material.reflectance = aiSpec(reflectance);

        aiColor3D transmittance;
        ai_mat.Get(AI_MATKEY_COLOR_TRANSPARENT, transmittance);
        material.transmittance = aiSpec(transmittance);

        ai_mat.Get(AI_MATKEY_REFRACTI, material.ior);
        ai_mat.Get(AI_MATKEY_SHININESS, material.intensity);

        aiString ai_type;
        ai_mat.Get(AI_MATKEY_NAME, ai_type);
        std::string type(ai_type.C_Str());

        if (type.find("lambertian") != std::string::npos) {
            material.type = Material_Type::lambertian;
        } else if (type.find("mirror") != std::string::npos) {
            material.type = Material_Type::mirror;
        } else if (type.find("refract") != std::string::npos) {
            material.type = Material_Type::refract;
        } else if (type.find("glass") != std::string::npos) {
            material.type = Material_Type::glass;
        } else if (type.find("diffuse_light") != std::string::npos) {
            material.type = Material_Type::diffuse_light;
        } else {
            material = Material::Options();
        }
        material.emissive *= 1.0f / material.intensity;
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