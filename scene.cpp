#include "scene.hpp"

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include <iostream>

static Vec3f aiVec(aiVector3D aiv) { return Vec3f(aiv.x, aiv.y, aiv.z); }
static Spectrum aiSpec(aiColor3D aiv) { return Spectrum(aiv.r, aiv.g, aiv.b); }
static Mat4f aiMat(aiMatrix4x4 T) {
    return Mat4f{Vec4f{T[0][0], T[1][0], T[2][0], T[3][0]}, Vec4f{T[0][1], T[1][1], T[2][1], T[3][1]},
                Vec4f{T[0][2], T[1][2], T[2][2], T[3][2]}, Vec4f{T[0][3], T[1][3], T[2][3], T[3][3]}};
}

static void load_node(Scene &scobj, const aiScene *scene, const aiNode *node, aiMatrix4x4 transform);
static void load_camera(Scene &scobj, const aiScene *scene);
static void load_light(Scene &scobj, const aiScene *scene);

Ray Camera::generate_ray(float x, float y) {
    Vec3f d( (2 * x - 1) * scale_w, (2 * y - 1) * scale_h, -1.f );

    d = cam_transform * d;
    d.normalize();

    return Ray(pos, d, 0);
}

int Scene::load(const std::string &scene_file) {
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
        std::cerr << "Parsing scene " << scene_file << ": "<< importer.GetErrorString() << std::endl;
        return -1;
    }

    load_node(*this, scene, scene->mRootNode, aiMatrix4x4());
    load_camera(*this, scene);
    load_light(*this, scene);

    return 0;
}

static void load_mat(Scene &scobj, size_t mesh_offset, const aiScene *scene, const aiMesh *mesh) {
    const aiMaterial &ai_mat = *scene->mMaterials[mesh->mMaterialIndex];
    Material material;

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

    material.emissive *= 1.0f / material.intensity;

    scobj.materials.push_back(std::make_pair(mesh_offset, material));
}

static void load_node(Scene &scobj, const aiScene *scene, const aiNode *node, aiMatrix4x4 transform) {
    transform = transform * node->mTransformation;
    Mat4f o2w = aiMat(transform);

    for (unsigned int i = 0; i < node->mNumMeshes; i++) {
        const aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
        size_t mesh_offset = scobj.meshes.size();

        for (unsigned int j = 0; j < mesh->mNumVertices; j++) {
            const aiVector3D &pos = mesh->mVertices[j];
            scobj.verts.push_back( o2w * Vec3f(pos.x, pos.y, pos.z));
        }

        for (unsigned int j = 0; j < mesh->mNumFaces; j++) {
            const aiFace &face = mesh->mFaces[j];
            if (face.mNumIndices != 3) continue;

            std::array<size_t, 3> mesh;
            mesh[0] = face.mIndices[0] + mesh_offset;
            mesh[1] = face.mIndices[1] + mesh_offset;
            mesh[2] = face.mIndices[2] + mesh_offset;
            scobj.meshes.push_back(mesh);
        }

        load_mat(scobj, mesh_offset, scene, mesh);
    }

    // Recursively iteration
    for (unsigned int i = 0; i < node->mNumChildren; i++)
        load_node(scobj, scene, node->mChildren[i], transform);
}

static aiMatrix4x4 node_transform(const aiNode *node) {
    aiMatrix4x4 T;
    while (node) {
        T = T * node->mTransformation;
        node = node->mParent;
    }
    return T;
}

static void load_camera(Scene &scobj, const aiScene *scene) {
    if (scene->mNumCameras < 1) {
        std::cout << "No Camera Found...\n";
        return;
    }

    const aiCamera &aiCam = *scene->mCameras[0];

    scobj.cam.cam_transform = aiMat(node_transform(scene->mRootNode->FindNode(aiCam.mName)));
    float ar = aiCam.mAspect;
    float hfov = aiCam.mHorizontalFOV;
    float vfov = hfov / ar;

    scobj.cam.scale_w = tan( hfov * PI_F / (2 * 180) );
    scobj.cam.scale_h = tan( vfov * PI_F / (2 * 180) );

    scobj.cam.pos = scobj.cam.cam_transform * aiVec(aiCam.mPosition);
    scobj.cam.dir = scobj.cam.cam_transform * aiVec(aiCam.mLookAt);
}

static void load_light(Scene &scobj, const aiScene *scene) {
    if (scene->mNumLights < 1) {
        std::cout << "No Light Found...\n";
        return;
    }

    for (unsigned int i = 0; i < scene->mNumLights; i++) {
        const aiLight &ailight = *scene->mLights[i];
        const aiNode *node = scene->mRootNode->FindNode(ailight.mName);

        Light li;
        li.li_transform = aiMat(node_transform(node));
        li.pos = aiVec(ailight.mPosition);
        
        aiColor3D color = ailight.mColorDiffuse;
        float power = std::max(color.r, std::max(color.g, color.b));
        li.spectrum = Spectrum(color.r, color.g, color.b) * (1.0f / power);
        li.intensity = power;

        scobj.lights.push_back(li);
    }
}