#pragma once

#include "bbox.hpp"
#include "ray_tracer.hpp"
#include <string>
#include <vector>
#include <array>

typedef std::array<size_t, 3> Triangle;
typedef std::vector<Triangle> Mesh;
typedef std::vector<Vec3f> Vertex;

struct BVHNode {
    BVHNode(BBox bb, size_t start, size_t range)
        : bb(bb), start(start), range(range), l(NULL), r(NULL) {}

    inline bool isLeaf() const { return l == NULL && r == NULL; }

    BBox bb;      ///< bounding box of the node
    size_t start; ///< start index into the primitive list
    size_t range; ///< range of index into the primitive list
    BVHNode *l;   ///< left child node
    BVHNode *r;   ///< right child node
};

struct BVHPrimitiveInfo;
struct BucketInfo;

class BVHAccel {
public:
    BVHAccel() {}
    BVHAccel(const Vertex &verts, const Mesh &meshes, size_t max_leaf_size = 4);
    ~BVHAccel();

    BBox get_bbox() const;
    bool intersect(Ray &r) const;

    BVHNode *root; ///< root node of the BVH
private:
    size_t max_leaf_size;

    const Vertex *verts;
    std::vector<const Triangle *> primitives;

    BVHNode *RecursiveBuild(
        size_t start, size_t end,
        std::vector<const Triangle *> &orderedPrims,
        std::vector<BVHPrimitiveInfo> &primInfos );
};