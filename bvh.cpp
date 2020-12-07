#include "bvh.hpp"
#include "ray_tracer.hpp"

#include <algorithm>
#include <iostream>

struct BVHPrimitiveInfo {
    BVHPrimitiveInfo() {}
    BVHPrimitiveInfo(size_t idx, const BBox &bb)
        : idx(idx),
          bb(bb),
          c(bb.centroid()) {}

    size_t idx;
    size_t bidx = 0;
    BBox bb;
    Vec3f c;
};

struct BucketInfo {
    int cnt = 0;
    BBox bb;
};

BVHAccel::BVHAccel(const Vertex &verts, const Mesh &meshes, size_t max_leaf_size) {
    if (verts.size() == 0 || meshes.size() == 0)
        return;
    
    this->verts = &verts;
    this->primitives.resize(meshes.size());
    for (size_t i = 0; i < meshes.size(); i++)
        this->primitives[i] = &meshes[i];

    this->max_leaf_size = max_leaf_size;

    // Initialize infos for primitives
    std::vector<BVHPrimitiveInfo> primInfos(meshes.size());
    for (size_t i = 0; i < primInfos.size(); i++) {
        const Triangle &tri = meshes[i];
        BBox bb;
        bb.expand(verts[tri[0]]);
        bb.expand(verts[tri[1]]);
        bb.expand(verts[tri[2]]);

        primInfos[i] = {i, bb};
    }

    // Build BVH Tree
    std::vector<const Triangle *> orderedPrims;
    orderedPrims.reserve(primitives.size());

    root = RecursiveBuild(0, primitives.size(), orderedPrims, primInfos);

    primitives.swap(orderedPrims);
    primInfos.resize(0);
    orderedPrims.resize(0);

    return;
}

inline size_t maxDimension(Vec3f &v) {
    return v.x > v.y ? (v.x > v.z ? 0 : 2)
                     : (v.y > v.z ? 1 : 2);
}

BVHNode *BVHAccel::RecursiveBuild(
    size_t start, size_t end,
    std::vector<const Triangle *> &orderedPrims,
    std::vector<BVHPrimitiveInfo> &primInfos) {

    // Check node validity
    if (start >= end) return nullptr;

    // Create a new BVH node
    BBox bounds;
    for (size_t i = start; i < end; i++)
        bounds.expand(primInfos[i].bb);
    size_t range = end - start;
    BVHNode *node = new BVHNode(bounds, start, range);

    // Single primitive must be a leaf node
    if (range == 1) {
        size_t idx = primInfos[start].idx;
        orderedPrims.push_back(primitives[idx]);
        node->l = node->r = nullptr;
        return node;
    }

    // Select the dim with the longest extent along it
    BBox centroidBounds;
    for (size_t i = start; i < end; i++)
        centroidBounds.expand(primInfos[i].c);
    size_t dim = maxDimension(centroidBounds.extent);
    float lower = centroidBounds.min[dim];
    float extent = centroidBounds.extent[dim];
    if (extent < EPS_F) { // All primitives have the same centroid
        for (size_t i = start; i < end; i++) {
            size_t idx = primInfos[i].idx;
            orderedPrims.push_back(primitives[idx]);
        }
        node->l = node->r = nullptr;
        return node;
    }

    /* ---------- SAH Partition ---------- */
    size_t mid;
    if (range == 2) {
        mid = (start + end) / 2;
        if (primInfos[start].c[dim] > primInfos[mid].c[dim])
            std::swap(primInfos[start], primInfos[mid]);
    } else {
        // Bucket initialization
        const size_t nBuckets = 12;
        BucketInfo buckets[nBuckets];
        float e2b = nBuckets * (1 - EPS_F) / extent;
        for (size_t i = start; i < end; i++) {
            int b = e2b * (primInfos[i].c[dim] - lower);
            primInfos[i].bidx = b;
            buckets[b].cnt++;
            buckets[b].bb.expand(primInfos[i].bb);
        }

        // Compute the cost for each possible split
        float cost[nBuckets - 1];
        std::fill(&cost[0], &cost[nBuckets-2]+1, 1.);
        BBox A, B;
        int NA = 0, NB = 0;
        float SN = bounds.surface_area();
        float iSN = iSN = 1. / SN;
        for (size_t i = 0; i < nBuckets - 1; i++) {
            NA += buckets[i].cnt;
            A.expand(buckets[i].bb);
            cost[i] += NA * A.surface_area() * iSN;
            NB += buckets[nBuckets - 1 - i].cnt;
            B.expand(buckets[nBuckets - 1 - i].bb);
            cost[nBuckets - 2 - i] += NB * B.surface_area() * iSN;
        }

        // Choose the lowest cost
        float *pSplit = std::min_element(&cost[0], &cost[nBuckets - 1]);
        float minCost = *pSplit;
        size_t minSplit = pSplit - &cost[0];

        // Do partition
        float directTraversal = (float) range;
        if (minCost < directTraversal || range > max_leaf_size) {
            BVHPrimitiveInfo *pMid = std::partition(
                &primInfos[start], &primInfos[end-1]+1,
                [minSplit](const BVHPrimitiveInfo &pi) {
                    return pi.bidx <= minSplit;
                });
            mid = pMid - &primInfos[0];
        } else {
            for (size_t i = start; i < end; i++) {
                size_t idx = primInfos[i].idx;
                orderedPrims.push_back(primitives[idx]);
            }
            node->l = node->r = nullptr;
            return node;
        }
    }

    node->l = RecursiveBuild(start, mid, orderedPrims, primInfos);
    node->r = RecursiveBuild(mid, end, orderedPrims, primInfos);

    return node;
}

BVHAccel::~BVHAccel() {
    if (primitives.empty())
        return;

    std::vector<BVHNode *> nodesToVisit;
    nodesToVisit.reserve(64);
    nodesToVisit.push_back(root);

    while (!nodesToVisit.empty()) {
        BVHNode *node = nodesToVisit.back();
        nodesToVisit.pop_back();
        if (node->r != NULL)
            nodesToVisit.push_back(node->r);
        if (node->l != NULL)
            nodesToVisit.push_back(node->l);
        delete node;
    }
}

BBox BVHAccel::get_bbox() const { return root->bb; }

bool BVHAccel::intersect(Ray &ray) const {
    if (primitives.empty()) return false;

    float t0, t1;
    if (!root->bb.intersect(ray, t0, t1)) return false;

    bool hit = false;
    std::vector<BVHNode *> nodesToVisit;
    nodesToVisit.reserve(64);
    nodesToVisit.push_back(root);

    while (!nodesToVisit.empty()) {
        BVHNode *node = nodesToVisit.back();
        nodesToVisit.pop_back();

        if (node->isLeaf()) {
            for (size_t i = node->start; i < node->start + node->range; i++) {
                const Triangle &tri = *(primitives[i]);
                const Vec3f &v0 = (*verts)[tri[0]];
                const Vec3f &v1 = (*verts)[tri[1]];
                const Vec3f &v2 = (*verts)[tri[2]];

                if (mesh_intersect(ray, v0, v1, v2))
                    hit = true;
            }
        } else {
            BVHNode *lChild = node->l;
            BVHNode *rChild = node->r;

            float tl, tr;
            bool hitl, hitr;

            if (lChild) {
                float tmp;
                hitl = lChild->bb.intersect(ray, tl, tmp);
            } else
                hitl = false;
            
            if (rChild) {
                float tmp;
                hitr = rChild->bb.intersect(ray, tr, tmp);
            } else
                hitr = false;
            
            if (hitl && hitr) {
                BVHNode *first = tl < tr ? lChild : rChild;
                BVHNode *second = tl < tr ? rChild : lChild;
                nodesToVisit.push_back(second);
                nodesToVisit.push_back(first);
            }
            else if (hitl && !hitr)
                nodesToVisit.push_back(lChild);
            else if (!hitl && hitr)
                nodesToVisit.push_back(rChild);
        }
    }

    return hit;
}