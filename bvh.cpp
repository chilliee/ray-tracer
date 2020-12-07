#include "scene.hpp"
#include "bvh.hpp"
#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

#include <iostream>





struct BVHPrimitiveInfo
{
  BVHPrimitiveInfo() {}
  BVHPrimitiveInfo(size_t idx, BBox &bb)
      : idx(idx),
        bb(bb),
        centroid(bb.centroid()) {}
  size_t idx;
  BBox bb;
  Vec3f centroid;
};

struct BucketInfo
{
  int count = 0;
  BBox bb;
};

BVHNode::~BVHNode()
{
  if (l != NULL)
    delete l;
  if (r != NULL)
    delete r;
}

BVHAccel::BVHAccel(const std::vector<std::array<size_t, 3>> meshes,
                    std::vector<Vec3f> verticles,
                   size_t max_leaf_size)
{
  this -> verts = verticles;
  this->local_meshes = meshes;

  if (meshes.size() == 0)
    return;

  // Convinient in recursively building BVH
  this->max_leaf_size = max_leaf_size;

  std::vector<BVHPrimitiveInfo> prims_info(local_meshes.size());

  for (size_t i = 0; i < local_meshes.size(); ++i){
    BBox curBox;
    curBox.expand(verts[local_meshes[i][0]]);
    curBox.expand(verts[local_meshes[i][1]]);
    curBox.expand(verts[local_meshes[i][2]]);
    prims_info[i] = {i, curBox};
  }

  // Build BVH tree
  total_nodes = 0;
  std::vector<std::array<size_t, 3>> ordered_prims;
  ordered_prims.reserve(meshes.size());
  root = recursiveBuild(&total_nodes, 0, meshes.size(), ordered_prims, prims_info);

  // Reorder primitives
  local_meshes.swap(ordered_prims);

  return;
}

BVHNode *BVHAccel::makeLeafNode(
    BVHNode *node, size_t start, size_t end,
    std::vector<std::array<size_t, 3>> &ordered_prims,
    std::vector<BVHPrimitiveInfo> &prims_info)
{
  for (size_t i = start; i < end; i++)
  {
    size_t idx = prims_info[i].idx;
    ordered_prims.push_back(local_meshes[idx]);
  }
  node->l = NULL;
  node->r = NULL;
  return node;
}

inline size_t max_dimension(Vec3f &v)
{
  return v.x > v.y ? (v.x > v.z ? 0 : 2)
                   : (v.y > v.z ? 1 : 2);
}

BVHNode *BVHAccel::recursiveBuild(
    int *total_nodes, size_t start, size_t end,
    std::vector<std::array<size_t, 3>> &ordered_prims,
  std::vector<BVHPrimitiveInfo> &prims_info)
{
  if (start >= end)
    return NULL;

  // Create a new BVH node
  BBox bounds, center_bounds;
  for (size_t i = start; i < end; i++)
  {
    bounds.expand(prims_info[i].bb);
    center_bounds.expand(prims_info[i].centroid);
  }
  size_t range = end - start;
  BVHNode *node = new BVHNode(bounds, start, range);
  (*total_nodes)++;

  if (range == 1) // Create leaf BVHNode
    return makeLeafNode(node, start, end, ordered_prims, prims_info);

  // Select the dim with the longest extent along it
  size_t dim = max_dimension(center_bounds.extent);
  float minBound = center_bounds.min[dim];
  float maxBound = center_bounds.max[dim];
  float dimExtent = center_bounds.extent[dim];

  // Partition primitives into two sets and build children
  size_t mid = (start + end) / 2;

  // What if all primitives have a common centroid?
  if (dimExtent < EPS_F) // Create leaf BVHNode
    return makeLeafNode(node, start, end, ordered_prims, prims_info);

  // ------- Partition primitives using approximate SAH -------
  if (range == 2)
  { // Directly partition
    if (prims_info[start].centroid[dim] > prims_info[mid].centroid[dim])
      std::swap(prims_info[start], prims_info[mid]);
  }
  else
  {
    // Allocate BucketInfo for SAH partition buckets
    const size_t nBuckets = 12;
    const size_t nPartitions = nBuckets - 1;
    const float mid_split = (nPartitions - 1) / 2.;
    BucketInfo buckets[nBuckets];

    // Initialize BucketInfo for SAH partition buckets
    float extent2bucket = (float)nBuckets / dimExtent;
    auto compute_bucket = [minBound, extent2bucket, nBuckets](float centroid) {
        int b = (int) floor((centroid - minBound) * extent2bucket);
        if (b >= nBuckets) b = nBuckets - 1;
        if (b < 0) b = 0;
        return (size_t) b; };
    for (size_t i = start; i < end; i++)
    {
      size_t b = compute_bucket(prims_info[i].centroid[dim]);
      buckets[b].count++;
      buckets[b].bb.expand(prims_info[i].bb);
    }

    // Compute costs for splitting after each bucket
    float SN = bounds.surface_area();
    float cost[nPartitions];
    for (size_t i = 0; i < nPartitions; i++)
    {
      BBox A, B;
      int NA = 0, NB = 0;
      for (size_t j = 0; j <= i; j++)
      {
        A.expand(buckets[j].bb);
        NA += buckets[j].count;
      }
      for (size_t j = i + 1; j < nBuckets; j++)
      {
        B.expand(buckets[j].bb);
        NB += buckets[j].count;
      }

      float SA = A.surface_area();
      float SB = B.surface_area();
      cost[i] = 1 + (NA * SA + NB * SB) / SN;
    }

    // Find bucket to split at that minimizes SAH metric
    float min_cost = *std::min_element(&cost[0], &cost[nBuckets - 1]);
    std::vector<size_t> possible_splits;
    for (size_t i = 0; i < nPartitions; i++)
      if (cost[i] == min_cost)
        possible_splits.push_back(i);
    // Choose the best split closest to the center
    float mid_dist = INF_F;
    size_t best_split;
    for (auto &split : possible_splits)
    {
      float dist = abs(split - mid_split);
      if (dist < mid_dist)
      {
        mid_dist = dist;
        best_split = split;
      }
    }

    // Either create leaf or split primitives at selected SAH bucket
    float leaf_cost = range;
    if (range > max_leaf_size || min_cost < leaf_cost)
    {
      BVHPrimitiveInfo *pmid = std::partition(
          &prims_info[start], &prims_info[end - 1] + 1,
          [=](const BVHPrimitiveInfo &pi) {
            size_t b = compute_bucket(pi.centroid[dim]);
            return b <= best_split;
          });
      mid = pmid - &prims_info[0];
    }
    else // Create leaf BVHNode
      return makeLeafNode(node, start, end, ordered_prims, prims_info);
  }

  // Cont to child
  node->l = recursiveBuild(total_nodes, start, mid, ordered_prims, prims_info);
  node->r = recursiveBuild(total_nodes, mid, end, ordered_prims, prims_info);

  return node;
}

void BVHAccel::printTree(){
  BVHNode* cur = root;
  this->recursiveprint(root);
}

void BVHAccel::recursiveprint(BVHNode*node){
  if(node == NULL){return;}
  printf("start node %d, range %d\n", node->start, node->range);
  if(node->isLeaf()){return;}
  recursiveprint(node->l);
  recursiveprint(node->r);
}

BVHAccel::~BVHAccel()
{
  // TODO (PathTracer):
  // Implement a proper destructor for your BVH accelerator aggregate
  delete root;
}
