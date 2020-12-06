#pragma once
#include "bbox.hpp"
#include "ray_tracer.hpp"
#include <string>
#include <vector>
#include <array>
struct BVHNode
{
  BVHNode(BBox bb, size_t start, size_t range)
      : bb(bb), start(start), range(range), l(NULL), r(NULL) {}

  ~BVHNode();

  inline bool isLeaf() const { return l == NULL && r == NULL; }

  BBox bb;      ///< bounding box of the node
  size_t start; ///< start index into the primitive list
  size_t range; ///< range of index into the primitive list
  BVHNode *l;   ///< left child node
  BVHNode *r;   ///< right child node
};

struct BVHPrimitiveInfo;
struct BucketInfo;

/**
 * Bounding Volume Hierarchy for fast Ray - Primitive intersection.
 * Note that the BVHAccel is an Aggregate (A Primitive itself) that contains
 * all the primitives it was built from. Therefore once a BVHAccel Aggregate
 * is created, the original input primitives can be ignored from the scene
 * during ray intersection tests as they are contained in the aggregate.
 */
class BVHAccel
{
public:
  BVHAccel() {}
  BVHAccel(const std::vector<std::array<size_t, 3>> meshes, size_t max_leaf_size = 4);

  ~BVHAccel();


  BBox get_bbox() const;





private:
  std::vector<std::array<size_t, 3>> local_meshes;
  BVHNode *root; ///< root node of the BVH
  int total_nodes;
  size_t max_leaf_size;
  BVHNode *makeLeafNode(
      BVHNode *node, size_t start, size_t end,
      std::vector<std::array<size_t, 3>> &ordered_prims);
  BVHNode *recursiveBuild(
      int *total_nodes, size_t start, size_t end,
      std::vector<std::array<size_t, 3>> &ordered_prims);
};
