#pragma once

#include "ray_tracer.hpp"

struct BBox {
    Vec3f max;    ///< min corner of the bounding box
    Vec3f min;    ///< max corner of the bounding box
    Vec3f extent; ///< extent of the bounding box (min -> max)

    BBox() {
        max = Vec3f(-INF_F, -INF_F, -INF_F);
        min = Vec3f(INF_F, INF_F, INF_F);
        extent = max - min;
    }
    BBox(const Vec3f &p) : min(p), max(p) { extent = max - min; }
    BBox(const Vec3f &min, const Vec3f &max) : min(min), max(max) {
        extent = this->max - this->min;
    }

    BBox(const float minX, const float minY, const float minZ,
         const float maxX, const float maxY, const float maxZ) {
        min = Vec3f(minX, minY, minZ);
        max = Vec3f(maxX, maxY, maxZ);
        extent = max - min;
    }


    void expand(const BBox &bbox) {
        min.x = std::min(min.x, bbox.min.x);
        min.y = std::min(min.y, bbox.min.y);
        min.z = std::min(min.z, bbox.min.z);
        max.x = std::max(max.x, bbox.max.x);
        max.y = std::max(max.y, bbox.max.y);
        max.z = std::max(max.z, bbox.max.z);
        extent = max - min;
    }

    void expand(const Vec3f &p) {
        min.x = std::min(min.x, p.x);
        min.y = std::min(min.y, p.y);
        min.z = std::min(min.z, p.z);
        max.x = std::max(max.x, p.x);
        max.y = std::max(max.y, p.y);
        max.z = std::max(max.z, p.z);
        extent = max - min;
    }

    Vec3f centroid() { return (min + max) / 2; }

    float surface_area() const
    {
        if (empty())
            return 0.0;
        return 2 *
               (extent.x * extent.z + extent.x * extent.y + extent.y * extent.z);
    }

    bool empty() const { return min.x > max.x || min.y > max.y || min.z > max.z; }


    bool intersect(const Ray &r, float &t0, float &t1) const;

};
