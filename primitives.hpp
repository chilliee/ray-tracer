#pragma once

#include "bbox.hpp"
#include "bsdf.hpp"
#include "scene.hpp"

#include <vector>

struct Intersection {
    Intersection() : t(INF_F), primitive(NULL), bsdf(NULL) {}
    double t;
    const Primitive* primitive;
    Vec3f n;
    BSDF* bsdf;
};

std::ostream &operator<<(std::ostream &os, const BBox &b);

class Primitive {
public:
    virtual BBox get_bbox() const = 0;
    virtual bool intersect(const Ray& r) const = 0;
    virtual bool intersect(const Ray& r, Intersection* i) const = 0;
};

class Triangle : public Primitive {
public:
    Triangle(Scene *scene, size_t idx);
    BBox get_bbox() const;
    bool intersect(const Ray& r) const;
    bool intersect(const Ray& r, Intersection* i) const;

private:
    const Scene* scene;
    size_t v1, v2, v3;

};