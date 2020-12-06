#pragma once

#include "spectrum.hpp"

class BSDF {
public:
    virtual Spectrum f(const Vec3f &wo, const Vec3f &wi) = 0;
    virtual Spectrum sample_f(const Vec3f &wo, Vec3f *wi, float *pdf) = 0;
    virtual Spectrum get_emission() const = 0;
    virtual bool is_delta() const = 0;
    virtual void reflect(const Vec3f &wo, Vec3f *wi);
    virtual bool refract(const Vec3f &wo, Vec3f *wi, float ior);
    Spectrum rasterize_color;
};