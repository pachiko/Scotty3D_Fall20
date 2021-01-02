
#include "../rays/env_light.h"
#include "debug.h"
#include <iostream>
#include <limits>

namespace PT {

Light_Sample Env_Map::sample() const {

    Light_Sample ret;
    ret.distance = std::numeric_limits<float>::infinity();

    // TODO (PathTracer): Task 7
    // Uniformly sample the sphere. Tip: implement Samplers::Sphere::Uniform
    // Samplers::Sphere::Uniform uniform;
    // ret.direction = uniform.sample(ret.pdf);

    // Once you've implemented Samplers::Sphere::Image, remove the above and
    // uncomment this line to use importance sampling instead.
    ret.direction = sampler.sample(ret.pdf);

    ret.radiance = sample_direction(ret.direction);
    return ret;
}

Spectrum Env_Map::sample_direction(Vec3 dir) const {

    // TODO (PathTracer): Task 7
    // Find the incoming light along a given direction by finding the corresponding
    // place in the enviornment image. You should bi-linearly interpolate the value
    // between the 4 image pixels nearest to the exact direction.

    dir.normalize();
    float theta = std::acos(-dir.y);
    float phi = std::atan2(dir.z, dir.x) + PI_F;

    size_t w = image.dimension().first; 
    size_t h = image.dimension().second;

    float x = phi/(2*PI_F)*w;
    float y = theta/PI_F*h;

    size_t lx = (size_t) floor(x - 0.5f); 
    size_t ly = (size_t) floor(y - 0.5f);
    lx = std::clamp(lx, size_t(0), w - 1);
    ly = std::clamp(ly, size_t(0), h - 1);
    size_t ux = std::clamp(lx + 1, size_t(0), w - 1); 
    size_t uy = std::clamp(ly + 1, size_t(0), h - 1); 

    Spectrum lxly = image.at(lx, ly);
    Spectrum lxuy = image.at(lx, uy);
    Spectrum uxly = image.at(ux, ly);
    Spectrum uxuy = image.at(ux, uy);

    float s = std::clamp(x - lx - 0.5f, 0.f, 1.f);
    float t = std::clamp(y - ly - 0.5f, 0.f, 1.f);

    return (lxly*(1 - s) + uxly*s) * (1 - t) + (lxuy*(1 - s) + uxuy*s) * t;
}

Light_Sample Env_Hemisphere::sample() const {
    Light_Sample ret;
    ret.direction = sampler.sample(ret.pdf);
    ret.radiance = radiance;
    ret.distance = std::numeric_limits<float>::infinity();
    return ret;
}

Spectrum Env_Hemisphere::sample_direction(Vec3 dir) const {
    if (dir.y > 0.0f)
        return radiance;
    return {};
}

Light_Sample Env_Sphere::sample() const {
    Light_Sample ret;
    ret.direction = sampler.sample(ret.pdf);
    ret.radiance = radiance;
    ret.distance = std::numeric_limits<float>::infinity();
    return ret;
}

Spectrum Env_Sphere::sample_direction(Vec3) const { return radiance; }

} // namespace PT
