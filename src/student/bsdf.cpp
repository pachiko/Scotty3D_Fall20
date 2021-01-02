
#include "../rays/bsdf.h"
#include "../util/rand.h"
#include "debug.h"

namespace PT {

Vec3 reflect(Vec3 dir) {

    // TODO (PathTracer): Task 6
    // Return reflection of dir about the surface normal (0,1,0).
    return Vec3(-dir.x, dir.y, -dir.z);
}

Vec3 refract(Vec3 out_dir, float index_of_refraction, bool &was_internal) {

    // TODO (PathTracer): Task 6
    // Use Snell's Law to refract out_dir through the surface
    // Return the refracted direction. Set was_internal to false if
    // refraction does not occur due to total internal reflection,
    // and true otherwise.

    // When dot(out_dir,normal=(0,1,0)) is positive, then out_dir corresponds to a
    // ray exiting the surface into vaccum (ior = 1). However, note that
    // you should actually treat this case as _entering_ the surface, because
    // you want to compute the 'input' direction that would cause this output,
    // and to do so you can simply find the direction that out_dir would refract
    // _to_, as refraction is symmetric.
    
    float cos_i_sqr = out_dir.y*out_dir.y;
    float eta_i_over_t = (out_dir.y > 0) ? 1.f/index_of_refraction : index_of_refraction;

    float cos_t_sqr = 1 - (eta_i_over_t*eta_i_over_t)*(1 - cos_i_sqr);
    if (cos_t_sqr < 0) {
        was_internal = true;
        return {};
    }

    float cos_t = sqrtf(cos_t_sqr);
    if (out_dir.y > 0) cos_t *= -1;
    was_internal = false;
    return Vec3(-out_dir.x*eta_i_over_t, cos_t, -out_dir.z*eta_i_over_t);

}

BSDF_Sample BSDF_Lambertian::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 5
    // Implement lambertian BSDF. Use of BSDF_Lambertian::sampler may be useful

    BSDF_Sample ret;
    Vec3 in_dir = sampler.sample(ret.pdf); // What was the PDF of the sampled direction?
    ret.attenuation = evaluate(out_dir, in_dir); // What is the ratio of reflected/incoming light? The higher the brighter
    ret.direction = in_dir;       // What direction should we sample incoming light from?        
    return ret;
}

Spectrum BSDF_Lambertian::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    return albedo * (1.0f / PI_F);
}

BSDF_Sample BSDF_Mirror::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 6
    // Implement mirror BSDF

    BSDF_Sample ret;
    ret.attenuation = reflectance; // What is the ratio of reflected/incoming light?
    ret.direction = reflect(out_dir); // What direction should we sample incoming light from?
    ret.pdf = 1.f; // Was was the PDF of the sampled direction? (In this case, the PMF)
    return ret;
}

Spectrum BSDF_Mirror::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // Technically, we would return the proper reflectance
    // if in_dir was the perfectly reflected out_dir, but given
    // that we assume these are single exact directions in a
    // continuous space, just assume that we never hit them
    // _exactly_ and always return 0.
    return {};
}

BSDF_Sample BSDF_Glass::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 6

    // Implement glass BSDF.
    // (1) Compute Fresnel coefficient. Tip: use Schlick's approximation.
    // (2) Reflect or refract probabilistically based on Fresnel coefficient. Tip: RNG::coin_flip
    // (3) Compute attenuation based on reflectance or transmittance

    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?

    float eta_i = 1.f; float eta_t = index_of_refraction;
    if (out_dir.y < 0.f) std::swap(eta_i, eta_t);
    float R0 = (eta_i - eta_t) / (eta_i + eta_t);
    R0 *= R0;
    float fresnel_reflectance = R0 + (1 - R0)*std::pow((1 - std::abs(out_dir.y)), 5.f);

    BSDF_Sample ret;

    if (RNG::coin_flip(fresnel_reflectance)) {
        ret.attenuation = reflectance;
        ret.direction = reflect(out_dir);
        ret.pdf = fresnel_reflectance;
    } else {
        bool was_internal;
        ret.direction = refract(out_dir, index_of_refraction, was_internal);
        ret.attenuation = was_internal? reflectance : transmittance;
        ret.pdf = 1.f - fresnel_reflectance;
    }
    
    return ret;
}

Spectrum BSDF_Glass::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // As with BSDF_Mirror, just assume that we never hit the correct
    // directions _exactly_ and always return 0.
    return {};
}

BSDF_Sample BSDF_Diffuse::sample(Vec3 out_dir) const {
    BSDF_Sample ret;
    ret.direction = sampler.sample(ret.pdf);
    ret.emissive = radiance;
    ret.attenuation = {};
    return ret;
}

Spectrum BSDF_Diffuse::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // No incoming light is reflected; only emitted
    return {};
}

BSDF_Sample BSDF_Refract::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 6
    // Implement pure refraction BSDF.

    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?
    bool was_internal;
    BSDF_Sample ret;
    ret.attenuation = transmittance; // What is the ratio of reflected/incoming light?
    ret.direction = refract(out_dir, index_of_refraction, was_internal);       // What direction should we sample incoming light from?
    ret.pdf = 1.f; // Was was the PDF of the sampled direction? (In this case, the PMF)
    return ret;
}

Spectrum BSDF_Refract::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // As with BSDF_Mirror, just assume that we never hit the correct
    // directions _exactly_ and always return 0.
    return {};
}

} // namespace PT
