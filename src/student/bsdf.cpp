
#include "../rays/bsdf.h"
#include "../util/rand.h"
#include "debug.h"

namespace PT {

Vec3 reflect(Vec3 dir) {

    // TODO (PathTracer): Task 6
    // Return reflection of dir about the surface normal (0,1,0).

    // In general: -wo + 2 * Dot(wo, n) * n
    // -(wo - 2*(wo.n)n) from particle simulation
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
    // if out_dir > 0, incident ray is 0 - 90 deg wrt surface normal (ie. in vacuum)
    float eta_i_over_t = (out_dir.y >= 0) ? 1.f/index_of_refraction : index_of_refraction;

    float cos_t_sqr = 1.f - (eta_i_over_t*eta_i_over_t)*(1.f - cos_i_sqr); // Derive with Snell's Law
    if (cos_t_sqr < 0) {
        was_internal = true;
        return {}; // we explicitly calculate the reflected direction later, if TIR.
    }

    float cos_t = sqrtf(cos_t_sqr);
    if (out_dir.y >= 0) cos_t *= -1.f; // refraction inverts the initial y-direction
    was_internal = false;
    
    // Eqn (8.8) of PBR Book 3rd Ed. Reflection Models: Specular Reflection and Transmission
    // wt = eta_In/eta_Tx*(-wi) + [eta_In/eta_Tx*(wi.n) - cos_Tx]n
    // since n = (0, 1, 0)
    // wt = (-eta_In/eta_Tx*wi.x,
    //      -eta_In/eta_Tx*wi.y + [eta_In/eta_Tx*wi.y - cos_Tx] = -cos_Tx,
    //      -eta_In/eta_Tx*wi.z)
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
    ret.direction = reflect(out_dir); // What direction should we sample incoming light from?
    ret.attenuation = reflectance/std::abs(ret.direction.y); // What is the ratio of reflected/incoming light?
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
     // w_i = incident light; w_o = outgoing view

    bool was_internal;
    Vec3 refracted = refract(out_dir, index_of_refraction, was_internal); // could be TIR, could be refraction

    float fresnel = 1.f; // assume it was TIR first.

    if (!was_internal) { // NOT TIR. Time for Fresnel
        // Schlick's Approx. for Fresnel Coeff.
        float cosI = std::abs(out_dir.y);
        float r0 = (1.f - index_of_refraction)/(1.f + index_of_refraction);
        r0 = r0*r0;
        fresnel = r0 + (1.f - r0)*pow(1.f - cosI, 5.f);
    }

    BSDF_Sample ret;
    // You must multiply attenuation with fresnel, since it gives the fraction of light scattered
    // Scatter = Reflect + Transmit/Refract
    if (RNG::coin_flip(fresnel)) { // reflect or TIR
        ret.direction = reflect(out_dir);
        ret.attenuation = fresnel/std::abs(ret.direction.y)*reflectance; // see Section 8.2.2 Specular Reflection
        ret.pdf = fresnel;
    } else { // refract
        ret.direction = refracted;
        float eta_i_over_t = (out_dir.y >= 0) ? 1.f/index_of_refraction : index_of_refraction;
        // see Section 8.2.3 Specular Transmission
        ret.attenuation = eta_i_over_t*eta_i_over_t*(1.f - fresnel)/std::abs(ret.direction.y)*transmittance;
        ret.pdf = 1.f - fresnel;
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
    ret.direction = refract(out_dir, index_of_refraction, was_internal); // What direction should we sample incoming light from?
    ret.attenuation = transmittance/std::abs(ret.direction.y); // What is the ratio of reflected/incoming light?
    ret.pdf = 1.f; // Was was the PDF of the sampled direction? (In this case, the PMF)
    return ret;
}

Spectrum BSDF_Refract::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // As with BSDF_Mirror, just assume that we never hit the correct
    // directions _exactly_ and always return 0.
    return {};
}

} // namespace PT
