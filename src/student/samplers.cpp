
#include "../rays/samplers.h"
#include "../util/rand.h"
#include "debug.h"

namespace Samplers {

Vec2 Rect::Uniform::sample(float &pdf) const {

    // TODO (PathTracer): Task 1
    // Generate a uniformly random point on a rectangle of size size.x * size.y
    // Tip: RNG::unit()
    float rx = RNG::unit(); // generate a random x
    float ry = RNG::unit(); // generate a random y
    pdf =  1.f/(size.x*size.y); // the PDF should integrate to 1 over the whole rectangle
    return Vec2(rx*size.x, ry*size.y);
}

Vec3 Hemisphere::Cosine::sample(float &pdf) const {

    // TODO (PathTracer): Task 6
    // You may implement this, but don't have to.

    float Xi1 = RNG::unit();
    float Xi2 = RNG::unit();

    float theta = std::asin(std::sqrtf(Xi1));
    float phi = 2.0f * PI_F * Xi2;

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);

    pdf = ys / PI_F; // PDF = cos(theta)/pi
    return Vec3(xs, ys, zs);
}

Vec3 Sphere::Uniform::sample(float &pdf) const {

    // TODO (PathTracer): Task 7
    // Generate a uniformly random point on the unit sphere (or equivalently, direction)
    // Tip: start with Hemisphere::Uniform
    Vec3 dir = hemi.sample(pdf);
    if (RNG::coin_flip(0.5f)) dir.y *= -1.f;

    pdf = 1.f/(4*PI_F); // what was the PDF at the chosen direction?
    return dir;
}

Sphere::Image::Image(const HDR_Image &image) {

    // TODO (PathTracer): Task 7
    // Set up importance sampling for a spherical environment map image.

    // You may make use of the pdf, cdf, and total members, or create your own
    // representation.

    const auto [_w, _h] = image.dimension();
    w = _w;
    h = _h;

    // Calculate Lsintheta across the image. Accumulate to normalize the pdf to 1 later.
    for (size_t j = 0; j < h; j++) {
        for (size_t i = 0; i < w; i++) {
            // 0.5f at the pixel center. Else we never sample top and bottom rows
            float Lsintheta = image.at(i, j).luma() * sin(PI_F*(j + 0.5f)/h);
            total += Lsintheta;
            cdf.push_back(total); // Joint Cdf. P(theta, phi)
        }
    }
}

Vec3 Sphere::Image::sample(float &out_pdf) const {

    // TODO (PathTracer): Task 7
    // Use your importance sampling data structure to generate a sample direction.
    // Tip: std::upper_bound can easily binary search your CDF

    float Xi = RNG::unit();

    auto lower = std::lower_bound(cdf.begin(), cdf.end(), Xi*total); // cdf is not normalized yet. Scale the RV
    auto index = std::distance(cdf.begin(), lower);
    out_pdf = *lower;
    if (index > 0) out_pdf -= cdf[index - 1]; // pdf[index] = cdf[index] - cdf[index - 1]
    out_pdf /= total;

    size_t row = index/h;
    size_t column = index%h;
    float theta = (row + 0.5f)*PI_F/h;
    float phi = (column + 0.5f)*2*PI_F/w;

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);

    return Vec3(xs, ys, zs);
}

Vec3 Point::sample(float &pmf) const {

    pmf = 1.0f;
    return point;
}

Vec3 Two_Points::sample(float &pmf) const {
    if (RNG::unit() < prob) {
        pmf = prob;
        return p1;
    }
    pmf = 1.0f - prob;
    return p2;
}

Vec3 Hemisphere::Uniform::sample(float &pdf) const {

    float Xi1 = RNG::unit();
    float Xi2 = RNG::unit();

    float theta = std::acos(Xi1);
    float phi = 2.0f * PI_F * Xi2;

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);

    pdf = 1.0f / (2.0f * PI_F);
    return Vec3(xs, ys, zs);
}

} // namespace Samplers
