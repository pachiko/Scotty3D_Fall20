
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

    float theta = std::asin(sqrtf(Xi1));
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

    // Conditional
    for (size_t j = 0; j < h; j++) {
        float sin_theta = sin(PI_F*(j + 0.5f)/h);

        std::vector<float> cond_func; // func at row v
        std::vector<float> cond_cdf; // cdf at row v

        float funcInt = 0.f;
        for (size_t i = 0; i < w; i++) {
            // 0.5f at the pixel center. Else we never sample top and bottom rows
            float Lsintheta = image.at(i, j).luma() * sin_theta;

            cond_func.push_back(Lsintheta);
            cond_cdf.push_back(funcInt); // cdf does NOT include itself ie. cdf[0] = 0
            funcInt += Lsintheta/w; // cdf[i] = cdf[i - 1] + func[i - 1]/n;
        }
        cond_cdf.push_back(funcInt); // cdf has n + 1 elements

        // actually normalize the cdf
        for (size_t i = 0; i < w + 1; i++) {
            cond_cdf[i] /= funcInt; // cdf[i]/funcInt
        }

        // save p[u|v] and cdf[u|v]
        conditional_func.push_back(cond_func);
        conditional_cdf.push_back(cond_cdf);

        marginal_func.push_back(funcInt); // marginalFunc.push_back(pConditionalV[v]->funcInt);
        marginal_cdf.push_back(marginal_I);
        marginal_I += funcInt/h; // the funcInt (total) of all funcInts (totals)
    }
    marginal_cdf.push_back(marginal_I); // cdf has n + 1 elements
    
    // actually normalize the cdf
    for (size_t j = 0; j < h + 1; j++) {
        marginal_cdf[j] /= marginal_I;  // cdf[i]/funcInt
    }

}

Vec3 Sphere::Image::sample(float &out_pdf) const {

    // TODO (PathTracer): Task 7
    // Use your importance sampling data structure to generate a sample direction.
    // Tip: std::upper_bound can easily binary search your CDF

    float Xi = RNG::unit();
    float Yi = RNG::unit();

    // cdf[offset] <= u < cdf[offset+1] <- what upper_bound returns
    auto y = --std::upper_bound(marginal_cdf.begin(), marginal_cdf.end(), Yi);
    auto ind_y = std::distance(marginal_cdf.begin(), y); // offset
    float dy = (Yi - *y) / (marginal_cdf[ind_y + 1] - *y); // (u - cdf[offset])/(cdf[offset + 1] - cdf[offset])
    float pdf_y = marginal_func[ind_y]/marginal_I; // pdf = func[offset]/funcInt
    float y_sample = (ind_y + dy)/h; // (offset + du) / count

    // Repeat for p(u|v)
    auto x = --std::upper_bound(conditional_cdf[ind_y].begin(), conditional_cdf[ind_y].end(), Xi);
    auto ind_x = std::distance(conditional_cdf[ind_y].begin(), x);
    float dx = (Xi - *x) / (conditional_cdf[ind_y][ind_x + 1] - *x);
    float pdf_x = conditional_func[ind_y][ind_x]/marginal_func[ind_y];
    float x_sample = (ind_x + dx)/w;

    // Get theta and phi
    float theta = y_sample * PI_F;
    float phi = x_sample * 2.f * PI_F;

    // Transform from (u, v) to (theta, phi) in unit sphere
    // using absolute value of the determinant of the Jacobian
    out_pdf = pdf_x*pdf_y/(2*PI_F*std::sin(theta)*PI_F); 

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
