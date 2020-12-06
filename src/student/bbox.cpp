
#include "../lib/mathlib.h"
#include "debug.h"

bool BBox::hit(const Ray &ray, Vec2 &times) const {

    // TODO (PathTracer):
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // [times.x,times.y], update times with the new intersection times.

    // holders for tmin and tmax
    float tmin, tmax;

    // holders for inverse of direction
    float invx = 1/ray.dir.x;

    // Why inverse of direction? -0.0 == 0.0, BUT 1/-0.0 = -inf.
    if (invx >= 0) {
        tmin = (min.x - ray.point.x)*invx;
        tmax = (max.x - ray.point.x)*invx;
    } else {
        tmin = (max.x - ray.point.x)*invx;
        tmax = (min.x - ray.point.x)*invx;
    }

    // Do the same for Y
    float tymin, tymax;
    float invy = 1/ray.dir.y;
    if (invy >= 0) {
        tymin = (min.y - ray.point.y)*invy;
        tymax = (max.y - ray.point.y)*invy;
    } else {
        tymin = (max.y - ray.point.y)*invy;
        tymax = (min.y - ray.point.y)*invy;
    }

    // Any min > max is not possible to result in hit
    if (tymin > tmax || tmin > tymax) return false;
    else {
        // tighten intersection time
        if (tymin > tmin) tmin = tymin;
        if (tymax < tmax) tmax = tymax;
    }

    // Do the same for Z
    float tzmin, tzmax;
    float invz = 1/ray.dir.z;
    if (invz >= 0) {
        tzmin = (min.z - ray.point.z)*invz;
        tzmax = (max.z - ray.point.z)*invz;
    } else {
        tzmin = (max.z - ray.point.z)*invz;
        tzmax = (min.z - ray.point.z)*invz;
    }
    if (tzmin > tmax || tmin > tzmax) return false;
    else {
        if (tzmin > tmin) tmin = tzmin;
        if (tzmax < tmax) tmax = tzmax;
    }

    // update time bounds. Does not block further queries if out-of-bounds
    if (times.x < tmin) times.x = tmin;
    if (times.y > tmax) times.y = tmax;
    return true;
}
