
#include "../lib/mathlib.h"
#include "debug.h"

bool BBox::hit(const Ray &ray, Vec2 &times) const {

    // TODO (PathTracer):
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // [times.x,times.y], update times with the new intersection times.

    // Calculate intersection times with each side
    float txmin = (min.x - ray.point.x)/ (ray.dir.x);
    float txmax = (max.x - ray.point.x)/ (ray.dir.x);
    float tymin = (min.y - ray.point.y)/ (ray.dir.y);
    float tymax = (max.y - ray.point.y)/ (ray.dir.y);
    float tzmin = (min.z - ray.point.z)/ (ray.dir.z);
    float tzmax = (max.z - ray.point.z)/ (ray.dir.z);

    // Vectors containing earliest and latest intersection times with each plane
    std::vector<float> tmins, tmaxs;

    // Does not intersect if NaN 
    if (!std::isnan(txmin) && !std::isnan(txmax)) {
        // Swap times if ray is going from max to min (opposite direction)
        if (txmin > txmax) std::swap(txmin, txmax);
        // Collect times that are within bounds
        if (txmin <= times.y && txmin >= times.x) tmins.push_back(txmin);
        if (txmax <= times.y && txmax >= times.x) tmaxs.push_back(txmax);
    }
    if (!std::isnan(tymin) && !std::isnan(tymax)) {
        if (tymin > tymax) std::swap(tymin, tymax);
        if (tymin <= times.y && tymin >= times.x) tmins.push_back(tymin);
        if (tymax <= times.y && tymax >= times.x) tmaxs.push_back(tymax);
    }
    if (!std::isnan(tzmin) && !std::isnan(tzmax)) {
        if (tzmin > tzmax) std::swap(tzmin, tzmax);
        if (tzmin <= times.y && tzmin >= times.x) tmins.push_back(tzmin);
        if (tzmax <= times.y && tzmax >= times.x) tmaxs.push_back(tzmax);
    }

    bool hit = checkHit(tmins, tmaxs);

    // Update timebounds
    if (hit) {
        times.x = *std::max_element(tmins.begin(), tmins.end());
        times.y = *std::min_element(tmaxs.begin(), tmaxs.end());
    }
    return hit;
}

bool BBox::checkHit(const std::vector<float> tmins, const std::vector<float> tmaxs) const {
    // Helps to compare each tmax with tmins, if any tmax < any tmin, return hit=false
    // If any vector is empty, then return hit=false
    bool hit = false;
    for (auto tmin : tmins) {
        for (auto tmax: tmaxs) {
            if (!hit) hit = !hit; // true... for now, since neither vector is empty
            if (tmin > tmax) return false;
        }
    }
    return hit;

}