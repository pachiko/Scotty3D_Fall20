
#include "../util/camera.h"
#include "debug.h"

Ray Camera::generate_ray(Vec2 screen_coord) const {

    // TODO (PathTracer): Task 1
    // compute position of the input sensor sample coordinate on the
    // canonical sensor plane one unit away from the pinhole.
    // Tip: compute the ray direction in view space and use
    // the camera transform to transform it back into world space.

    // Screen space [-0.5, 0.5]
    // No need to flip along x-axis. Diff convention now
    screen_coord -= Vec2(.5f);

    // Camera Space
    float h = 2*tan(vert_fov/2.f * PI_F/ 180.f); // tan(fov/2) = 0.5*h/1
    float w = h*aspect_ratio; // ar = w/h
    screen_coord *= Vec2(w, h);

    // Ray from camera origin to sensor sample point
    Vec3 dir = Vec3(screen_coord.x, screen_coord.y, -1.f).unit();

    // Transform to world space, dont translate dir, since 
    // vectors have no origin
    dir = iview.rotate(dir);
    return Ray(pos(), dir);
}
