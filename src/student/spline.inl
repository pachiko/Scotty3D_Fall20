
#include "../geometry/spline.h"
#include "debug.h"

template <typename T>
T Spline<T>::cubic_unit_spline(float time, const T &position0, const T &position1,
                               const T &tangent0, const T &tangent1) {

    // TODO (Animation): Task 1a
    // Given time in [0,1] compute the cubic spline coefficients and use them to compute
    // the interpolated value at time 'time' based on the positions & tangenets

    // Note that Spline is parameterized on type T, which allows us to create splines over
    // any type that supports the * and + operators.

    float t2 = time*time;
    float t3 = time*t2;

    float h00 = 2*t3 - 3*t2 + 1;
    float h10 = t3 - 2*t2 + time;
    float h01 = -2*t3 + 3*t2;
    float h11 = t3 - t2;

    return h00*position0 + h10*tangent0 + h01*position1 + h11*tangent1;
}

template <typename T> T Spline<T>::at(float time) const {

    // TODO (Animation): Task 1b

    // Given a time, find the nearest positions & tangent values
    // defined by the control point map.

    // Transform them for use with cubic_unit_spline

    // Be wary of edge cases! What if time is before the first knot,
    // before the second knot, etc...

    // Less than 2 points
    if (control_points.empty()) return T();
    else if (control_points.size() == 1) return control_points.begin()->second;
    
    // Found the exact value?
    auto it = control_points.find(time);
    if (it != control_points.end()) return it->second;

    // Out-of-bounds
    auto lowest = control_points.begin(); // forward iterator to beginning (1st -> last)
    auto highest = control_points.rbegin(); // reverse iterator to beginning (last -> 1st)
    if (time <= lowest->first) return lowest->second;
    else if (time >= highest->first) return highest->second;

    // At least 2 knots are present. Interpolate std::map<float,T>::iterator
    auto right = control_points.upper_bound(time);
    auto left = prev(right, 1);
    
    float k0, k1 = left->first, k2 = right->first, k3;
    T p0, p1 = left->second, p2 = right->second, p3;
    float t_interval = k2 - k1;
    T p_interval = p2 - p1;

    auto right_right = next(right, 1);
    if (right_right != control_points.end()) {
        k3 = right_right->first;
        p3 = right_right->second;
    } else {
        k3 = k2 + t_interval;
        p3 = p2 + p_interval;
    }

    auto left_left = prev(left, 1);
    if (left_left != control_points.end()) {
        k0 = left_left->first;
        p0 = left_left->second;
    } else {
        k0 = k1 - t_interval;
        p0 = p1 - p_interval;
    }

    T m1 = (p2 - p0) / (k2 - k0);
    T m2 = (p3 - p1) / (k3 - k1);
    return cubic_unit_spline((time - k1)/t_interval, p1, p2, m1/t_interval, m2/t_interval);
}
