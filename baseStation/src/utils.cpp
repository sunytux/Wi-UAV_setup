/* System Headers */
#include <cmath>

#include "utils.hpp"

double normalizedAngle(double x)
{
    x = fmod(x, 2 * PI);
    if (x > PI) {
        x -= 2 * PI;
    }
    if (x < -PI) {
        x += 2 * PI;
    }
    return x;
}

double angularSubstraction(double a, double b)
{
    double diff = normalizedAngle(a) - normalizedAngle(b);
    if (diff > PI) {
        diff -= 2 * PI;
    }
    if (diff < -PI) {
        diff += 2 * PI;
    }

    return diff;
}
