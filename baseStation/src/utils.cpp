/* System Headers */
#include <cmath>

#include "utils.hpp"

double normalizedAngle(double x)
{
    x = fmod(x + PI / 2, PI);
    if (x < 0)
        x += PI;
    return x - PI / 2;
}