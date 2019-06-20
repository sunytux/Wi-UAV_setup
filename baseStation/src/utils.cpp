/* System Headers */
#include <algorithm>
#include <cmath>
#include <iostream>

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

double nat2db(double xNat)
{
    return 20.0f * log10(xNat);
}

double db2nat(double xDb)
{
    return pow(10.0f, xDb / 20.0f);
}

void printRss(float* rss)
{
    printf("           %d\n", 2);
    printf("  -------------------\n");
    printf("  |      %.1f      |\n", nat2db(rss[2]));
    printf("%d | %.1f     %.1f | %d\n", 1, nat2db(rss[1]), nat2db(rss[3]), 3);
    printf("  |      %.1f      |\n", nat2db(rss[0]));
    printf("  -------------------\n");
    printf("           %d\n", 0);
}

int indexOfMaxElement(float* list, int size)
{
    return std::distance(list, std::max_element(list, list + size));
}
