/*
 * Header file for utils.cpp
 */

#ifndef UTILS_HPP
#define UTILS_HPP

#define MS (1000u)

#define PI (3.14159265359f)
#define deg2rad(x) (((x) *PI) / 180.f)
#define rad2deg(x) (((x) *180.f) / PI)

#define ERROR_STATUS -1
#define SUCCESS_STATUS 0

/**
 * @brief Normalize angle between -PI and +PI.
 *
 * @param[in]   x
 *              Angle in radian            
 *
 */
double normalizedAngle(double x);

/**
 * @brief Compute the difference between two  angles.
 *
 * @param[in]   a and b
 *              Angles in radian
 *
 * @return angle in radian between -PI and +PI.
 */
double angularSubstraction(double a, double b);

#endif // UTILS_HPP
