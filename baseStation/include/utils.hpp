/*
 * Header file for utils.cpp
 */

#ifndef UTILS_HPP
#define UTILS_HPP

#define MS (1e3)
#define S  (1e6)

#define MINUS_INF (1.175494351e-38)
#define PLUS_INF (3.402823466e38)

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
 *
 */
double angularSubstraction(double a, double b);

/**
 * @brief Convert from natural to dB.
 *
 * @param[in]   xNat
 *              Value in natural
 *
 * @return Value in dB.
 *
 */
double nat2db(double xNat);

/**
 * @brief Convert from dB to Natural.
 *
 * @param[in]   xDb
 *              Value in dB
 *
 * @return Value in Natural.
 *
 */
double db2nat(double xDb);

/**
 * @brief Print an easy to read report of RSS received on each antenna.
 *
 * @param[in]   rss
 *              Array of rss
 *
 * @TODO Antenna offsets are hardcoded, change that.
 *
 */
void printRss(float* rss);

/**
 * @brief Find the index of the maximum element in a list.
 *
 * @param[in]   list
 *              List of floats.
 * 
 * @param[in]   size
 *              Size of the list
 *
 * @return the index.
 *
 */
int indexOfMaxElement(float* list, int size);

#endif // UTILS_HPP
