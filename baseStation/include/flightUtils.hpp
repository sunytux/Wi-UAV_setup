/*
 * Header file for flightUtils.cpp
 */

#ifndef FLIGHTUTILS_HPP
#define FLIGHTUTILS_HPP

#include "LinuxSetup.h"
#include "unistd.h"
#include <DJI_API.h>
#include <DJI_Flight.h>
#include <DJI_Type.h>
#include <cmath>
#include <cstdlib>
#include <iostream>

int moveWithVelocity_modified(CoreAPI* api,
                              Flight* flight,
                              float32_t xVelocityDesired,
                              float32_t yVelocityDesired,
                              float32_t zVelocityDesired,
                              float32_t yawRateDesired,
                              int timeoutInMs = 2000,
                              float yawRateThresholdInDegS = 0.5,
                              float velThresholdInMs = 0.5);

int moveByPositionOffset_modified(CoreAPI* api,
                                  Flight* flight,
                                  float32_t xOffsetDesired,
                                  float32_t yOffsetDesired,
                                  float32_t zOffsetDesired,
                                  float32_t yawDesired,
                                  int timeoutInMs = 10000,
                                  float yawThresholdInDeg = 1,
                                  float posThresholdInCm = 5);

#endif // FLIGHTUTILS_HPP
