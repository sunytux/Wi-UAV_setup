/* DJI OSDK Library Headers*/
#include "LinuxFlight.h"
#include "LinuxSetup.h"
#include "unistd.h"
#include <DJI_API.h>
#include <DJI_Flight.h>
#include <DJI_Type.h>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "utils.hpp"

#define SPEED_FACTOR (1) /* in m/s */

int moveWithVelocity_modified(CoreAPI* api,
                              Flight* flight,
                              float32_t xVelocityDesired,
                              float32_t yVelocityDesired,
                              float32_t zVelocityDesired,
                              float32_t yawRateDesired,
                              int timeoutInMs,
                              float yawRateThresholdInDegS,
                              float velThresholdInMs)
{
    uint8_t flag = 0x49; // Velocity Control

    VelocityData curVelocity = api->getBroadcastData().v;
    CommonData curW = api->getBroadcastData().w;

    double yawRateDesiredRad = deg2rad(yawRateDesired);
    double yawRateThresholdInRadS = deg2rad(yawRateThresholdInDegS);

    int elapsedTime = 0;

    while (std::abs(curVelocity.x - xVelocityDesired) > velThresholdInMs
           || std::abs(curVelocity.y - yVelocityDesired) > velThresholdInMs
           || std::abs(curVelocity.z - zVelocityDesired) > velThresholdInMs
           || std::abs(curW.z - yawRateDesiredRad) > yawRateThresholdInRadS) {
        if (elapsedTime >= timeoutInMs){
            break;
        }

        flight->setMovementControl(flag,
                                   xVelocityDesired,
                                   yVelocityDesired,
                                   zVelocityDesired,
                                   yawRateDesired);
        usleep(20 * MS);
        elapsedTime += 20;

        curW = api->getBroadcastData().w;
        curVelocity = api->getBroadcastData().v;
    }

    /* @TODO Remove me */
    /*
    if (elapsedTime >= timeoutInMs) {
        std::cout << "Timed out " << elapsedTime << " ms" << std::endl;
        if (std::abs(curVelocity.x - xVelocityDesired) > velThresholdInMs) {
            std::cout << "    because of x, delta_dot_x = "
                      << std::abs(curVelocity.x - xVelocityDesired)
                      << " - Vel_threshold: " << velThresholdInMs
                      << std::endl;
        }
        if (std::abs(curVelocity.y - yVelocityDesired) > velThresholdInMs) {
            std::cout << "    because of y, delta_dot_y = "
                      << std::abs(curVelocity.y - yVelocityDesired)
                      <<  " - Vel_threshold: " << velThresholdInMs
                      << std::endl;
        }
        if (std::abs(curVelocity.z - zVelocityDesired) > velThresholdInMs) {
            std::cout << "    because of z, delta_dot_z = "
                      << std::abs(curVelocity.z - zVelocityDesired)
                      << " - Vel_threshold: " << velThresholdInMs
                      << std::endl;
        }
        if (std::abs(curW.z - yawRateDesiredRad) > yawRateThresholdInRadS) {
            std::cout << "    because of yaw, delta_dot_yaw = "
                      << rad2deg(std::abs(curW.z - yawRateDesiredRad)) 
                      << " - curW.z: " << rad2deg(curW.z)
                      << " - yawRateDesiredRad: " << rad2deg(yawRateDesiredRad)
                      << " - yaw_threshold: " << rad2deg(yawRateThresholdInRadS)
                      << std::endl;
        }
    }
    */

    return 1;
}

int moveByPositionOffset_modified(CoreAPI* api,
                                  Flight* flight,
                                  float32_t xOffsetDesired,
                                  float32_t yOffsetDesired,
                                  float32_t zOffsetDesired,
                                  float32_t yawDesired,
                                  int timeoutInMs,
                                  float yawThresholdInDeg,
                                  float posThresholdInCm)
{
    uint8_t flag = 0x91; // Position Control

    // Get current poition
    PositionData curPosition = api->getBroadcastData().pos;
    PositionData originPosition = curPosition;
    DJI::Vector3dData curLocalOffset;

    DJI::EulerAngle curEuler = Flight::toEulerAngle(api->getBroadcastData().q);

    // Convert position offset from first position to local coordinates
    localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);

    // See how much farther we have to go
    float32_t xOffsetRemaining = xOffsetDesired - curLocalOffset.x;
    float32_t yOffsetRemaining = yOffsetDesired - curLocalOffset.y;
    float32_t zOffsetRemaining = zOffsetDesired - curLocalOffset.z;

    // Conversions
    double yawDesiredRad = deg2rad(yawDesired);
    double yawThresholdInRad = deg2rad(yawThresholdInDeg);
    float32_t posThresholdInM = posThresholdInCm / 100;

    int elapsedTime = 0;
    float xCmd, yCmd, zCmd;

    /*! Calculate the inputs to send the position controller. We implement basic
        receding setpoint position control and the setpoint is always 1 m away
        from the current position - until we get within a threshold of the goal.
        From that point on, we send the remaining distance as the setpoint.
    !*/
    if (xOffsetDesired > 0)
        xCmd = xOffsetDesired < SPEED_FACTOR ? xOffsetDesired : SPEED_FACTOR;
    else if (xOffsetDesired < 0)
        xCmd = xOffsetDesired > -1 * SPEED_FACTOR ? xOffsetDesired :
                                                    -1 * SPEED_FACTOR;
    else
        xCmd = 0;

    if (yOffsetDesired > 0)
        yCmd = yOffsetDesired < SPEED_FACTOR ? yOffsetDesired : SPEED_FACTOR;
    else if (yOffsetDesired < 0)
        yCmd = yOffsetDesired > -1 * SPEED_FACTOR ? yOffsetDesired :
                                                    -1 * SPEED_FACTOR;
    else
        yCmd = 0;

    zCmd = curPosition.height + zOffsetDesired;

    //! Main closed-loop receding setpoint position control
    while (std::abs(xOffsetRemaining) > posThresholdInM
           || std::abs(yOffsetRemaining) > posThresholdInM
           || std::abs(zOffsetRemaining) > posThresholdInM
           || std::abs(curEuler.yaw - yawDesiredRad) > yawThresholdInRad) {
        // Check timeout
        if (elapsedTime >= timeoutInMs) {
            break;
        }

        // MovementControl API call
        flight->setMovementControl(flag, xCmd, yCmd, zCmd, yawDesired);
        usleep(20 * MS);
        elapsedTime += 20;

        // Get current position in required coordinates and units
        curEuler = Flight::toEulerAngle(api->getBroadcastData().q);
        curPosition = api->getBroadcastData().pos;
        localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);

        // See how much farther we have to go
        xOffsetRemaining = xOffsetDesired - curLocalOffset.x;
        yOffsetRemaining = yOffsetDesired - curLocalOffset.y;
        zOffsetRemaining = zOffsetDesired - curLocalOffset.z;
        // See if we need to modify the setpoint
        if (std::abs(xOffsetRemaining) < SPEED_FACTOR)
            xCmd = xOffsetRemaining;
        if (std::abs(yOffsetRemaining) < SPEED_FACTOR)
            yCmd = yOffsetRemaining;
    }
    return 1;
}