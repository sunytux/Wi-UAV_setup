/* System Headers */
#include <iostream>
#include <string.h>
#include <thread>
#include <unistd.h>
#include <limits>
#include <algorithm>
#include <fstream>

/*DJI Linux Application Headers */
#include "LinuxCleanup.h"
#include "LinuxFlight.h"
#include "LinuxSerialDevice.h"
#include "LinuxSetup.h"
#include "ReadUserConfig.h"

/* DJI OSDK Library Headers*/
#include <DJI_Flight.h>

/* @TODO Manage includes */
#include "radio.hpp"

using namespace DJI::onboardSDK;

/* @TODO move that */
/* Prototype */
int routineSquare(CoreAPI*, Flight*);
int routineLocate(CoreAPI*, Flight*);
/**
 * @brief Thread function for making the drone rotate.
 *
 *  The given DJI command is blocking hence the Thread.
 *
 * @param[in]   api
 *              DJI core API.
 * @param[in]   flight
 *              DJI flight object.
 */
void turnMain(CoreAPI* api, Flight* flight);
void dataToFile(std::string filename, Flight* flight);

// #define RAD2DEG 57.2957795131f
// #define DEG2RAD 0.01745329252f
#define PI 3.14159265359f
#define deg2rad(x) (((x) *PI) / 180.f)
#define rad2deg(x) (((x) *180.f) / PI)
#define MS (1000u)

double constrainAngle(double);

/* @TODO shitty stuff that need to go! */
// Those are declared in main.cpp
extern float measureFromRadioF1;
extern float measureFromRadioF2;
extern float measureFromRadio;

bool endFlag;
bool turnFlag;

extern bool usrpInitializedFlag;

extern int switchState;
extern bool current_freq;

static CoreAPI* api;
static Flight* flight;

std::thread* pRadioThread = nullptr;
std::thread* pDataThread = nullptr;

int main(int argc, char const* argv[])
{
    /* @TODO Check commands before starting*/
    /* Error handling */
    if (!argv[1]) {
        std::cout << "No flight routine provided" << std::endl
                  << "Exiting now." << std::endl;
        return 0;
    }

    /* Initialise drone */
    LinuxSerialDevice* serialDevice =
        new LinuxSerialDevice(UserConfig::deviceName, UserConfig::baudRate);
    CoreAPI* api = new CoreAPI(serialDevice);
    Flight* flight = new Flight(api);

    LinuxThread read(api, 2);
    int setupStatus = setup(serialDevice, api, &read);
    if (setupStatus == -1) {
        std::cout << "This program will exit now." << std::endl;
        return 0;
    }


    /* Monitored Take-off */

    /* @TODO move to define ? */
    /* Timeout for blocking API calls to wait for ack from aircraft. */
    /* Do not set to 0.*/
    int blockingTimeout = 1; // seconds
    ackReturnData takeoffAck = monitoredTakeoff(api, flight, blockingTimeout);
    if (takeoffAck.status != 1) {
        landing(api, flight, blockingTimeout);
        return 0;
    }
    /* @TODO Investigate why this needs to be done AFTER monitoredTakeOff. */
    api->setBroadcastFreqDefaults(1); // Set broadcast Freq Defaults
    unsigned short broadcastAck = api->setBroadcastFreqDefaults(100);
    if(broadcastAck != ACK_SUCCESS){
        std::cout << "Unable to set Broadcast Freqencies" << std::endl;
        std::cout << "This program will exit now." << std::endl;;
        return 0;
    }

    endFlag = 0;

    if (!strcmp(argv[1], "square")) {
        routineSquare(api, flight);
    }
    else if (!strcmp(argv[1], "locate")) {
        pRadioThread = new std::thread(radioMain, 1);
        while (!usrpInitializedFlag) {
        }
        pRadioThread = new std::thread(dataToFile, "out.csv", flight);
        routineLocate(api, flight);
    }
    else {
        std::cout << "Unknown command" << std::endl;
    }

    /* Drone landing */
    landing(api, flight, 10u);
    releaseControl(api);

    /* Stopping all thread */
    endFlag = 1;
    if (pRadioThread != nullptr) {
        pRadioThread->join();
    }
    if (pDataThread != nullptr) {
        pDataThread->join();
    }

    return 0;
}

int routineSquare(CoreAPI* api, Flight* flight)
{
    int altitude = 6;
    int side = 10;
    int waitingTime = 2000; // ms

    /* Square routine */
    moveByPositionOffset(api, flight, 0, 0, altitude, 0);
    moveWithVelocity(api, flight, 0, 0, 0, 0, waitingTime, 0, 0);

    moveByPositionOffset(api, flight, 0, side, 0, 0);
    moveWithVelocity(api, flight, 0, 0, 0, 0, waitingTime, 0, 0);

    moveByPositionOffset(api, flight, side, 0, 0, 0);
    moveWithVelocity(api, flight, 0, 0, 0, 0, waitingTime, 0, 0);

    moveByPositionOffset(api, flight, 0, -side, 0, 0);
    moveWithVelocity(api, flight, 0, 0, 0, 0, waitingTime, 0, 0);

    moveByPositionOffset(api, flight, -side, 0, 0, 0);
    moveWithVelocity(api, flight, 0, 0, 0, 0, waitingTime, 0, 0);
}

int routineLocate(CoreAPI* api, Flight* flight)
{
    float radialDist = 10;

    float currentRadioVal;
    float bestRadioVal = -1;
    
    int currentSwitchState;
    int bestSwitchState;
    
    float bestYaw;

    float offset = 0.0;
    float currentYaw;
    float finalYaw;
    
    int weakestFreqIndex;

    // @TODO check that
    float ANTENNA_OFFSETS[4] = {
        deg2rad(0), deg2rad(90), deg2rad(180), deg2rad(-90)};

    // Need to do the "turn" command in a separate thread because it is
    // blocking
    turnFlag = 1;
    std::thread turnThread(turnMain, api, flight);

    float initialYaw = flight->getYaw();
    float lastYaw = initialYaw;
    
    while (offset <= deg2rad(90.)) {
        currentYaw = constrainAngle(flight->getYaw());

        offset += std::min(
            std::abs(currentYaw - lastYaw),
            std::min(
                std::abs(deg2rad(360u) - lastYaw + currentYaw),
                std::abs(deg2rad(360u) - currentYaw + lastYaw)));
       
        currentRadioVal = measureFromRadio;
        currentSwitchState = switchState;

        if (currentRadioVal > bestRadioVal && currentRadioVal < 10) {
            bestRadioVal = currentRadioVal;
            bestYaw = currentYaw;
            bestSwitchState = currentSwitchState;
        }
        lastYaw = currentYaw;
        usleep(25u * MS);
    }

    turnFlag = 0;
    turnThread.join();

    std::cout << "Tour is done !" << std::endl;
    std::cout << "Best yaw was " << rad2deg(bestYaw)
              << " with a radio value of " << bestRadioVal << std::endl;
    std::cout << "The corresponding switch state was " << bestSwitchState
              << std::endl;

    finalYaw = ANTENNA_OFFSETS[bestSwitchState];

    // Head in the computed direction
    int status1 = moveByPositionOffset(api,
                                       flight,
                                       radialDist * cos(finalYaw),
                                       radialDist * sin(finalYaw),
                                       0,
                                       rad2deg(finalYaw), // WARNING deg !!
                                       30000,
                                       3,
                                       30);
    moveWithVelocity(api, flight, 0, 0, 0, 0, 2000u, 0, 0);

}

void turnMain(CoreAPI* api, Flight* flight)
{
    float angularSpeed = 5;
    while (turnFlag) {
        int status1 =
            moveWithVelocity(api, flight, 0, 0, 0, angularSpeed, 1500, 3, 0.1);
    }
}

int main2(int argc, char const* argv[]){
    char* dummy[0] = {};

    radioMain(1);
}

double constrainAngle(double x){
    x = fmod(x, PI);

    if (x < 0)
        x += PI;
    
    return x;
}

void dataToFile(std::string filename, Flight* flight)
{
    std::ofstream outfile;
    outfile.open(filename, std::ios::out);
    // Change precision of floats to print
    outfile.precision(10);
    outfile.setf(std::ios::fixed);
    outfile.setf(std::ios::showpoint);   
    outfile << "Latitude, Longitude, Altitude, Height, Yaw, SwitchState, RadioValue" << std::endl;

    PositionData pos;
    float yaw;
    int data_amount = 0;
    int data_threshold = 5000;

    while(!endFlag && data_amount < data_threshold)
    {
    data_amount++;
        usleep(25000);
        pos = flight -> getPosition();
        yaw = flight -> getYaw();     // IN RAD !

        outfile <<  pos.latitude    << ","
                <<  pos.longitude   << ","
                <<  pos.altitude    << ","
                <<  pos.height      << ","
                <<  yaw             << ","
                <<  switchState     << ","
                <<  measureFromRadio  << std::endl;
    }
    
    if(data_amount > data_threshold)
    {
       std::cout << "Threshold exceeded, expect incomplete data." << std::endl;
    }

    outfile.close();
}