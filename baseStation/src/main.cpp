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
#include "utils.hpp"
#include "config.hpp"

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
void droneRotation_thread(CoreAPI* api, Flight* flight);
void safetyMonitor_thread(CoreAPI* api, Flight* flight);
void dataToFile_thread(Flight* flight);
void radioScanning_thread(int config);
void stopAllThreads();
void startAllThreads();
int initDroneAndTakeoff();
void landDroneAndRealeControl();

/* @TODO shitty stuff that need to go! */
// Those are declared in main.cpp
// extern float measureFromRadioF1;
// extern float measureFromRadioF2;
extern Radio_data_s radioMeasure;
extern int switchState;

bool endFlag;
bool turnFlag;
extern bool usrpInitializedFlag;

/* @TODO make it not global. */ 
/* difficulties when passing those in argument in initDroneAndTakeoff() */
static CoreAPI* api;
static Flight* flight;

std::thread* pRadioScanningThread = nullptr;
std::thread* pDataToFileThread = nullptr;
std::thread* pSafetyMonitorThread = nullptr;

int main(int argc, char const* argv[])
{
    /* @TODO Check commands before starting */
    /* Error handling */
    if (!argv[1]) {
        std::cout << "No flight routine provided" << std::endl
                  << "Exiting now." << std::endl;
        return ERROR_STATUS;
    }

    /* Take-off */
    int takeoffStatus = initDroneAndTakeoff();
    if (takeoffStatus != SUCCESS_STATUS) {
        return ERROR_STATUS;
    }

    /* Start all threads */
    startAllThreads();

    if (!strcmp(argv[1], "square")) {
        routineSquare(api, flight);
    }
    else if (!strcmp(argv[1], "locate")) {
        routineLocate(api, flight);
    }
    else {
        std::cout << "Unknown command" << std::endl;
    }

    /* Drone landing */
    landDroneAndRealeControl();

    /* Stopping all threads */
    stopAllThreads();

    return SUCCESS_STATUS;
}


int initDroneAndTakeoff(){

    /* Initialise drone */
    LinuxSerialDevice* serialDevice =
        new LinuxSerialDevice(UserConfig::deviceName, UserConfig::baudRate);
    api = new CoreAPI(serialDevice);
    flight = new Flight(api);

    LinuxThread read(api, 2);
    int setupStatus = setup(serialDevice, api, &read);
    if (setupStatus == -1) {
        std::cout << "Unable to setup drivers." << std::endl;
        std::cout << "Exiting now." << std::endl;
        return ERROR_STATUS;
    }

    /* Monitored Take-off */
    int blockingTimeout = 10; /* in seconds */
    /* Timeout for blocking API calls to wait for ack from aircraft. */
    /* Do not set to 0.*/
    ackReturnData takeoffAck = monitoredTakeoff(api, flight, blockingTimeout);
    if (takeoffAck.status != 1) {
        std::cout << "Unable to take-off, landing started..." << std::endl;
        landing(api, flight, blockingTimeout);
        std::cout << "Exiting now." << std::endl;
        return ERROR_STATUS;
    }
    /* @TODO Investigate why this needs to be done AFTER monitoredTakeOff. */
    unsigned short broadcastAck =
        api->setBroadcastFreqDefaults(blockingTimeout);
    if (broadcastAck != ACK_SUCCESS) {
        std::cout << "Unable to set Broadcast Freqencies" << std::endl;
        std::cout << "Exiting now." << std::endl;
        return -1;
    }

    return SUCCESS_STATUS;
}

void landDroneAndRealeControl(){
    landing(api, flight, 10u);
    releaseControl(api);
}


/***********************************************************************
 * Threads
 **********************************************************************/
void startAllThreads(){
    endFlag = 0;
    pRadioScanningThread = new std::thread(radioScanning_thread, 1);
    while (!usrpInitializedFlag) {
    }
    pDataToFileThread = new std::thread(dataToFile_thread, flight);
    pSafetyMonitorThread = new std::thread(safetyMonitor_thread, api, flight);
}

void stopAllThreads(){
    endFlag = 1;

    if (pRadioScanningThread != nullptr) {
        pRadioScanningThread->join();
    }

    if (pDataToFileThread != nullptr) {
        pDataToFileThread->join();
    }
    
    if (pSafetyMonitorThread != nullptr) {
        pSafetyMonitorThread->join();
    }
}

void droneRotation_thread(CoreAPI* api, Flight* flight)
{
    float angularSpeed = 5;
    while (turnFlag) {
        int status1 =
            moveWithVelocity(api, flight, 0, 0, 0, angularSpeed, 1500, 3, 0.1);
    }
}

void dataToFile_thread(Flight* flight)
{
    std::ofstream outfile;
    outfile.open(DATA_FILE, std::ios::out);
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
        usleep(DATA_THREAD_PERIOD);
        pos = flight -> getPosition();
        yaw = flight -> getYaw();     // IN RAD !

        outfile <<  pos.latitude    << ","
                <<  pos.longitude   << ","
                <<  pos.altitude    << ","
                <<  pos.height      << ","
                <<  yaw             << ","
                <<  switchState     << ","
                <<  radioMeasure.average  << std::endl;
    }
    
    if(data_amount > data_threshold)
    {
       std::cout << "Threshold exceeded, expect incomplete data." << std::endl;
    }

    outfile.close();
}

void radioScanning_thread(int config)
{
    switch (config) {
    case 1: {
        std::cout << "Going for radio1" << std::endl;
        int status1 = RadioRXStream1();
        break;
    }
    case 2: {
        std::cout << "Going for radio2" << std::endl;
        // int status2 = RadioRXStream2(argc, argv);
        break;
    }
    }
}

void safetyMonitor_thread(CoreAPI* api, Flight* flight)
{
    while (!endFlag) {
        usleep(SAFETY_THREAD_PERIOD);
        VelocityData curVelocity = api->getBroadcastData().v;
        PositionData pos = flight->getPosition();

        if (curVelocity.x > VOLICTY_THRESHOLD
            || curVelocity.y > VOLICTY_THRESHOLD
            || curVelocity.z > VOLICTY_THRESHOLD) {
            std::cout << "Velocity exceeded threshold ! Emergency release of "
                         "the control."
                      << std::endl;
            ackReturnData releaseControlStatus = releaseControl(api);
            endFlag = 1;
        }

        if (pos.height > ALTITUDE_THRESHOLD) {
            std::cout << "Height exceeded threshold ! Emergency release of the "
                         "control."
                      << std::endl;
            ackReturnData releaseControlStatus = releaseControl(api);
            endFlag = 1;
        }
    }
}

/***********************************************************************
 * Flight routines
 **********************************************************************/
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
    float STEP = 10u;  /* in m */
    
    // @TODO check that
    float ANTENNA_OFFSETS[4] = {
        OFFSET_RF1,
        OFFSET_RF2,
        OFFSET_RF3,
        OFFSET_RF4};

    float initialYaw = normalizedAngle(flight->getYaw());
    float currentYaw = initialYaw;
    float previousYaw = initialYaw;
    
    Radio_data_s bestRadio;
    // float bestRadioVal = -1;
    // int bestSwitchState;
    float bestYaw;

    float offset = 0.0;

    // Need to do the "turn" command in a separate thread because it is
    // blocking
    turnFlag = 1;
    std::thread turnThread(droneRotation_thread, api, flight);

    while (offset <= std::abs(deg2rad(90.f))) {
        
        currentYaw = normalizedAngle(flight->getYaw());
        offset += angularSubstraction(currentYaw, previousYaw);

        // float currentRadioVal = radioMeasure.average;
        // int currentSwitchState = radioMeasure.switchState;
        Radio_data_s radioMeasure_cpy = radioMeasure;

        if (radioMeasure_cpy.average > bestRadio.average && radioMeasure_cpy.average < 10) {
            bestRadio = radioMeasure_cpy;
            bestYaw = currentYaw;
        }
        previousYaw = currentYaw;
        usleep(25u * MS);
    }

    turnFlag = 0;
    turnThread.join();
    
    float finalYaw = bestYaw + ANTENNA_OFFSETS[bestRadio.switchState];

    std::cout << "Tour is done !" << std::endl;
    std::cout << "Best yaw was " << rad2deg(bestYaw)
              << " with a radio value of " << bestRadio.average << std::endl;
    std::cout << "The corresponding switch state was " << bestRadio.switchState
              << std::endl;
    std::cout << "Going in direction: " << rad2deg(finalYaw) << std::endl;

    // Head in the computed direction
    int status1 = moveByPositionOffset(api,
                                       flight,
                                       STEP * cos(finalYaw),
                                       STEP * sin(finalYaw),
                                       0.f,
                                       rad2deg(finalYaw), // WARNING deg !!
                                       30000u,
                                       3.f,
                                       30.f);
    moveWithVelocity(api, flight, 0, 0, 0, 0, 2000u, 0, 0);
}

