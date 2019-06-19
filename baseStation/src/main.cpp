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
#include "LinuxFlight.h" // @TODO drop this ?
#include "LinuxSerialDevice.h"
#include "LinuxSetup.h"
#include "ReadUserConfig.h"

/* DJI OSDK Library Headers*/
#include <DJI_Flight.h>

/* @TODO Manage includes */
#include "radio.hpp"
#include "flightUtils.hpp"
#include "utils.hpp"
#include "config.hpp"

using namespace DJI::onboardSDK;

/* @TODO move that */
/* Prototype */
int routineSquare(CoreAPI*, Flight*);
int routineLocate(CoreAPI*, Flight*);
float getAoA_byTurning();

/**
 * @brief Thread function for making the drone rotate.
 *
 *  The given DJI command for yaw velocity control is blocking and need to be
 * executed inside a Thread.
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
void turnAndMoveForward(CoreAPI* api,
                        Flight* flight,
                        float absoluteYaw,
                        float stepInM);

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

float ANTENNA_OFFSETS[4] = {OFFSET_RF1, OFFSET_RF2, OFFSET_RF3, OFFSET_RF4};

int main(int argc, char const* argv[])
{
    /* @TODO Check commands before starting */
    /* Error handling */
    if (!argv[1]) {
        std::cout << "No flight routine provided" << std::endl
                  << "Exiting now." << std::endl;
        return ERROR_STATUS;
    }

    /* Start all threads */
    startAllThreads();

#ifdef ENABLE_DRONE
    /* Take-off */
    int takeoffStatus = initDroneAndTakeoff();
    if (takeoffStatus != SUCCESS_STATUS) {
        return ERROR_STATUS;
    }

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
#else
    std::vector<User_rss_s> usersRss(N_USERS);
    while (true) {

        scanAllUsers(usersRss);
        printf("user: %d - ant: %d - rss: %f\n", usersRss[0].user, 0, usersRss[0].rss[0]);
        printf("user: %d - ant: %d - rss: %f\n", usersRss[0].user, 1, usersRss[0].rss[1]);
        printf("user: %d - ant: %d - rss: %f\n", usersRss[0].user, 2, usersRss[0].rss[2]);
        printf("user: %d - ant: %d - rss: %f\n", usersRss[0].user, 3, usersRss[0].rss[3]);

        printf("user: %d - ant: %d - rss: %f\n", usersRss[1].user, 0, usersRss[1].rss[0]);
        printf("user: %d - ant: %d - rss: %f\n", usersRss[1].user, 1, usersRss[1].rss[1]);
        printf("user: %d - ant: %d - rss: %f\n", usersRss[1].user, 2, usersRss[1].rss[2]);
        printf("user: %d - ant: %d - rss: %f\n", usersRss[1].user, 3, usersRss[1].rss[3]);
        printf("\n\n");
        
        usleep(2 * S);
    }
#endif // ENABLE_DRONE

    /* Stopping all threads */
    stopAllThreads();

    return SUCCESS_STATUS;
}

int initDroneAndTakeoff()
{
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
        std::cout << "Unable to set Broadcast Frequencies" << std::endl;
        std::cout << "Exiting now." << std::endl;
        return ERROR_STATUS;
    }

    return SUCCESS_STATUS;
}

void landDroneAndRealeControl()
{
    landing(api, flight, 10u);
    releaseControl(api);
}

void turnAndMoveForward(CoreAPI* api,
                        Flight* flight,
                        float absoluteYaw, /* @TODO is it absolute ? */
                        float stepInM)
{
    int ROTATION_TIMEOUT = 10000;
    int MOVE_TIMEOUT = 30000;
    int STABILIZATION_TIMEOUT = 3000;

    float YAW_THRESHOLD_DEG = 3.0f;
    float POS_THRESHOLD = 30.0f; /* in cm */
    float YAW_VEL_THRESHOLD = 2.0f; /* in deg/s */
    float POS_VEL_THRESHOLD = 0.3f; /* in m/s */

    /* Rotation */
    std::cout << "Rotation...";
    moveByPositionOffset_modified(api,
                                  flight,
                                  0.f,
                                  0.f,
                                  0.f,
                                  rad2deg(absoluteYaw),
                                  ROTATION_TIMEOUT,
                                  YAW_THRESHOLD_DEG,
                                  POS_THRESHOLD);
    std::cout << "ok" << std::endl;

    /* Move forward */
    std::cout << "Fly forward...";
    moveByPositionOffset_modified(api,
                                  flight,
                                  stepInM * cos(absoluteYaw),
                                  stepInM * sin(absoluteYaw),
                                  0.f,
                                  rad2deg(absoluteYaw),
                                  MOVE_TIMEOUT,
                                  YAW_THRESHOLD_DEG,
                                  POS_THRESHOLD);
    std::cout << "ok" << std::endl;

    /* Stabilization */
    std::cout << "Stabilization...";
    moveWithVelocity_modified(api,
                              flight,
                              0,
                              0,
                              0,
                              0,
                              STABILIZATION_TIMEOUT,
                              YAW_VEL_THRESHOLD,
                              POS_VEL_THRESHOLD);
    std::cout << "ok" << std::endl;
}

/***********************************************************************
 * Threads
 **********************************************************************/
void startAllThreads()
{
    endFlag = 0;
#ifdef ENABLE_USRP
    /* @TODO remove radioScanning config arg, it's bullshit */
    pRadioScanningThread = new std::thread(radioScanning_thread, 1);
    while (!usrpInitializedFlag) {
    }
#endif // ENABLE_USRP

#ifdef ENABLE_DATA
    pDataToFileThread = new std::thread(dataToFile_thread, flight);
#endif // ENABLE_DATA

#ifdef ENABLE_SAFETY
    pSafetyMonitorThread = new std::thread(safetyMonitor_thread, api, flight);
#endif // ENABLE_SAFETY    
}

void stopAllThreads()
{
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
    int timeout = 3000; /* @TODO change back to 3000 ? */
    float angularSpeed = 5.0f; /* @TODO change back to 5 */
    
    float YAW_VEL_THRESHOLD = 2.0f; /* in deg/s */
    float POS_VEL_THRESHOLD = 0.3f; /* in m/s */

    while (turnFlag) {
        moveWithVelocity_modified(api,
                                  flight,
                                  0,
                                  0,
                                  0,
                                  angularSpeed,
                                  timeout,
                                  YAW_VEL_THRESHOLD,
                                  POS_VEL_THRESHOLD);
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
    outfile << "Latitude, Longitude, Altitude, Height, Yaw, User, SwitchState, RadioValue" << std::endl;

    PositionData pos;
    float yaw;
    int data_amount = 0;
    int data_threshold = 5000;

    while(!endFlag && data_amount < data_threshold)
    {
    data_amount++;
        usleep(DATA_THREAD_PERIOD * MS);
        pos = flight -> getPosition();
        yaw = flight -> getYaw();     // IN RAD !

        outfile <<  pos.latitude             << ","
                <<  pos.longitude            << ","
                <<  pos.altitude             << ","
                <<  pos.height               << ","
                <<  yaw                      << ","
                <<  radioMeasure.user        << ","
                <<  radioMeasure.switchState << ","
                <<  radioMeasure.average     << std::endl;
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
        // int status2 = RadioRXStream2();
        break;
    }
    }
}

void safetyMonitor_thread(CoreAPI* api, Flight* flight)
{
    while (!endFlag) {
        usleep(SAFETY_THREAD_PERIOD * MS);
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
    moveByPositionOffset_modified(api, flight, 0, 0, altitude, 0);
    moveWithVelocity_modified(api, flight, 0, 0, 0, 0, waitingTime, 0, 0);

    moveByPositionOffset_modified(api, flight, 0, side, 0, 0);
    moveWithVelocity_modified(api, flight, 0, 0, 0, 0, waitingTime, 0, 0);

    moveByPositionOffset_modified(api, flight, side, 0, 0, 0);
    moveWithVelocity_modified(api, flight, 0, 0, 0, 0, waitingTime, 0, 0);

    moveByPositionOffset_modified(api, flight, 0, -side, 0, 0);
    moveWithVelocity_modified(api, flight, 0, 0, 0, 0, waitingTime, 0, 0);

    moveByPositionOffset_modified(api, flight, -side, 0, 0, 0);
    moveWithVelocity_modified(api, flight, 0, 0, 0, 0, waitingTime, 0, 0);
}

int routineLocate(CoreAPI* api, Flight* flight)
{
    float STEP = 10u;  /* in m */

    while(true){

        float absoluteFinalYaw = getAoA_byTurning();

        std::cout << "Going in direction: " << rad2deg(absoluteFinalYaw) << std::endl;
        std::cout << "Approved ? (y/n): ";
        char answer;
        std::cin >> answer;
        
        if (answer == 'y') {
            turnAndMoveForward(api, flight, absoluteFinalYaw, STEP);
        }
        else {
            break;
        }
    }
}

float getAoA_byTurning()
{

    float initialYaw = normalizedAngle(flight->getYaw());
    float currentYaw = initialYaw;
    float previousYaw = initialYaw;

    Radio_data_s bestRadio;

    float bestYaw;

    float offset = 0.0;

    turnFlag = 1;
    std::thread turnThread(droneRotation_thread, api, flight);

    while (offset <= std::abs(deg2rad(90.f))) {
        currentYaw = normalizedAngle(flight->getYaw());
        offset += angularSubstraction(currentYaw, previousYaw);

        Radio_data_s radioMeasure_cpy = radioMeasure;

        if (radioMeasure_cpy.average > bestRadio.average
            && radioMeasure_cpy.average < 10) {
            bestRadio = radioMeasure_cpy;
            bestYaw = currentYaw;
        }
        previousYaw = currentYaw;
        usleep(25u * MS);
    }

    turnFlag = 0;
    turnThread.join();

    float absoluteFinalYaw = bestYaw + ANTENNA_OFFSETS[bestRadio.switchState];

    std::cout << "Tour is done !" << std::endl;
    std::cout << "Best absolute yaw was " << rad2deg(bestYaw)
              << " with a radio value of " << bestRadio.average << std::endl;
    std::cout << "The corresponding switch state was " << bestRadio.switchState
              << std::endl
              << std::endl;

    return absoluteFinalYaw;
}

