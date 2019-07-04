/* System Headers */
#include <iostream>
#include <string.h>
#include <thread>
#include <unistd.h>
#include <limits>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>

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
int routineTakeoff(CoreAPI*, Flight*);
int routineSquare(CoreAPI*, Flight*);
int routineLocate(CoreAPI*, Flight*, const char* locateMode);
float getAoA_byTurning();
float getAoA_weightedRss(int userIdx, std::ofstream* logFile=nullptr);

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
int initDroneAndTakeoff();
void landDroneAndRealeControl();
void turnAndMoveForward(CoreAPI* api,
                        Flight* flight,
                        float absoluteYaw,
                        float stepInM);

/* @TODO Consider semaphore ? */
bool endFlag = false;
bool turnFlag = false;
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

    /* Start USRP*/
    #ifdef ENABLE_USRP
    /* @TODO remove radioScanning config arg, it's bullshit */
    pRadioScanningThread = new std::thread(radioScanning_thread, 1);
    while (!usrpInitializedFlag) {
    }
    #endif // ENABLE_USRP

    /* Take-off */
    #ifdef ENABLE_DRONE
    int takeoffStatus = initDroneAndTakeoff();
    if (takeoffStatus != SUCCESS_STATUS) {
        std::cout << "Unable to take-off" << std::endl
                  << "Exiting now." << std::endl;
        return ERROR_STATUS;
    }
    #endif // ENABLE_DRONE
    
    /* Start threads */
    #ifdef ENABLE_DATA
    pDataToFileThread = new std::thread(dataToFile_thread, flight);
    #endif // ENABLE_DATA
    #ifdef ENABLE_DRONE
    pSafetyMonitorThread = new std::thread(safetyMonitor_thread, api, flight);
    #endif // ENABLE_DRONE    


    if (!strcmp(argv[1], "take-off")) {
        routineTakeoff(api, flight);
    }if (!strcmp(argv[1], "square")) {
        routineSquare(api, flight);
    }
    else if (!strcmp(argv[1], "locate")) {
        routineLocate(api, flight, argv[2]);
    }
    else if (!strcmp(argv[1], "manual")) {
        while (true) {
            usleep(100 * MS);
        }
    }
    else {
        std::cout << "Unknown command" << std::endl;
    }

    /* Drone landing */
    #ifdef ENABLE_DRONE
    landDroneAndRealeControl();
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

    /* Rotation */
    moveByPositionOffset_modified(api,
                                  flight,
                                  0.f,
                                  0.f,
                                  0.f,
                                  rad2deg(absoluteYaw),
                                  ROTATION_TIMEOUT,
                                  YAW_THRESHOLD,
                                  POSITION_THRESHOLD);

    /* Move forward */
    moveByPositionOffset_modified(api,
                                  flight,
                                  stepInM * cos(absoluteYaw),
                                  stepInM * sin(absoluteYaw),
                                  0.f,
                                  rad2deg(absoluteYaw),
                                  MOVE_TIMEOUT,
                                  YAW_THRESHOLD,
                                  POSITION_THRESHOLD);

    /* Stabilization */
    moveWithVelocity_modified(api,
                              flight,
                              0,
                              0,
                              0,
                              0,
                              STABILIZATION_TIMEOUT,
                              YAW_VELOCITY_THRESHOLD,
                              POSITION_VELOCITY_THRESHOLD);
}

/***********************************************************************
 * Threads
 **********************************************************************/
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
    
    while (turnFlag) {
        moveWithVelocity_modified(api,
                                  flight,
                                  0,
                                  0,
                                  0,
                                  angularSpeed,
                                  timeout,
                                  YAW_VELOCITY_THRESHOLD,
                                  POSITION_VELOCITY_THRESHOLD);
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

    while(!endFlag && data_amount < DATA_MAX_RECORDED_SAMPLE)
    {
        data_amount++;
        Radio_data_s rMeasure = getCurrentRadioMeasure();
        
        #ifdef ENABLE_DRONE
        pos = flight -> getPosition();
        yaw = flight -> getYaw();
        #else
        pos.longitude = 0.f;
        pos.latitude = 0.f;
        pos.height = 0.f;
        yaw = 0.f;
        #endif // ENABLE_DRONE


        outfile <<  pos.latitude             << ","
                <<  pos.longitude            << ","
                <<  pos.altitude             << ","
                <<  pos.height               << ","
                <<  yaw                      << ","
                <<  rMeasure.user        << ","
                <<  rMeasure.switchState << ","
                <<  rMeasure.average     << std::endl;

        usleep(DATA_THREAD_PERIOD * MS);
    }
    
    if(data_amount > DATA_MAX_RECORDED_SAMPLE)
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
    VelocityData curVelocity;
    PositionData pos;

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
int routineTakeoff(CoreAPI* api, Flight* flight){
    moveWithVelocity_modified(api, flight, 0, 0, 0, 0, STABILIZATION_TIMEOUT, 0, 0);
}
int routineSquare(CoreAPI* api, Flight* flight)
{
    float altitudeCmd = FLIGHT_ALTITUDE - flight->getPosition().height;

    moveByPositionOffset_modified(api, flight, 0, 0, altitudeCmd, 0);
    moveWithVelocity_modified(api, flight, 0, 0, 0, 0, STABILIZATION_TIMEOUT, 0, 0);

    moveByPositionOffset_modified(api, flight, 0, SQUARE_STEP, 0, 0);
    moveWithVelocity_modified(api, flight, 0, 0, 0, 0, STABILIZATION_TIMEOUT, 0, 0);

    moveByPositionOffset_modified(api, flight, SQUARE_STEP, 0, 0, 0);
    moveWithVelocity_modified(api, flight, 0, 0, 0, 0, STABILIZATION_TIMEOUT, 0, 0);

    moveByPositionOffset_modified(api, flight, 0, -SQUARE_STEP, 0, 0);
    moveWithVelocity_modified(api, flight, 0, 0, 0, 0, STABILIZATION_TIMEOUT, 0, 0);

    moveByPositionOffset_modified(api, flight, -SQUARE_STEP, 0, 0, 0);
    moveWithVelocity_modified(api, flight, 0, 0, 0, 0, STABILIZATION_TIMEOUT, 0, 0);
}

int routineLocate(CoreAPI* api, Flight* flight, const char* locateMode)
{   
    char answer;
    float altitudeCmd = FLIGHT_ALTITUDE - flight->getPosition().height;

    char* fileName;
    time_t now = time(0);
    tm* now_tm = localtime(&now);
    strftime(fileName, 50, "locate-log_%T.csv", now_tm);
    std::ofstream locOutputFile;
    locOutputFile.open(fileName, std::ios::out);
    locOutputFile.precision(10);
    locOutputFile.setf(std::ios::fixed);
    locOutputFile.setf(std::ios::showpoint);   
    locOutputFile << "Time,Latitude, Longitude, Altitude, Height, Yaw, Estimated AoA, rss0, rss1, rss2, rss3"  << std::endl;

    moveByPositionOffset_modified(api, flight, 0, 0, altitudeCmd, 0);
    moveWithVelocity_modified(api, flight, 0, 0, 0, 0, STABILIZATION_TIMEOUT, 0, 0);

    while(true){
        float absoluteFinalYaw;

        std::cout << "\n\nStart scanning ? (y/n): ";
        std::cin >> answer;
        if (answer != 'y') {
            break;
        }
        
        if (!strcmp(locateMode, "turn")) {
            absoluteFinalYaw = getAoA_byTurning();
        }
        else if (!strcmp(locateMode, "wrss")) {
            absoluteFinalYaw = getAoA_weightedRss(0, &locOutputFile);
        }else{
            std::cout << "Unknown localization mode" << std::endl
                      << "Exiting now." << std::endl;
            break;
        }

        printf("Computed relative AoA; %.0f deg\n",
            rad2deg(angularSubstraction(absoluteFinalYaw, flight->getYaw())));
        printf("Going %.0fm in AoA direction\n", LOCALIZATION_STEP);
        std::cout << "Approved ? (y/n): ";
        std::cin >> answer;
        
        if (answer == 'y') {
            turnAndMoveForward(api, flight, absoluteFinalYaw, LOCALIZATION_STEP);
        }
        else {
            break;
        }
    }
    locOutputFile.close();

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

        Radio_data_s rMeasure = getCurrentRadioMeasure();

        if (rMeasure.average > bestRadio.average
            && rMeasure.average < 10) {
            bestRadio = rMeasure;
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

float getAoA_weightedRss(int userIdx, std::ofstream* logFile){

    std::vector<User_rss_s> usersRss(N_USERS);
    PositionData pos;
    float initialYaw;

    printf("\n----------------------------\n");
    #ifdef ENABLE_DRONE
    pos = flight -> getPosition();
    initialYaw = normalizedAngle(flight->getYaw());
    #else
    pos.latitude = 0.0f;
    pos.longitude = 0.0f;
    pos.altitude = 0.0f;
    pos.height = 0.0f;
    initialYaw = 0.0f;
    #endif // ENABLE_DRONE

    scanAllUsers(usersRss);
    printRss(usersRss[userIdx].rss);

    /* @TODO clean that */
    float rRss0 = usersRss[userIdx].rss[0];
    float rRss1 = usersRss[userIdx].rss[1];
    float rRss2 = usersRss[userIdx].rss[2];
    float rRss3 = usersRss[userIdx].rss[3];

    int idxMax1 = indexOfMaxElement(usersRss[userIdx].rss, N_ANTENNAS);
    float phi1 = ANTENNA_OFFSETS[idxMax1];
    float rss1 = usersRss[userIdx].rss[idxMax1];

    usersRss[userIdx].rss[idxMax1] = MINUS_INF;
    int idxMax2 = indexOfMaxElement(usersRss[userIdx].rss, N_ANTENNAS);
    float phi2 = ANTENNA_OFFSETS[idxMax2];
    float rss2 = usersRss[userIdx].rss[idxMax2];

    if (std::max({phi1, phi2}) - std::min({phi1, phi2}) > deg2rad(180)) {
        if (phi1 > phi2){
            phi2 += deg2rad(360);
        }else{
            phi1 += deg2rad(360);
        }
    }

    float relativeAngle = normalizedAngle((rss1 * phi1 + rss2 * phi2) / (rss1 + rss2));
    float absoluteAngle = normalizedAngle(initialYaw + relativeAngle);

    printf("Relative angle: %.0f deg\n", rad2deg(relativeAngle));
    printf("Absolute angle: %.0f deg\n", rad2deg(absoluteAngle));
    
    printf("----------------------------\n");

   if (logFile != nullptr) {
       *logFile << time(0) << ","
            << pos.latitude << ","
            << pos.longitude << ","
            << pos.altitude << ","
            << pos.height << ","
            << initialYaw << ","
            << absoluteAngle << ","
            << rRss0 << ","
            << rRss1 << ","
            << rRss2 << ","
            << rRss3 << std::endl;
   }

   return absoluteAngle;

}

