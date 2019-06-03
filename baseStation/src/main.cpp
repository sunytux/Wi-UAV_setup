
/*! @file main.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  New Linux App for DJI Onboard SDK.
 *  Provides a number of convenient abstractions/wrappers around core API calls.
 *
 *  Calls are blocking; the calling thread will sleep until the
 *  call returns. Use Core API calls or another sample if you
 *  absolutely need non-blocking calls.
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

// System Headers
#include <unistd.h>
#include <algorithm>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

// DJI Linux Application Headers
#include "LinuxCamera.h"
#include "LinuxCleanup.h"
#include "LinuxFlight.h"
#include "LinuxInteractive.h"
#include "LinuxMobile.h"
#include "LinuxSerialDevice.h"
#include "LinuxSetup.h"
#include "LinuxThread.h"
#include "LinuxWaypoint.h"
#include "ReadUserConfig.h"

// DJI OSDK Library Headers
#include <DJI_Flight.h>
#include <DJI_Follow.h>
#include <DJI_Version.h>
#include <DJI_WayPoint.h>

// USRP files
#include "radio.hpp"

// Local Mission Planning Suite Headers
//#include <MissionplanHeaders.h>

using namespace std;
using namespace DJI;
using namespace DJI::onboardSDK;

bool endFlag = 0;
bool turnFlag;
float measureFromRadioF1;
float measureFromRadioF2;
float measureFromRadio;
extern int switchState;
extern bool current_freq;

int speedMonitoring(CoreAPI *api, Flight *flight, float32_t velocity_threshold,
                    float32_t height_threshold) {
  while (!endFlag) {
    usleep(100000);
    VelocityData curVelocity = api->getBroadcastData().v;
    PositionData pos = flight->getPosition();

    if (curVelocity.x > velocity_threshold ||
        curVelocity.x > velocity_threshold ||
        curVelocity.x > velocity_threshold) {
      std::cout
          << "Velocity exceeded threshold ! Emergency release of the control."
          << std::endl;
      ackReturnData releaseControlStatus = releaseControl(api);
      endFlag = 1;
    }

    if (pos.height > height_threshold) {
      std::cout
          << "Height exceeded threshold ! Emergency release of the control."
          << std::endl;
      ackReturnData releaseControlStatus = releaseControl(api);
      endFlag = 1;
    }
  }

  return 0;
}

void turnMain(CoreAPI *api, Flight *flight) {
  float angularSpeed = 5;
  while (turnFlag) {
    int status1 =
        moveWithVelocity(api, flight, 0, 0, 0, angularSpeed, 1500, 3, 0.1);
  }
}

//! Main function for the Linux sample. Lightweight. Users can call their own
//! API calls inside the Programmatic Mode else on Line 68.
int main(int argc, char *argv[]) {
  //! Instantiate a serialDevice, an API object, flight and waypoint objects and
  //! a read thread.
  LinuxSerialDevice *serialDevice =
      new LinuxSerialDevice(UserConfig::deviceName, UserConfig::baudRate);
  CoreAPI *api = new CoreAPI(serialDevice);
  Flight *flight = new Flight(api);
  WayPoint *waypointObj = new WayPoint(api);
  Camera *camera = new Camera(api);
  LinuxThread read(api, 2);

  ackReturnData takeControlStatus;
  ackReturnData releaseControlStatus;

  //! Setup
  int setupStatus = setup(serialDevice, api, &read);
  if (setupStatus == -1) {
    std::cout << "This program will exit now. \n";
    return 0;
  }
  //! Set broadcast Freq Defaults
  unsigned short broadcastAck = api->setBroadcastFreqDefaults(1);
  // usleep(500000);

  // Setup USRP threads

  // std::thread radioThread(radioMain, 1, dummy, 2);
  // std::cout << "Waiting for USRP" << std::endl;
  // sleep(7);
  // std::thread dataThread(dataToFile, "out.csv", flight);

  // Setup speed monitoring thread
  float32_t v_threshold = 2;
  float32_t height_threshold = 4;
  // std::thread monitoringThread(speedMonitoring, api, flight, v_threshold,
  // height_threshold);

  //! Mobile mode
  if (argv[1] && !strcmp(argv[1], "-mobile")) {
    std::cout << "Listening to Mobile Commands\n";
    mobileCommandSpin(api, flight, waypointObj, camera, argv, argc);
  }
  //! Interactive Mode
  else if (argv[1] && !strcmp(argv[1], "-interactive")) {
    if (argc > 3)
      interactiveSpin(api, flight, waypointObj, camera, std::string(argv[2]),
                      std::string(argv[3]));
    else if (argc == 3)
      interactiveSpin(api, flight, waypointObj, camera, std::string(argv[2]),
                      std::string(""));
    else
      interactiveSpin(api, flight, waypointObj, camera, std::string(""),
                      std::string(""));
  }

  //! Programmatic Mode - execute everything here without any interactivity.
  //! Useful for automation.
  else if (argv[1] && !strcmp(argv[1], "-programmatic-square")) {
    /*! Set a blocking timeout - this is the timeout for blocking API calls
      to wait for acknowledgements from the aircraft. Do not set to 0.
    !*/
    int blockingTimeout = 1;  // Seconds

    // Flag of end of code for other threads
    endFlag = 0;

    //! Monitored Takeoff
    ackReturnData takeoffStatus =
        monitoredTakeoff(api, flight, blockingTimeout);

    //! Set broadcast Freq Defaults
    unsigned short broadcastAck = api->setBroadcastFreqDefaults(1);

    //! If the aircraft took off, continue to do flight control tasks
    if (takeoffStatus.status == 1) {
      int drawSqrPosCtrlStatus = drawSqrPosCtrlSample(api, flight);

      //! Land
      ackReturnData landingStatus = landing(api, flight, blockingTimeout);
    } else {
      // Try to land directly
      ackReturnData landingStatus = landing(api, flight, blockingTimeout);
    }

    endFlag = 1;
  }

  // Test radio code, 1 antenna
  else if (argv[1] && !strcmp(argv[1], "-programmatic-radio1")) {
    /*! Set a blocking timeout - this is the timeout for blocking API calls
      to wait for acknowledgements from the aircraft. Do not set to 0.
    !*/
    int blockingTimeout = 1;  // Seconds
    float angularSpeed = 5;
    float radialDist = 10;

    // Flag of end of code for other threads
    endFlag = 0;

    //! Monitored Takeoff
    ackReturnData takeoffStatus =
        monitoredTakeoff(api, flight, blockingTimeout);

    //! Set broadcast Freq Defaults
    unsigned short broadcastAck = api->setBroadcastFreqDefaults(1);

    int res = moveByPositionOffset(api, flight, 0, 0, 2, 0, 1500, 3, 10);

    //! If the aircraft took off, continue to do flight control tasks
    if (takeoffStatus.status == 1) {
      std::cout << "Waiting for input before rotation" << std::endl;
      char whatever;
      // std::cin >> whatever;

      takeControlStatus = takeControl(api);

      float currentRadioVal;
      float bestRadioVal = 0.;
      float initialYaw = flight->getYaw();
      initialYaw *= RAD2DEG;
      float offset = 0.;
      float bestYaw;
      float currentYaw;
      float lastYaw;

      // while(offset <= 360. && offset >= 0)
      //{
      // currentYaw = flight -> getYaw();
      // if (currentYaw < 0)
      //{
      // currentYaw += 2*M_PI;
      //}

      // offset += abs(currentYaw*RAD2DEG - lastYaw);

      // std::cout << "initialYaw is " << initialYaw*RAD2DEG << std::endl;
      // std::cout << "currentYaw is " << currentYaw*RAD2DEG << std::endl;

      // currentRadioVal = measureFromRadio;
      // int status1 = moveWithVelocity(api, flight, 0, 0, 0, angularSpeed*2,
      // 10, 3, 0.1);

      // if (currentRadioVal > bestRadioVal)
      //{
      // bestYaw = currentYaw;
      // bestRadioVal = currentRadioVal;
      //}
      // std::cout << "Offset - RadioValue : " << offset << " - " <<
      // currentRadioVal << std::endl;  lastYaw = currentYaw*RAD2DEG;

      ////usleep(5000);
      //}

      std::cout << "Tour is done, best yaw was " << bestYaw * RAD2DEG
                << " with a radio value of " << bestRadioVal << std::endl;
      bestYaw = 0;
      // Head in the computed direction
      int status1 = moveByPositionOffset(api, flight, radialDist * cos(bestYaw),
                                         radialDist * sin(bestYaw), 0, bestYaw,
                                         15000000, 3, 30);

      std::cout << "Thanks for flying with USRP Airlines" << std::endl;

      std::cout << "Waiting for input before landing" << std::endl;
      // std::cin >> whatever;
      //! Land
      ackReturnData landingStatus = landing(api, flight, blockingTimeout);
      releaseControlStatus = releaseControl(api);

    } else {
      // Try to land directly
      ackReturnData landingStatus = landing(api, flight, blockingTimeout);
    }

    endFlag = 1;
  }

  // 4 antennas, with switch
  else if (argv[1] && !strcmp(argv[1], "-programmatic-radio2")) {
    /*! Set a blocking timeout - this is the timeout for blocking API calls
      to wait for acknowledgements from the aircraft. Do not set to 0.
    !*/
    int blockingTimeout = 1;  // Seconds
    float radialDist = 10;

    // Flag of end of code for other threads
    endFlag = 0;

    //! Monitored Takeoff
    ackReturnData takeoffStatus =
        monitoredTakeoff(api, flight, blockingTimeout);

    //! Set broadcast Freq Defaults
    unsigned short broadcastAck = api->setBroadcastFreqDefaults(1);

    //! If the aircraft took off, continue to do flight control tasks
    if (takeoffStatus.status == 1) {
      takeControlStatus = takeControl(api);

      float currentRadioVal;
      std::vector<float> bestRadioVal(2, 0);

      std::vector<float> bestYaw(2, 0);

      std::vector<int> bestSwitchState(2, 0);

      float initialYaw = (flight->getYaw()) + M_PI;
      initialYaw *= RAD2DEG;
      float offset = 0.;
      int currentSwitchState;
      float currentYaw;
      bool currentFreq;
      float finalYaw;
      float lastYaw = initialYaw;
      int weakestFreqIndex;
      std::cout << "initalYaw :" << initialYaw << std::endl;

      // Need to do the "turn" command in a separate thread because it is
      // blocking
      turnFlag = 1;
      std::thread turnThread(turnMain, api, flight);

      while (offset <= 90.) {
        currentYaw = ((flight->getYaw()) + M_PI) * RAD2DEG;
        std::cout << "currentYaw :" << currentYaw << std::endl;
        // std::cout << "Adding " << abs(currentYaw - lastYaw) << std::endl;
        offset += std::min(abs(currentYaw - lastYaw),
                           std::min(abs(360 - lastYaw + currentYaw),
                                    abs(360 - currentYaw + lastYaw)));
        currentRadioVal = measureFromRadio;
        currentSwitchState = switchState;
        currentFreq = current_freq;

        if (currentRadioVal > bestRadioVal[currentFreq] &&
            currentRadioVal < 10) {
          bestRadioVal[currentFreq] = currentRadioVal;
          bestYaw[currentFreq] = currentYaw;
          bestSwitchState[currentFreq] = currentSwitchState;
        }
        std::cout << "Offset - RadioValue : " << offset << " - "
                  << currentRadioVal << std::endl;
        lastYaw = currentYaw;
        usleep(25000);
      }
      turnFlag = 0;
      turnThread.join();

      std::cout << "Tour is done !" << std::endl;
      std::cout << "bestRadioVal :" << bestRadioVal[0] << ", "
                << bestRadioVal[1] << std::endl;
      std::cout << "bestYaw :" << bestYaw[0] << ", " << bestYaw[1] << std::endl;
      std::cout << "Frequency 0: best yaw was " << bestYaw[0]
                << " with a radio value of " << bestRadioVal[0] << std::endl;
      std::cout << "The corresponding switch state was " << bestSwitchState[0]
                << std::endl;

      std::cout << "Frequency 1: best yaw was " << bestYaw[1]
                << " with a radio value of " << bestRadioVal[1] << std::endl;
      std::cout << "The corresponding switch state was " << bestSwitchState[1]
                << std::endl;

      weakestFreqIndex = std::distance(
          bestRadioVal.begin(),
          std::min_element(bestRadioVal.begin(), bestRadioVal.end()));
      finalYaw = bestYaw[weakestFreqIndex];

      // Adjust yaw based on antenna position
      if (bestSwitchState[weakestFreqIndex] == 0) {
        // 0 is RF4, 90 deg offset
        finalYaw += M_PI / 2;
      } else if (bestSwitchState[weakestFreqIndex] == 1) {
        // 1 is RF1, 180 deg offset
        finalYaw += M_PI;
      } else if (bestSwitchState[weakestFreqIndex] == 2) {
        // 2 is RF2, 90 deg offset
        finalYaw -= M_PI / 2;
      }
      // Else: RF3, no offest needed

      // Head in the computed direction
      int status1 = moveByPositionOffset(
          api, flight, radialDist * cos(DEG2RAD * finalYaw),
          radialDist * sin(DEG2RAD * finalYaw), 0, finalYaw, 15000, 3, 30);

      std::cout << "Thanks for flying with USRP Airlines" << std::endl;

      //! Land
      ackReturnData landingStatus = landing(api, flight, blockingTimeout);
      releaseControlStatus = releaseControl(api);

    } else {
      // Try to land directly
      ackReturnData landingStatus = landing(api, flight, blockingTimeout);
    }

    endFlag = 1;
  }

  //! No mode specified or invalid mode specified"
  else {
    std::cout << "No argument given. Entering listen mode." << std::endl;
    std::vector<float> bestMeasurementF1(3, 0);
    // contains radio, yaw, switch
    std::vector<float> bestMeasurement2F1(3, 0);
    std::vector<float> bestMeasurementF2(3, 0);
    std::vector<float> bestMeasurement2F2(3, 0);

    float currentYaw;
    float currentRadioVal;
    float currentSwitchState;
    bool currentFreq;

    char *dummy[0] = {};
    std::vector<float> measurementsF1 = RadioRXBurst(1, dummy, 2e9);
    std::vector<float> measurementsF2 = RadioRXBurst(1, dummy, 2100e6);

    float bestVal = 0.;
    float secondBestVal = 0.;

    for (int i = 0; i < measurementsF1.size(); ++i) {
      currentYaw = ((flight->getYaw()) + M_PI) * RAD2DEG;
      if (measurementsF1[i] > bestVal) {
        std::cout << "New best value for F1" << std::endl;
        bestMeasurementF1[0] = measurementsF1[i];
        bestMeasurementF1[1] = currentYaw;
        bestMeasurementF1[2] = i;
        bestVal = measurementsF1[i];
      } else if (measurementsF1[i] > secondBestVal) {
        std::cout << "New second best value for F1" << std::endl;
        bestMeasurement2F1[0] = measurementsF1[i];
        bestMeasurement2F1[1] = currentYaw;
        bestMeasurement2F1[2] = i;
        secondBestVal = measurementsF1[i];
      }
    }

    bestVal = 0.;
    secondBestVal = 0.;

    for (int i = 0; i < measurementsF2.size(); ++i) {
      currentYaw = ((flight->getYaw()) + M_PI) * RAD2DEG;
      if (measurementsF2[i] > bestVal) {
        std::cout << "New best value for F2" << std::endl;
        bestMeasurementF2[0] = measurementsF2[i];
        bestMeasurementF2[1] = currentYaw;
        bestMeasurementF2[2] = i;
        bestVal = measurementsF2[i];
      } else if (measurementsF2[i] > secondBestVal) {
        std::cout << "New second best value for F2" << std::endl;
        bestMeasurement2F2[0] = measurementsF2[i];
        bestMeasurement2F2[1] = currentYaw;
        bestMeasurement2F2[2] = i;
        secondBestVal = measurementsF2[i];
      }
    }

    std::cout << "Displaying measurementsF1." << std::endl;
    for (int i = 0; i < measurementsF1.size(); ++i) {
      std::cout << measurementsF1[i] << std::endl;
    }

    std::cout << std::endl
              << "Here is the content of bestMeasurementF1:" << std::endl;
    std::cout << bestMeasurementF1[0] << " - " << bestMeasurementF1[1] << " - "
              << bestMeasurementF1[2] << std::endl;

    std::cout << std::endl
              << "Here is the content of bestMeasurement2F1:" << std::endl;
    std::cout << bestMeasurement2F1[0] << " - " << bestMeasurement2F1[1]
              << " - " << bestMeasurement2F1[2] << std::endl
              << std::endl;

    std::cout << std::endl
              << "Here is the content of bestMeasurementF2:" << std::endl;
    std::cout << bestMeasurementF2[0] << " - " << bestMeasurementF2[1] << " - "
              << bestMeasurementF2[2] << std::endl;

    std::cout << std::endl
              << "Here is the content of bestMeasurement2F2:" << std::endl;
    std::cout << bestMeasurement2F2[0] << " - " << bestMeasurement2F2[1]
              << " - " << bestMeasurement2F2[2] << std::endl
              << std::endl;

    // Just follow strongest antenna
    //    float finalYaw1;
    //    if (bestMeasurementF1[2] == 0)
    // {
    //            // Best measurement from RF3, no offset needed
    //            finalYaw1 = bestMeasurementF1[1];
    //    }

    //    else if (bestMeasurementF1[2] == 1)
    // {
    //            // Best measurement from RF4, 90deg offset
    //            finalYaw1 = (int(bestMeasurementF1[1]) + 90) % 360;
    //    }
    // else if (bestMeasurementF1[2] == 2)
    // {
    //            // Best measurement from RF1, 180deg offset
    //            finalYaw1 = (int(bestMeasurementF1[1]) + 180) % 360;
    //    }
    // else if (bestMeasurementF1[2] == 3)
    //    {
    //            // Best measurement from RF2, 270deg offset
    //            finalYaw1 = (int(bestMeasurementF1[1]) + 270) % 360;
    //    }

    //    std::cout << std::endl << "The final yaw in simple case is " <<
    //    finalYaw1 << std::endl;

    std::vector<int> offsets = {0, 90, 180, -90};

    // Do a weighted sum of antenna readings
    float finalYaw2;

    // Select the weakest signal
    float F1strength = bestMeasurementF1[0] + bestMeasurement2F1[0];
    float F2strength = bestMeasurementF2[0] + bestMeasurement2F2[0];

    if (F1strength > F2strength) {
      finalYaw2 = int(bestMeasurementF2[1] +
                      (bestMeasurementF2[0] * offsets[bestMeasurementF2[2]] +
                       bestMeasurement2F2[0] * offsets[bestMeasurement2F2[2]]) /
                          (bestMeasurementF2[0] + bestMeasurement2F2[0])) %
                  360;
      std::cout << "F2 was weaker ! Going for yaw " << finalYaw2 << std::endl;
    } else {
      finalYaw2 = int(bestMeasurementF1[1] +
                      (bestMeasurementF1[0] * offsets[bestMeasurementF1[2]] +
                       bestMeasurement2F1[0] * offsets[bestMeasurement2F1[2]]) /
                          (bestMeasurementF1[0] + bestMeasurement2F1[0])) %
                  360;
      std::cout << "F1 was weaker ! Going for yaw " << finalYaw2 << std::endl;
    }

    // std::cout << "Press any key + ENTER to join threads." << std::endl;
    // char whatever;
    // std::cin >> whatever;
    endFlag = 1;

    char write;
    std::cout << "Want to save this decision point to file ? (y for yes)"
              << std::endl;
    std::cin >> write;
    if (write == 'y') {
      std::ofstream outfile;
      outfile.open("decision.csv", std::ios_base::app);
      outfile.precision(16);
      outfile.setf(std::ios::fixed);
      outfile.setf(std::ios::showpoint);
      // Structure:
      // bestMeas1:4 (12 cols) bc 2 freqs, finalYaw2, latitude, longitude

      PositionData pos = flight->getPosition();
      outfile << bestMeasurementF1[0] << "," << bestMeasurementF1[1] << ","
              << bestMeasurementF1[2] << "," << bestMeasurement2F1[0] << ","
              << bestMeasurement2F1[1] << "," << bestMeasurement2F1[2] << ","
              << bestMeasurementF2[0] << "," << bestMeasurementF2[1] << ","
              << bestMeasurementF2[2] << "," << bestMeasurement2F2[0] << ","
              << bestMeasurement2F2[1] << "," << bestMeasurement2F2[2] << ","
              << finalYaw2 << "," << pos.latitude << "," << pos.longitude
              << std::endl;
    }
  }

  //! Cleanup
  std::cout << "Waiting for dataThread to join" << std::endl;
  // dataThread.join();
  std::cout << "Waiting for radioThread to join" << std::endl;
  // radioThread.join();
  std::cout << "Waiting for monitoringThread to join" << std::endl;
  // monitoringThread.join();

  int cleanupStatus = cleanup(serialDevice, api, flight, &read);
  if (cleanupStatus == -1) {
    std::cout << "Unable to cleanly destroy OSDK infrastructure. There may be "
                 "residual objects in the system memory.\n";
    return 0;
  }
  std::cout << "Program exited successfully." << std::endl;

  return 0;
}
