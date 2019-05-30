/*! @file LinuxInteractive.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Barebones interactive UI for executing Onboard SDK commands.
 *  Calls functions from the new Linux example based on user input.
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#include "LinuxInteractive.h"

#define RAD2DEG 57.2957795131

using namespace std;

char userInput()
{
  cout << endl;
  cout << "|------------------DJI Onboard SDK Interactive Sample------------|" << endl;
  cout << "|                                                                |" << endl;
  cout << "| Available commands:                                            |" << endl;
  cout << "| [1] 360deg rotation                                            |" << endl;
  cout << "| [2] 1.5m ahead                                                 |" << endl;
  cout << "| [3] 90deg rotation                                             |" << endl;
  cout << "| [4] 1m higher                                                  |" << endl;
  cout << "| [a] Request Control                                            |" << endl;
  cout << "| [b] Release Control                                            |" << endl;
  cout << "| [c] Arm the Drone                                              |" << endl;
  cout << "| [d] Disarm the Drone                                           |" << endl;
  cout << "| [e] Takeoff                                                    |" << endl;
  cout << "| [f] Waypoint Sample                                            |" << endl;
  cout << "| [g] Position Control Sample: Draw Square                       |" << endl;
  cout << "| [h] Landing                                                    |" << endl;
  cout << "| [i] Go Home                                                    |" << endl;
  cout << "| [j] Set Gimbal Angle                                           |" << endl;
  cout << "| [k] Set Gimbal Speed                                           |" << endl;
  cout << "| [l] Take Picture                                               |" << endl;
  cout << "| [m] Take Video                                                 |" << endl;
  cout << "| [n] Exit this sample                                           |" << endl;
  cout << "|                                                                |" << endl;
  cout << "|------------------DJI Onboard SDK Interactive Sample------------|" << endl;
#ifdef LIDAR_LOGGING
  cout << "                                                                  " << endl;
  cout << "                                                                  " << endl;
  cout << "|------------------LiDAR Logging Sample--------------------------|" << endl;
  cout << "|                                                                |" << endl;
  cout << "| [o] Start LiDAR Logging in pcap and LAS format                 |" << endl;
  cout << "| You would need a Velodyne PUCK or Simulator to run this sample |" << endl;
  cout << "|                                                                |" << endl;
  cout << "| [p] Stop LiDAR Logging                                         |" << endl;
  cout << "|                                                                |" << endl;
  cout << "|------------------LiDAR Logging Sample--------------------------|" << endl;
#endif
#ifdef USE_PRECISION_MISSIONS
  cout << "                                                                  " << endl;
  cout << "                                                                  " << endl;
  cout << "|------------------Precision Trajectories------------------------|" << endl;
  cout << "|                                                                |" << endl;
  cout << "| [z] Precision Mission Plan: Execute a pre-planned spiral       |" << endl;
  cout << "|                                                                |" << endl;
  cout << "|------------------Precision Trajectories------------------------|" << endl;
#endif
  char inputChar;
  cin >> inputChar;
  return inputChar;
}

void interactiveSpin(CoreAPI* api, Flight* flight, WayPoint* waypointObj, Camera* camera, std::string pathToSpiral, std::string paramTuningFile)
{
  bool userExitCommand = false;

  ackReturnData takeControlStatus;
  ackReturnData releaseControlStatus;
  ackReturnData armStatus;
  ackReturnData disArmStatus;
  ackReturnData takeoffStatus;
  ackReturnData landingStatus;
  ackReturnData goHomeStatus;
  int res;
  int drawSqrPosCtrlStatus;
  uint16_t trajectoryStatus;
  Angle orientation;

#ifdef USE_PRECISION_MISSIONS
  //! Instantiate a local frame for trajectory following
  BroadcastData data = api->getBroadcastData();
  Eigen::Vector3d originLLA(data.pos.latitude, data.pos.longitude, data.pos.altitude);
  CartesianFrame localFrame(originLLA);
  TrajectoryFollower* follower;
  Trajectory* trajectory;

  //! Extract the drone version from the UserConfig params
  std::string droneVer;

  if (UserConfig::targetVersion == versionM100_23 || UserConfig::targetVersion == versionM100_31)
    droneVer = "M100";
  else if (UserConfig::targetVersion == versionA3_31)
    droneVer = "A3";
  else {
    // default case - M100
    droneVer = "M100";
  }

  //! Set up the follower using the tuning parameters supplied
  if (!pathToSpiral.empty()) {
    TrajectoryInfrastructure::startStateBroadcast(api);
    follower = TrajectoryInfrastructure::setupFollower(api,
                                                       flight,
                                                       &localFrame,
                                                       camera,
                                                       droneVer,
                                                       paramTuningFile);
  } else {
    follower = NULL;
    std::cout << "You have chosen to enable precision missions at compile time, but to run a precision mission, you need to supply a trajectory as a program argument.\n";
  }
  //! Set up the trajectory using the trajectory parameters supplied
  trajectory = TrajectoryInfrastructure::setupTrajectory(pathToSpiral);
#endif

  while (!userExitCommand)
  {
    char getUserInput = userInput();
    switch (getUserInput)
    {
      case '1':
        res = moveByPositionOffset(api, flight, 0, 0, 0, 359, 1500, 3, 10);
        break;
      case '2':
	orientation = flight -> getYaw();
        res = moveByPositionOffset(api, flight, 1.5, 0, 0, orientation*RAD2DEG, 1500, 3, 10);
        break;
      case '3':
        res = moveByPositionOffset(api, flight, 0, 0, 0, 90, 1500, 3, 10);
        break;
      case '4':
	orientation = flight -> getYaw();
        res = moveByPositionOffset(api, flight, 0, 0, 1, orientation*RAD2DEG, 1500, 3, 10);
        break;
      case 'a':
        takeControlStatus = takeControl(api);
        break;
      case 'b':
        releaseControlStatus = releaseControl(api);
        break;
      case 'c':
        armStatus = arm(flight);
        break;
      case 'd':
        disArmStatus = disArm(flight);
        break;
      case 'e': 
        takeoffStatus = monitoredTakeoff(api, flight);
        break;
      case 'f':
        wayPointMissionExample(api, waypointObj,1);
        break;
      case 'g':
        drawSqrPosCtrlStatus = drawSqrPosCtrlSample(api, flight);
        break;
      case 'h':
        landingStatus = landing(api,flight);
        break;
      case 'i':
        goHomeStatus = goHome(flight);
        break;
      case 'j':
        gimbalAngleControlSample(camera);
        break;
      case 'k':
  	    gimbalSpeedControlSample(camera);
  	    break;
      case 'l':
  	    takePictureControl(camera);
  	    break;
      case 'm':
	      takeVideoControl(camera);
      case 'n':
        userExitCommand = true;
        break;
    #ifdef LIDAR_LOGGING
      case 'o':
        startLiDARlogging();
        break;
      case 'p':
        stopLiDARlogging();
        break;
    #endif
    #ifdef USE_PRECISION_MISSIONS
      case 'z':

        //! Check if the aircraft has taken off. If not, make it do so.
        data = api->getBroadcastData();
        if (data.status < 2) {
          std::cout << "Aircraft has not taken off. Taking off now...\n";
          takeoffStatus = monitoredTakeoff(api, flight, 1);
          if (takeoffStatus.status == -1) {
            break;
          }
        }
        //! Start the trajectory
        if (!pathToSpiral.empty()) {
          TrajectoryInfrastructure::startStateBroadcast(api);
          trajectoryStatus = TrajectoryInfrastructure::executeFromParams(api, &localFrame, originLLA, trajectory, follower);
        }
        else
          std::cout << "You need to supply a parameters file as an argument to the program.\n";
        break;
    #endif
      default:
        std::cout << "Invalid option entered - please enter a letter from the choices in the prompt.\n";
        break;
    }
    usleep(1000000);
  }
  return;
}
