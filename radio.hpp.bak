/*
 * Header file for radio.cpp
 */ 

#ifndef UHD_RADIO_HPP
#define UHD_RADIO_HPP

#include <iostream>
#include <fstream>

//DJI Linux Application Headers
#include "LinuxSerialDevice.h"
#include "LinuxThread.h"
#include "LinuxSetup.h"
#include "LinuxCleanup.h"
#include "ReadUserConfig.h"
#include "LinuxMobile.h"
#include "LinuxFlight.h"
#include "LinuxInteractive.h"
#include "LinuxWaypoint.h"
#include "LinuxCamera.h"

//DJI OSDK Library Headers
#include <DJI_Follow.h>
#include <DJI_Flight.h>
#include <DJI_Version.h>
#include <DJI_WayPoint.h>

#define RAD2DEG 57.2957795131
#define DEG2RAD 0.01745329252

//UHD Library Headers
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <complex>

#define US_TO_S		1000000

void testSwitch();

int initSwitchGPIOControl();

int toggleGPIOs(bool V2_status, bool V3_status);

void switchNextAntenna(int * currentState);

int uhd_init_usrp(int argc, char *argv[]);

void dataToFile(std::string filename, Flight* flight);

int radioRXStream(int argc, char *argv[]);


int radioRXStream1(int argc, char *argv[]);
// Config 1: only 1 antenna

std::vector<float> RadioRXBurst(int argc, char *argv[], float frequency);

int radioMain(int argc, char *argv[], int config);

#endif //UHD_RADIO_HPP
