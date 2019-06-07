/*
 * Header file for radio.cpp
 */

#ifndef UHD_RADIO_HPP
#define UHD_RADIO_HPP

/* DJI OSDK Library Headers*/
#include <DJI_Flight.h>

/* UHD Library Headers */
#include <uhd/usrp/multi_usrp.hpp>

#define RAD2DEG 57.2957795131
#define DEG2RAD 0.01745329252

#define US_TO_S 1000000

/**
 * @brief Initialize USRP.
 *
 * @param[in]   addr
 *              USRP address.
 */
uhd::usrp::multi_usrp::sptr initUsrp(std::string addr);

void dataToFile(std::string filename, DJI::onboardSDK::Flight* flight);

int radioRXStream(int argc, char* argv[]);

/**
 * @brief Four antennas, one frequency.
 */
int radioRXStream1();

std::vector<float> RadioRXBurst(int argc, char* argv[], float frequency);

void radioMain(int config);

#endif // UHD_RADIO_HPP
