/*
 * Header file for radio.cpp
 */

#ifndef UHD_RADIO_HPP
#define UHD_RADIO_HPP

/* DJI OSDK Library Headers*/
#include <DJI_Flight.h>

/* UHD Library Headers */
#include <uhd/usrp/multi_usrp.hpp>

/**
 * @brief Single radio measurement.
 */
typedef struct {
    float average = 0;
    int switchState;
} Radio_data_s;

/**
 * @brief Initialize USRP.
 *
 * @param[in]   addr
 *              USRP address.
 */
uhd::usrp::multi_usrp::sptr initUsrp(std::string addr);

/**
 * @brief Four antennas, one frequency.
 */
int RadioRXStream1();

// std::vector<float> RadioRXBurst(int argc, char* argv[], float frequency);

#endif // UHD_RADIO_HPP
