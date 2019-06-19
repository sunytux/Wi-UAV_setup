/*
 * Header file for radio.cpp
 */

#ifndef UHD_RADIO_HPP
#define UHD_RADIO_HPP

/* DJI OSDK Library Headers*/
#include <DJI_Flight.h>

/* UHD Library Headers */
#include <uhd/usrp/multi_usrp.hpp>

#include "config.hpp"

/**
 * @brief Single radio measurement.
 */
typedef struct {
    int user = 0;
    float average = 0;
    int switchState = 0;
} Radio_data_s;

/**
 * @brief Radio measurement Snapshot.
 */
typedef struct {
	int user = 0;
    float rss[N_ANTENNAS];
} User_rss_s;

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

/**
 * @brief Returns the last available radio measurement from RadioRXStream1().
 *
 * @return a Radio_data_s struct.
 */
Radio_data_s getCurrentRadioMeasure();

/**
 * @brief Scan for all users frequencies and antennas.
 *
 * @param[in] radioSnapshot
 *            vector of User_rss_s.
 */
void scanAllUsers(std::vector<User_rss_s> &radioSnapshot);

Radio_data_s getRadioMeasure();

#endif // UHD_RADIO_HPP
