#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "utils.hpp"

// #define CONFIG_DEPLOY
// #define CONFIG_DEV_DRONE
#define CONFIG_DEV_USRP

/* Antennas */
#define N_ANTENNAS (4u)
#define OFFSET_RF1 deg2rad(180.f)
#define OFFSET_RF2 deg2rad(-90.f)
#define OFFSET_RF3 deg2rad(0.f)
#define OFFSET_RF4 deg2rad(90.f)

/* Users */
#define N_USERS (2u)
#define FREQ_USER1 (2e9)
#define FREQ_USER2 (2.1e9)

/* Radio */
// #define MEASURE_PERIOD (0.05f) /* in seconds */
#define MEASURE_PERIOD (0.2f) /* in seconds */
#define SDR_ANTENNA ("TX/RX")
#define RX_GAIN (0u)               /* in dB */
#define RX_DAC_RATE (100e6 / 16.f) /* rate of the DAC */
#define BANDWIDTH (100e6)
#define SDR_CLOCK_SOURCE ("internal")
#define MAX_RECORDED_SAMPLES (1000u)

/* Safety */
#define VOLICTY_THRESHOLD (3.f) /* in m/s */
#define ALTITUDE_THRESHOLD (4.f) /* in meters */
#define SAFETY_THREAD_PERIOD (100u) /* in ms */

/* Data record */
#define DATA_FILE ("out.csv")
#define DATA_THREAD_PERIOD (MEASURE_PERIOD * MS) /* in ms */

/* Configuration mode */
#ifdef CONFIG_DEPLOY
#define ENABLE_DRONE
#define ENABLE_SAFETY
#define ENABLE_USRP
#define ENABLE_DATA
#endif // CONFIG_DEPLOY

#ifdef CONFIG_DEV_DRONE
#define ENABLE_DRONE
#define ENABLE_SAFETY
#endif // CONFIG_DEV_DRONE

#ifdef CONFIG_DEV_USRP
#define ENABLE_USRP
#endif // CONFIG_DEV_USRP

#endif // CONFIG_HPP