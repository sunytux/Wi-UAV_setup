#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "utils.hpp"

#define CONFIG_DEPLOY
// #define CONFIG_DEV_DRONE
// #define CONFIG_DEV_USRP

/* Antennas */
#define N_ANTENNAS (4u)
#define OFFSET_RF1 deg2rad(0.f)
#define OFFSET_RF2 deg2rad(90.f)
#define OFFSET_RF3 deg2rad(-90.f)
#define OFFSET_RF4 deg2rad(180.f)

/* Flight */
#define FLIGHT_ALTITUDE (6.f)    /* in meters */
#define LOCALIZATION_STEP (10.f) /* in metes */
#define SQUARE_STEP (5.f)        /* in metes */

#define MOVE_TIMEOUT (20000)         /* in ms */
#define STABILIZATION_TIMEOUT (2000) /* in ms */
#define ROTATION_TIMEOUT (5000)      /* in ms */

#define POSITION_THRESHOLD (30.f) /* in cm */
#define YAW_THRESHOLD (3.f)       /* in deg */

#define POSITION_VELOCITY_THRESHOLD (0.3f) /* in m/s */
#define YAW_VELOCITY_THRESHOLD (2.f)       /* in deg/s */

/* Safety */
#define VOLICTY_THRESHOLD (3.f)     /* in m/s */
#define ALTITUDE_THRESHOLD (10.f)   /* in meters */
#define SAFETY_THREAD_PERIOD (100u) /* in ms */

/* Users */
#define N_USERS (1u)
#define FREQ_USER1 (2e9)
#define FREQ_USER2 (2.1e9)

/* Radio */
#define MEASURE_PERIOD (0.1f) /* in seconds */
#define SDR_ANTENNA ("TX/RX")
#define RX_GAIN (0u)               /* in dB */
#define RX_DAC_RATE (100e6 / 16.f) /* rate of the DAC */
#define BANDWIDTH (100e6)
#define SDR_CLOCK_SOURCE ("internal")
#define MAX_RECORDED_SAMPLES (1000u)

/* Data record */
#define DATA_FILE ("out.csv")
#define DATA_THREAD_PERIOD (MEASURE_PERIOD * MS) /* in ms */
#define DATA_MAX_RECORDED_SAMPLE 18000

/* Configuration mode */
#ifdef CONFIG_DEPLOY
#define ENABLE_DRONE
#define ENABLE_DATA
#define ENABLE_USRP
#endif // CONFIG_DEPLOY

#ifdef CONFIG_DEV_DRONE
#define ENABLE_DRONE
#endif // CONFIG_DEV_DRONE

#ifdef CONFIG_DEV_USRP
#define ENABLE_USRP
#endif // CONFIG_DEV_USRP

#endif // CONFIG_HPP