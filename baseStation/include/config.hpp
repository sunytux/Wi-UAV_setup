#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "utils.hpp"

#define OFFSET_RF1 deg2rad(180.f)
#define OFFSET_RF2 deg2rad(-90.f)
#define OFFSET_RF3 deg2rad(0.f)
#define OFFSET_RF4 deg2rad(90.f)

/* Radio */
#define MEASURE_PERIOD (0.05f) /* in seconds */
#define CARRIER_FREQ (2e9)
#define SDR_ANTENNA ("TX/RX")
#define RX_GAIN (0u)               /* in dB */
#define RX_DAC_RATE (100e6 / 16.f) /* rate of the DAC */
#define BANDWIDTH (100e6)
#define SDR_CLOCK_SOURCE ("internal")
#define MAX_RECORDED_SAMPLES (1000u)

#endif // CONFIG_HPP