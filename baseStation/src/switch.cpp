/* System Headers */
#include <iostream>

/* mraa headers */
#include "mraa/common.hpp"
#include "mraa/gpio.hpp"

#include "switch.hpp"

int switchState = 1;

mraa::Gpio gpio_1(V1_PIN);
mraa::Gpio gpio_2(V2_PIN);
mraa::Gpio gpio_3(V3_PIN);

int initSwitch()
{
    // Initial state: listen to RF1
    mraa::Result status;

    // init V1 pin to LOW
    status = gpio_1.dir(mraa::DIR_OUT); // Set as output
    if (status != mraa::SUCCESS) {
        printError(status);
        return EXIT_FAILURE;
    }
    status = gpio_1.write(0); // Set to low
    if (status != mraa::SUCCESS) {
        printError(status);
        return EXIT_FAILURE;
    }

    // init V2 pin to LOW
    status = gpio_2.dir(mraa::DIR_OUT); // Set as output
    if (status != mraa::SUCCESS) {
        printError(status);
        return EXIT_FAILURE;
    }
    status = gpio_2.write(0); // Set to low
    if (status != mraa::SUCCESS) {
        printError(status);
        return EXIT_FAILURE;
    }

    // init V3 pin to HIGH
    status = gpio_3.dir(mraa::DIR_OUT); // Set as output
    if (status != mraa::SUCCESS) {
        printError(status);
        return EXIT_FAILURE;
    }
    status = gpio_3.write(1); // Set to high
    if (status != mraa::SUCCESS) {
        printError(status);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

/**
 * @brief Change RF switch active input for a given antenna.
 *
 * Antenna are indexed starting from 0 (i.e. RF1 = 0,... RF4 = 3).
 *
 * @param[in]   antenna
 *              antenna ID.
 *
 */
int switchAntenna(int antenna)
{
    /*
     * Truth table of the RF switch:
     * | V1 | V2 | V3 | Active Input
     * |____|____|____|_____________
     * |    |    |    |
     * | Lo | Lo | Lo |   RF4
     * | Lo | Lo | Hi |   RF1
     * | Lo | Hi | Lo |   RF2
     * | Lo | Hi | Hi |   RF3
     * | Hi | Lo | Lo |   RF4
     * | Hi | Lo | Hi |   All Off
     * | Hi | Hi | Hi |   All Off
     * | Hi | Hi | Hi |   Unsupported
     *
     * All control have 100k internal pull down !
     *
     * With V1 always grounded, two pin control is enough -> 2-bit state
     * variable. 0b01 means V2 low, V3 high -> RF1 active 0b11 means V2 high,V3
     * high -> RF3 active, etc.
     */
    bool v2;
    bool v3;

    switch (antenna) {
    case 0: /* RF1 */
        v2 = 0;
        v3 = 1;
        break;
    case 1: /* RF2 */
        v2 = 1;
        v3 = 0;
        break;

    case 2: /* RF3 */
        v2 = 1;
        v3 = 1;
        break;

    case 3: /* RF4 */
        v2 = 0;
        v3 = 0;
        break;
    }
    
    mraa::Result status;

    status = gpio_2.write(v2);
    if (status != mraa::SUCCESS) {
        printError(status);
        return EXIT_FAILURE;
    }

    status = gpio_3.write(v3);
    if (status != mraa::SUCCESS) {
        printError(status);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;

}

void switchNextAntenna()
{
    switchState = (switchState + 1) % 4;
    switchAntenna(switchState);
}