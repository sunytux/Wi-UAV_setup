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

void testSwitch()
{
    initSwitchGPIOControl();
    toggleGPIOs(0, 1);

    while (1) {
        switchNextAntenna(&switchState);
        std::cout << "New switchstate is " << switchState
                  << ", waiting for input to switch" << std::endl;
        char whatever;
        std::cin >> whatever;
    }
}

int initSwitchGPIOControl()
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

int toggleGPIOs(bool V2_status, bool V3_status)
{
    mraa::Result status;

    status = gpio_2.write(V2_status);
    if (status != mraa::SUCCESS) {
        printError(status);
        return EXIT_FAILURE;
    }

    status = gpio_3.write(V3_status);
    if (status != mraa::SUCCESS) {
        printError(status);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

void switchNextAntenna(int* currentState)
{
    /*
    Truth table of the RF switch:
    | V1 | V2 | V3 | Active Input
    |____|____|____|_____________
    |    |    |    |
    | Lo | Lo | Lo |   RF4
    | Lo | Lo | Hi |   RF1
    | Lo | Hi | Lo |   RF2
    | Lo | Hi | Hi |   RF3
    | Hi | Lo | Lo |   RF4
    | Hi | Lo | Hi |   All Off
    | Hi | Hi | Hi |   All Off
    | Hi | Hi | Hi |   Unsupported

    All control have 100k internal pull down !

    With V1 always grounded, two pin control is enough -> 2-bit state variable.
    0b01 means V2 low, V3 high -> RF1 active
    0b11 means V2 high, V3 high -> RF3 active, etc.
    */

    switch (*currentState) {
    // 0b01 = 1; RF1 active, go to RF2
    case 1:
        // std::cout << "Switching from RF1 to RF2" << std::endl;
        toggleGPIOs(1, 0);
        *currentState = 2;
        break;

    // 0b10 = 2; RF2 active, go to RF3
    case 2:
        // std::cout << "Switching from RF2 to RF3" << std::endl;
        toggleGPIOs(1, 1);
        *currentState = 3;
        break;

    // 0b11 = 3; RF3 active, go to RF4
    case 3:
        // std::cout << "Switching from RF3 to RF4" << std::endl;
        toggleGPIOs(0, 0);
        *currentState = 0;
        break;

    // 0b00 = 0; RF4 active, go to RF1
    case 0:
        // std::cout << "Switching from RF4 to RF1" << std::endl;
        toggleGPIOs(0, 1);
        *currentState = 1;
        break;
    }
}