
#ifndef SWITCH_HPP
#define SWITCH_HPP

// defin GPIOs
#define V1_PIN 7    // Corresponds to R-pi GPIO4
#define V2_PIN 15   // Corresponds to R-pi GPIO22
#define V3_PIN 16   // Corresponds to R-pi PWM3

int initSwitch();
void switchNextAntenna(int*);

#endif // SWITCH_HPP
