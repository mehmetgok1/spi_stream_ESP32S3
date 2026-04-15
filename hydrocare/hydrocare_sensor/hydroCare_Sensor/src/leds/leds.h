#ifndef LEDS_H
#define LEDS_H

#include <Arduino.h>

void powerLED(uint16_t brightness);
void powerLEDInit();
void IRLED(bool status);

#endif