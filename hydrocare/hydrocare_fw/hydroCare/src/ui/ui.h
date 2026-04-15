#ifndef LED_H
#define LED_H

#include <Arduino.h>
#include "measurement/measurement.h"

extern uint8_t red, green, blue;

void initLed();
void uiInit();
void uiChargingScenario();
void setLED();
void uiOTAStarted();

#endif