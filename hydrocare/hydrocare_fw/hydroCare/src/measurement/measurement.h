#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <Arduino.h>
#include "config/config.h"
#include <SPI.h>

#define Batt_Meas_Count     10
#define Batt_Const_X        1
#define Batt_Const_Y        -0.1
#define Batt_VoltDiv_Mult   2
#define DigitalSupply       3.3

#define measurementTime     10000       //in ms

extern float batteryLevel, batteryPercentage, PIRValue, ambLight;
extern uint16_t ambLight_Int;
extern bool chargingStatus, buttonStatus;
extern uint16_t movingDist, staticDist, detectionDist;
extern uint8_t  movingEnergy,  staticEnergy;
extern uint16_t startTime;
extern bool deviceStatus;

void measureBatteryLevel();
void measurePIR();
void measureAmbLight();
void checkUSB();
void checkButton();
void measuremmWave();
void initmmWave();

#endif