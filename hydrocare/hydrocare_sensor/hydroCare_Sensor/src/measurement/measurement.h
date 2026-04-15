#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <Arduino.h>
#include "config/config.h"
#include <SPI.h>
#include "MLX90641.h"

extern uint16_t ambLight;
extern uint16_t microphone;  // Microphone ADC reading
extern float ax,ay,az;
extern float bme_temp;
extern float bme_hum;
extern float bme_pres;
extern float bme_gas;
extern MLX90641 myIRcam;

// Measurement functions (called on-demand)
void measureAmbLight();
void measureMicrophone();     // Read microphone ADC
void measureIRTemp();
void readAcceleration();
void measureCamera();
void measureBME688();

// Initialization
void initCamera();
void initIMU();
void initIRTemp();
void initBME688();

// I2C/SPI primitives for sensors
void writeRegister(uint8_t reg, uint8_t value);
uint8_t readRegister(uint8_t reg);
void readMultiple(uint8_t startReg, uint8_t *buffer, uint8_t len);

#endif