#include <Arduino.h>
#include "config/config.h"
#include "communication/communication.h"

#define fw_version  "0.0.14"

void initPins()
{
    pinMode(ledCntrlIR, OUTPUT);
    digitalWrite(ledCntrlIR, HIGH);
    pinMode(SPI_CS, INPUT);
    pinMode(CAM_PWR, OUTPUT);
    digitalWrite(CAM_PWR, HIGH);
    pinMode(ledCntrl, OUTPUT);
    digitalWrite(ledCntrl, HIGH);
    pinMode(Acc_CS, OUTPUT);
    digitalWrite(Acc_CS, HIGH);
    pinMode(AQ_CS, OUTPUT);
    digitalWrite(AQ_CS, HIGH);
    pinMode(Perip_PWR, OUTPUT);
    digitalWrite(Perip_PWR, HIGH);
}

void initPeripherals()
{
  //setCpuFrequencyMhz(80);

  Serial.begin(115200);
  // Set ADC resolution (ESP32 default is 12-bit = 0-4095)
  analogReadResolution(12);

  // Set ADC attenuation for full 0-3.3V range
  analogSetPinAttenuation(AmbLight, ADC_11db);

  int available_PSRAM_size = ESP.getFreePsram();
  Serial.println((String)"PSRAM Size available (bytes): " + available_PSRAM_size);

}