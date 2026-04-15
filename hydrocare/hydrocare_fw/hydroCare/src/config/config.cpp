#include <Arduino.h>
#include "config/config.h"
#include <SPI.h>

String fw_version = "0.0.21";

SPISettings sdSPI(16000000, MSBFIRST, SPI_MODE0);

void initPins(){

  pinMode(PIR, INPUT);
  pinMode(Button, INPUT);
  pinMode(mmWave_Out, INPUT);
  pinMode(USB_Voltage, INPUT);
  pinMode(CE_En, OUTPUT);
  digitalWrite(CE_En, LOW);
  pinMode(Batt_EN, OUTPUT);
  digitalWrite(Batt_EN, LOW);
  pinMode(FLASH_CS, OUTPUT);
  digitalWrite(FLASH_CS, HIGH);
  pinMode(Sensor_EN, OUTPUT);
  digitalWrite(Sensor_EN, HIGH);
  Serial.println("[Master] Sensor_EN (GPIO16) set to HIGH");
  pinMode(Perip_EN, OUTPUT);
  digitalWrite(Perip_EN, HIGH);
  Serial.println("[Master] Perip_EN (GPIO19) set to HIGH - Slave should power ON");
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);
}

void initPeripherals(){

  setCpuFrequencyMhz(80);

  Serial.begin(115200);

  SPI.begin(SCK, MISO, MOSI, SD_CS);

  analogReadResolution(12);

  // Set ADC attenuation for full 0-3.3V range
  analogSetPinAttenuation(AmbLight, ADC_11db);
}
