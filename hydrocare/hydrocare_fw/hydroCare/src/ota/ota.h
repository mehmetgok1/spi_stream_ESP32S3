#ifndef OTA_H
#define OTA_H
#include <Arduino.h>

//void printFirmwareInfo();
//void manualUpdate();
void printUpdateError();
void performOTAUpdate();
void checkForUpdate();
void connectToWiFi();

extern bool otaUpdateAvailable;
extern String ssid, password, ver;
extern String firmwareUrl;

#endif