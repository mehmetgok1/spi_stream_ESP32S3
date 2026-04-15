#include <SPI.h>
#include <SD.h>
#include <time.h>
#include <cstdio>
#include "memory/memory.h"
#include "Arduino.h"
#include "EEPROM.h"
#include "ble/ble.h"
#include "config/config.h"

String sessionFolder = "";  // Root session folder path (e.g., 20260325_143022)

uint8_t eepromRead(uint8_t address)
{
    return EEPROM.read(address);
}
void eepromWrite(uint8_t address, uint8_t value)
{ 
    EEPROM.write(address, value);
    EEPROM.commit();
}
void initEEPROM()
{
    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("Failed to initialise EEPROM");
        while (true);  // Stop execution
    }
    byte value = eepromRead(deviceRoleAddress);
    Serial.print("Read byte from EEPROM: ");
    Serial.println(value, HEX);
}
void initSD(){

  Serial.print("Mounting SD Card... ");

  if (!SD.begin(SD_CS,SPI, 40000000)) {
    for(int i=0; i<5; i++){
        Serial.println("Error: Card not found or wiring incorrect.");
        delay(500);
    }
  }
  else
    Serial.println("Success.");
}

// Create timestamped session folder (e.g., /20260325_143022/)
void initSessionFolder() {
  // Generate timestamp: YYYYMMDD_HHMMSS
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  char timestamp[16];
  strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", timeinfo);
  
  sessionFolder = String(timestamp);
  
  Serial.println("Creating session folder: /" + sessionFolder);
  
  // Create main session folder with leading slash for consistency
  bool created = SD.mkdir(("/" + sessionFolder).c_str());
  
  if (created) {
    Serial.println("Session folder created successfully: /" + sessionFolder);
  } else {
    Serial.println("Session folder creation failed or already exists: /" + sessionFolder);
  }
}