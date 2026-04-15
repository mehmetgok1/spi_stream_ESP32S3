#ifndef MEMORY_H
#define MEMORY_H

#include "Arduino.h"

#define EEPROM_SIZE       1        // Size in bytes
#define deviceRoleAddress 0        // Address to read/write

extern String sessionFolder;      // Root session folder (e.g., 20260325_143022)

void initEEPROM();
uint8_t eepromRead(uint8_t address);
void eepromWrite(uint8_t address, uint8_t value);
void initSD();
void initSessionFolder();          // Create timestamped folder

#endif