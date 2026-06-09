#include <Arduino.h>
#include <SD.h>
#include "config/config.h"
#include "ui/ui.h"
#include "measurement/measurement.h"
#include "ble/ble.h"
#include "timer/timer.h"
#include "memory/memory.h"
#include "ota/ota.h"
#include "communication/communication.h"
#include "wifi_stream/wifi_stream.h"

bool deviceStatus = false; // false = stopped, true = logging
bool sessionInitialized = false; // true = session folders and files ready
bool wifi_connect = false; // Flag to trigger WiFi connection in main loop
bool stream_wifi = false; // Flag to trigger WiFi streaming after session ends
uint16_t downsampled16x16[256];  // Shared with BLE for transmission
uint16_t irFrame16x12[192];      // Shared with BLE for transmission
bool debug_infos = false; // Set to true to enable detailed debug prints
// ==================== COMBINED DATA PACKET ====================
#pragma pack(1)
typedef struct {
  float batteryLevel;             
  float batteryPercentage;        
  float ambLight;                 
  uint16_t ambLight_Int;          
  float PIRValue;                 
  uint16_t movingDist;            
  uint8_t movingEnergy;           
  uint16_t staticDist;            
  uint8_t staticEnergy;           
  uint16_t detectionDist;         
  SensorDataPacket slaveData;     
} CombinedDataPacket;
#pragma pack()

TaskHandle_t sdTaskHandle = NULL;
SemaphoreHandle_t sdLogMutex = NULL;
CombinedDataPacket globalLogPacket; // ONE global buffer replaces the 3 separate 24KB buffers!
uint32_t packetsLogged = 0;

// Creates new binary file every 50 packets (e.g., part_0.bin, part_50.bin, part_100.bin)
void sdCardLoggingTask(void *parameter) {
  Serial.println("[SD-TASK] SD logging task started (BINARY mode - 50-packet rotation, safe Open/Close)");

  while(1) {

    // Wait for the main loop to signal that new data is ready
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100))) {
      uint32_t taskStart = millis();  // Start timing this packet
      
      xSemaphoreTake(sdLogMutex, portMAX_DELAY);
      if (globalLogPacket.slaveData.temperature == 0.0 || globalLogPacket.slaveData.sequence == 0xFFFF) {
        Serial.printf("[SD-TASK-ERR] Dropped invalid packet (seq=%u, temp=%.1f)\n", 
                     globalLogPacket.slaveData.sequence, globalLogPacket.slaveData.temperature);
        xSemaphoreGive(sdLogMutex);
        continue;  
      }
      
      // Calculate which part file to use (every 50 packets = new file)
      uint32_t fileIndex = (packetsLogged / 50) * 50;
      char dataFile[128];
      snprintf(dataFile, sizeof(dataFile), "/%s/%s_part_%u.bin", sessionFolder.c_str(), sessionFolder.c_str(), fileIndex);
      
      // Open file with FILE_APPEND on every packet for 100% FAT safety
      File df = SD.open(dataFile, FILE_APPEND);
      if (!df) {
        Serial.printf("[SD-TASK-WARN] FILE_APPEND failed, attempting FILE_WRITE create for: %s\n", dataFile);
        df = SD.open(dataFile, FILE_WRITE);
        if (!df) {
          Serial.printf("[SD-TASK-ERR] Failed to create binary file: %s\n", dataFile);
          xSemaphoreGive(sdLogMutex);
          continue;
        }
      }
      
      uint32_t writeStart = micros();  // Microsecond precision for SD timing
      
      // Chunk the 24KB write into 4KB blocks to prevent ESP32 SD buffer saturation
      size_t totalWritten = 0;
      const uint8_t* pData = (const uint8_t*)&globalLogPacket;
      size_t remaining = sizeof(CombinedDataPacket);
      
      while (remaining > 0) {
        size_t toWrite = (remaining > 4096) ? 4096 : remaining;
        size_t writtenChunk = 0;
        int retries = 0;
        
        // Robust Retry Mechanism: Wait for SD card internal sector erases (up to 300ms)
        while (writtenChunk == 0 && retries < 15) {
          writtenChunk = df.write(pData, toWrite);
          if (writtenChunk == 0) {
            retries++;
            delay(20); // Give the SD card 20ms to breathe before retrying
          }
        }
        
        if (writtenChunk == 0) break; // Real failure after 300ms of patience
        
        totalWritten += writtenChunk;
        pData += writtenChunk;
        remaining -= writtenChunk;
      }
      
      uint32_t writeTime = micros() - writeStart;
      
      xSemaphoreGive(sdLogMutex); // Give back mutex immediately so main loop can prepare next packet
      
      uint32_t closeStart = micros();
      df.close();  // Close immediately to finalize the FAT table cleanly
      uint32_t closeTime = micros() - closeStart;
      
      if (totalWritten != sizeof(CombinedDataPacket)) {
        Serial.printf("[SD-TASK-ERR] Incomplete write! Expected %d bytes, wrote %d bytes\n", 
                     sizeof(CombinedDataPacket), totalWritten);
      }
      
      packetsLogged++;
      uint32_t taskDuration = millis() - taskStart;
      
      // Print detailed timing on every packet (with microsecond precision)
      if(debug_infos) {
        Serial.printf("[SD-TASK] Packet #%u | Write: %u µs | Close: %u µs | Total: %u ms\n",
                     packetsLogged, writeTime, closeTime, taskDuration);
      }
      Serial.printf("%u, ",packetsLogged);
    } else {
      // Timeout hit, just idle. File is already closed safely on every packet.
    }
  }
}

void setup() {
  initPins();
  initPeripherals();
  initLed();
  initSD();
  uiInit();
  initmmWave();
  initBLE();
  initTimer();
  disableTimer();
  initSPIComm();
  
  sdLogMutex = xSemaphoreCreateMutex();
  
  // Create SD logging task (lower priority, gets 16KB stack - sufficient for CombinedDataPacket)
  xTaskCreatePinnedToCore(
    sdCardLoggingTask,      // Task function
    "SDLoggingTask",        // Task name
    16384,                   // Stack size (Reduced to 4KB since CombinedDataPacket is now static)
    NULL,                   // Parameter
    1,                      // Priority (lower than main)
    &sdTaskHandle,          // Task handle
    0                       // Core 0 
  );
  
  Serial.println("\n=== HYDROCARE MASTER - SPI SENSOR ACQUISITION ===");
}

void loop() {
  //checkUSB();
  
  if (otaUpdateAvailable) {
    uiOTAStarted();
    connectToWiFi();
    performOTAUpdate();
  }
  if(wifi_connect) {
    connectToWiFi();
    wifi_connect = false;  // Reset flag after connecting
  }
  if(deviceStatus==0 && stream_wifi==true){
    delay(150); // Give SD logging task time to hit timeout and close the active file
    streamFolderToTCP(sessionFolder);
    stream_wifi=false;
  }
  if (deviceConnected && timerStream == 1 && deviceStatus == 1 && sessionInitialized) {
    uint32_t loopStart = millis();
    
    // ==================== FETCH SLAVE DATA FIRST ====================
    SensorDataPacket* slaveData = readSlaveData(); 
    
    // ==================== MEASURE MASTER SENSORS ====================
    measureBatteryLevel();
    measureAmbLight();
    measurePIR();
    measuremmWave();
    
    // ==================== COMBINE AND PUSH TO QUEUE ====================
    if (slaveData != nullptr) {
      
      xSemaphoreTake(sdLogMutex, portMAX_DELAY);

      // Copy master sensor readings
      globalLogPacket.batteryLevel = batteryLevel;
      globalLogPacket.batteryPercentage = batteryPercentage;
      globalLogPacket.ambLight = ambLight;
      globalLogPacket.ambLight_Int = ambLight_Int;
      globalLogPacket.PIRValue = PIRValue;
      globalLogPacket.movingDist = movingDist;
      globalLogPacket.movingEnergy = movingEnergy;
      globalLogPacket.staticDist = staticDist;
      globalLogPacket.staticEnergy = staticEnergy;
      globalLogPacket.detectionDist = detectionDist;
      
      // Copy slave sensor data
      memcpy(&globalLogPacket.slaveData, slaveData, sizeof(SensorDataPacket));
      
      xSemaphoreGive(sdLogMutex);

      // Notify SD logging task that new data is ready
      if (sdTaskHandle != NULL) xTaskNotifyGive(sdTaskHandle);
    }
    
    // ==================== BLE NOTIFICATION PHASE ====================
    if (slaveData != nullptr) {
      downsampleRGBFrame(slaveData->rgbFrame, downsampled16x16);
      memcpy(irFrame16x12, slaveData->irFrame, sizeof(irFrame16x12));
    }
    notifyAll();
    
    // Total loop execution time
    if(debug_infos){
      Serial.printf("[LOOP] Cycle: %u ms (< 1000 ms timer)\n", millis() - loopStart);
    }
    
    timerStream = 0;
  }
}
