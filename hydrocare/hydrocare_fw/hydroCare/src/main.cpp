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
#define NUM_BUFFERS 3
QueueHandle_t dataQueue = NULL;
QueueHandle_t emptyQueue = NULL;
CombinedDataPacket* packetBuffers[NUM_BUFFERS] = {NULL, NULL, NULL};
uint32_t packetsLogged = 0;

// Creates new binary file every 50 packets (e.g., part_0.bin, part_50.bin, part_100.bin)
void sdCardLoggingTask(void *parameter) {
  Serial.println("[SD-TASK] SD logging task started (BINARY mode - 50-packet rotation, safe Open/Close)");

  File df;
  uint32_t currentFileIndex = 0xFFFFFFFF;

  while(1) {
    CombinedDataPacket* packetToWrite = NULL;
    
    // Wait with a 100ms timeout so we can check if logging stopped
    if (xQueueReceive(dataQueue, &packetToWrite, pdMS_TO_TICKS(100)) == pdTRUE) {
      uint32_t taskStart = millis();  // Start timing this packet
      
      if (packetToWrite->slaveData.temperature == 0.0 || packetToWrite->slaveData.sequence == 0xFFFF) {
        Serial.printf("[SD-TASK-ERR] Dropped invalid packet (seq=%u, temp=%.1f)\n", 
                     packetToWrite->slaveData.sequence, packetToWrite->slaveData.temperature);
        xQueueSend(emptyQueue, &packetToWrite, 0); // Return buffer to empty pool
        continue;  
      }
      
      // Calculate which part file to use (every 50 packets = new file)
      uint32_t fileIndex = (packetsLogged / 50) * 50;
      
      // Only open a new file if we crossed the 50-packet boundary or starting fresh
      if (fileIndex != currentFileIndex || !df) {
        if (df) df.close(); // Close the previous file safely
        
        char dataFile[128];
        snprintf(dataFile, sizeof(dataFile), "/%s/%s_part_%u.bin", sessionFolder.c_str(), sessionFolder.c_str(), fileIndex);
        
        df = SD.open(dataFile, FILE_APPEND);
        if (!df) {
          Serial.printf("[SD-TASK-ERR] Failed to open binary file: %s\n", dataFile);
          xQueueSend(emptyQueue, &packetToWrite, 0); // Return buffer to empty pool
          continue;
        }
        currentFileIndex = fileIndex;
      }
      
      size_t expectedSizeAtStart = df.size();
      uint32_t writeStart = micros();  // Microsecond precision for SD timing
      
      // Chunk the 24KB write into 4KB blocks to prevent ESP32 SD buffer saturation
      size_t totalWritten = 0;
      const uint8_t* pData = (const uint8_t*)packetToWrite;
      size_t remaining = sizeof(CombinedDataPacket);
      int writeErrors = 0;
      
      while (remaining > 0) {
        size_t toWrite = (remaining > 4096) ? 4096 : remaining;
        
        size_t writtenChunk = df.write(pData, toWrite);
        
        if (writtenChunk == 0) {
          writeErrors++;
          if (writeErrors > 15) {
            Serial.println("[SD-TASK-ERR] Max SD write errors reached. Aborting packet.");
            break;
          }
          // FATFS error state hit! Retrying on the same 'df' object will instantly return 0.
          Serial.printf("[SD-TASK-WARN] SD stall. Recovering FATFS state (retry %d/15)...\n", writeErrors);
          df.close();
          delay(100); // Give the SD card 100ms to finish its internal block erase
          char recFile[128];
          snprintf(recFile, sizeof(recFile), "/%s/%s_part_%u.bin", sessionFolder.c_str(), sessionFolder.c_str(), currentFileIndex);
          df = SD.open(recFile, FILE_APPEND);
          
          if (!df) {
            Serial.println("[SD-TASK-ERR] Failed to reopen file during recovery.");
            break;
          }
          
          // CRITICAL FIX: Synchronize RAM pointers with actual physical file size on disk
          // because unflushed buffers might have been lost during close().
          size_t actualSize = df.size();
          if (actualSize < expectedSizeAtStart) {
            Serial.println("[SD-TASK-ERR] FAT corruption detected (size shrank)!");
            break;
          }
          
          size_t bytesSuccessfullySaved = actualSize - expectedSizeAtStart;
          if (bytesSuccessfullySaved > sizeof(CombinedDataPacket)) {
            Serial.println("[SD-TASK-ERR] File size grew beyond current packet!");
            break;
          }
          
          // Adjust pointers to resume exactly where the file on disk left off
          pData = (const uint8_t*)packetToWrite + bytesSuccessfullySaved;
          remaining = sizeof(CombinedDataPacket) - bytesSuccessfullySaved;
          totalWritten = bytesSuccessfullySaved;
          
          continue; // Try writing the correctly adjusted chunk
        }
        
        writeErrors = 0; // Reset errors on successful write
        
        totalWritten += writtenChunk;
        pData += writtenChunk;
        remaining -= writtenChunk;
        
        delay(2); // Give the SD card and FreeRTOS a 2ms breathing room between 4KB blocks
      }
      
      uint32_t writeTime = micros() - writeStart;
      
      uint32_t closeStart = micros();
      df.flush();  // Flush instead of close! Saves massive FAT table thrashing.
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

      // Important: Return the processed buffer pointer back to the emptyQueue
      xQueueSend(emptyQueue, &packetToWrite, 0);
    } else {
      // Timeout hit, check if we need to close the file because logging stopped
      if (deviceStatus == 0 && df) {
        df.close();
        currentFileIndex = 0xFFFFFFFF; // Reset for the next session
      }
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
  
  dataQueue = xQueueCreate(NUM_BUFFERS, sizeof(CombinedDataPacket*));
  emptyQueue = xQueueCreate(NUM_BUFFERS, sizeof(CombinedDataPacket*));
  
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
    Serial.println("[MAIN] Stopping logging, waiting for SD card to finish writing...");
    
    delay(150); // Ensure SD task hits its 100ms timeout and safely closes the final file
    
    // 1. Calculate how many buffers we actually allocated
    int allocatedBuffers = 0;
    for(int i=0; i<NUM_BUFFERS; i++) {
      if(packetBuffers[i] != NULL) allocatedBuffers++;
    }
    
    // 2. Wait for all active buffers to return to the emptyQueue (meaning SD task is fully done)
    int waitTime = 0;
    while(uxQueueMessagesWaiting(emptyQueue) < allocatedBuffers && waitTime < 300) {
       delay(10); // Wait up to 3 seconds
       waitTime++;
    }
    
    // 3. Clear queues and free up the ~72 KB of RAM for the Wi-Fi Stack!
    xQueueReset(dataQueue);
    xQueueReset(emptyQueue);
    
    for(int i=0; i<NUM_BUFFERS; i++) {
      if(packetBuffers[i] != NULL) {
        free(packetBuffers[i]);
        packetBuffers[i] = NULL;
      }
    }
    
    Serial.println("[MAIN] Buffers fully freed. Starting Wi-Fi TCP stream.");
    streamFolderToTCP(sessionFolder);
    stream_wifi=false;
  }
  if (deviceConnected && timerStream == 1 && deviceStatus == 1 && sessionInitialized) {
    uint32_t loopStart = millis();
    
    // ==================== DYNAMICALLY ALLOCATE BUFFERS ====================
    for(int i=0; i<NUM_BUFFERS; i++) {
      if(packetBuffers[i] == NULL) {
        packetBuffers[i] = (CombinedDataPacket*)malloc(sizeof(CombinedDataPacket));
        if(packetBuffers[i] != NULL) xQueueSend(emptyQueue, &packetBuffers[i], 0);
        else Serial.println("[MAIN-ERR] Failed to allocate heap buffer!");
      }
    }

    // ==================== FETCH SLAVE DATA FIRST ====================
    SensorDataPacket* slaveData = readSlaveData(); 
    
    // ==================== MEASURE MASTER SENSORS ====================
    measureBatteryLevel();
    measureAmbLight();
    measurePIR();
    measuremmWave();
    
    // ==================== COMBINE AND PUSH TO QUEUE ====================
    if (slaveData != nullptr) {
      
      CombinedDataPacket* currentPacket = NULL;
      
      // Try to get a free buffer from the queue (0 block time)
      if (xQueueReceive(emptyQueue, &currentPacket, 0) == pdTRUE) {
        
        currentPacket->batteryLevel = batteryLevel;
        currentPacket->batteryPercentage = batteryPercentage;
        currentPacket->ambLight = ambLight;
        currentPacket->ambLight_Int = ambLight_Int;
        currentPacket->PIRValue = PIRValue;
        currentPacket->movingDist = movingDist;
        currentPacket->movingEnergy = movingEnergy;
        currentPacket->staticDist = staticDist;
        currentPacket->staticEnergy = staticEnergy;
        currentPacket->detectionDist = detectionDist;
        
        memcpy(&currentPacket->slaveData, slaveData, sizeof(SensorDataPacket));
        
        // Pass the filled pointer to the SD task
        xQueueSend(dataQueue, &currentPacket, 0);
      } else {
        Serial.println("[MAIN-ERR] All 3 buffers full! SD card is too slow. Dropping packet.");
      }
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
