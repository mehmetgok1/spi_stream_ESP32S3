#include "communication/communication.h"
#include <Arduino.h>
#include <SPI.h>
#include "config/config.h"
#include <cstring>
#include <cstddef>

SPIClass spi(HSPI);
static uint8_t *spiTxBuffer = NULL;
static uint8_t *spiRxBuffer = NULL;

void initSPIComm() {
  spi.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
  
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);
  
  spiTxBuffer = (uint8_t*) heap_caps_malloc(SPI_BUFFER_SIZE, MALLOC_CAP_DMA);
  spiRxBuffer = (uint8_t*) heap_caps_malloc(SPI_BUFFER_SIZE, MALLOC_CAP_DMA);
  
  if (!spiTxBuffer || !spiRxBuffer) {
    Serial.println("[Master] DMA allocation failed!");
    while(1) delay(1000);
  }
  
  memset(spiTxBuffer, 0, SPI_BUFFER_SIZE);
  memset(spiRxBuffer, 0, SPI_BUFFER_SIZE);
  
  Serial.println("[Master] SPI Init OK - Protocol-based 10 MHz, 20KB DMA buffers");
}

// ==================== PROTOCOL IMPLEMENTATION ====================
// Protocol: Master-Slave SPI with address-based R/W
// 
// *** CRITICAL: SPI Protocol Sequence ***
// Master MUST send first. Slave ONLY responds to what master sends.
// 
// STEP 1: Master sends Command Byte [R/W bit (MSB)] [Address (7 bits)]
//         => Slave receives command, prepares response
//         => Master receives garbage/echo (first byte is MEANINGLESS - discard it)
//
// STEP 2: Master sends dummy bytes (0x00) to clock in slave response
//         => Slave sends actual response/data
//         => Master receives real data
//
// Command Byte format:
//   Bit 7: R/W (1 = Read, 0 = Write)
//   Bits 6-0: Address (0-127)
//
// For WRITE operations:
//   Byte 0: [R/W=0 | Address]
//   Byte 1: Data byte
//
// For READ operations:
//   Byte 0: [R/W=1 | Address]  (master sends, slave receives)
//   Byte 1+: Master sends 0x00, slave responds with data

// Single byte READ from slave register/memory address
// 
// Synchronous 2-byte protocol:
// Byte 0: Master sends [R/W=1 | Address]  <- Slave sends DUMMY
// Byte 1: Master sends 0x00 (dummy)       <- Slave sends DATA/STATUS
//
// Status is always at byte 1 in slave response!
// Returns: Data/Status value from slave (byte 1 of transaction)
uint8_t spiRead(uint8_t address) {
  if (address > 0x7F) {
    Serial.printf("[SPI Error] Read address 0x%02X exceeds 7-bit space\n", address);
    return 0xFF;
  }
  
  uint8_t cmdByte = PROTO_CMD_READ | (address & PROTO_ADDR_MASK);
  
  spi.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(50);
  
  // ATOMIC: Prevent task switching during critical SPI read
  taskDISABLE_INTERRUPTS();
  
  // Byte 0: Master sends command, discard slave's dummy response
  spi.transfer(cmdByte);
  
  // Byte 1: Master sends dummy (0x00), slave sends DATA/STATUS
  uint8_t readData = spi.transfer(0x00);
  
  taskENABLE_INTERRUPTS();
  
  delayMicroseconds(50);
  digitalWrite(SPI_CS, HIGH);
  spi.endTransaction();
  
  return readData;
}

// Single byte WRITE to slave register/memory address
//
// Synchronous 3-byte protocol:
// Byte 0: Master sends [R/W=0 | Address]  <- Slave sends DUMMY
// Byte 1: Master sends 0x00 (dummy)       <- Slave sends STATUS
// Byte 2: Master sends WRITE DATA         <- Slave sends DUMMY/ACK
//
// Note: Slave processes write AFTER CS goes HIGH (blocking, synchronous)
//
void spiWrite(uint8_t address, uint8_t data) {
  if (address > 0x7F) {
    Serial.printf("[SPI Error] Write address 0x%02X exceeds 7-bit space\n", address);
    return;
  }
  
  uint8_t cmdByte = PROTO_CMD_WRITE | (address & PROTO_ADDR_MASK);
  
  spi.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(50);
  
  // ATOMIC: Prevent task switching during critical SPI write
  taskDISABLE_INTERRUPTS();
  
  // Byte 0: Master sends write command+address, discard dummy
  spi.transfer(cmdByte);
  
  // Byte 1: Master sends dummy (0x00), slave sends STATUS
  uint8_t statusByte = spi.transfer(0x00);
  
  // Byte 2: Master sends actual WRITE DATA, slave sends ACK/DUMMY
  spi.transfer(data);
  
  taskENABLE_INTERRUPTS();
  
  delayMicroseconds(50);
  digitalWrite(SPI_CS, HIGH);
  spi.endTransaction();
  
  // Slave now processes the write (after CS HIGH)
}

// Bulk READ from slave (for large transfers like sensor data)
//
// Synchronous protocol with pre-prepared data:
// 1. Master sends: [R/W=1 | Address]  <- Slave sends DUMMY
//    Master receives: GARBAGE (discard)
// 2. Master sends: 0x00               <- Slave sends STATUS
//    Master receives: STATUS byte
// 3. Master loop: Send 0x00 bytes     <- Slave sends pre-prepared data
//    Master receives: actual bulk data (bytes 2 onwards)
//
// Special case: address=0 (ADDR_SENSOR_DATA) reads pre-prepared sensor packet
// Slave has prepared all data BEFORE this transaction (during LOCK command)
//
void spiReadBulk(uint8_t address, uint8_t *buffer, uint16_t numBytes) {
  if (address > 0x7F) {
    Serial.printf("[SPI Error] Read address 0x%02X exceeds 7-bit space\n", address);
    return;
  }
  
  if (numBytes > SPI_BUFFER_SIZE) {
    Serial.printf("[SPI Error] Requested bytes %u exceeds buffer %u\n", numBytes, SPI_BUFFER_SIZE);
    return;
  }
  
  if (!buffer) {
    Serial.println("[SPI Error] NULL buffer pointer");
    return;
  }
  
  uint8_t cmdByte = PROTO_CMD_READ | (address & PROTO_ADDR_MASK);
  
  spi.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(50);
  
  // Fill TX buffer: first byte is command, rest are zeros (clock in slave response)
  spiTxBuffer[0] = cmdByte;
  memset(&spiTxBuffer[1], 0x00, numBytes + 1);  // Clear rest of buffer
  
  // CRITICAL: Disable both interrupts AND FreeRTOS task switching during DMA transfer
  // This prevents BLE/SD tasks from interrupting the critical SPI operation
  taskDISABLE_INTERRUPTS();
  spi.transferBytes(spiTxBuffer, spiRxBuffer, numBytes + 2);  // CMD + STATUS + DATA
  taskENABLE_INTERRUPTS();
  
  // Extract status from received byte 1 and sensor data from bytes 2+
  uint8_t statusByte = spiRxBuffer[1];
  memmove(buffer, &spiRxBuffer[2], numBytes);
  
  delayMicroseconds(50);
  digitalWrite(SPI_CS, HIGH);
  spi.endTransaction();
  
  Serial.printf("[SPI] Bulk read status: 0x%02X | %u bytes received\n", statusByte, numBytes);
}

// ==================== CONVENIENCE FUNCTIONS ====================

// Read sensor data from slave using protocol-based state machine:
// 1. Set trigger in CTRL register
// 2. Poll STATUS for measurement complete
// 3. Set lock in CTRL register
// 4. Poll STATUS for lock complete
// 5. Bulk read sensor data
// Called from main.cpp every 1 second
SensorDataPacket* readSlaveData() {

  // ========== STEP 1: Set Trigger ==========
  spiWrite(ADDR_CTRL, CTRL_TRIGGER_MEASUREMENT);
  Serial.println("[Master] write trigger measurement command");
  delay(1);
  // ========== STEP 2: Poll for MEASURED status ==========
  uint32_t startTime = millis();
  uint8_t status = 0;
  bool measured = false;
  while (millis() - startTime < 2000) {  // 2 second timeout
    status = spiRead(ADDR_STATUS);
    if (status != 0xFF && (status & STATUS_MEASURED)) {
      measured = true;
      break;
    }
    delay(2);
  }
  if (!measured) {
    Serial.println("[Master] ERROR: Timeout waiting for STATUS_MEASURED");
    return nullptr;
  }
  // ========== STEP 3: Set Lock ==========
  spiWrite(ADDR_CTRL, CTRL_LOCK_BUFFERS);
  Serial.println("[Master] write lock data trigger");
  delay(2);
  // ========== STEP 4: Poll for LOCKED status ==========
  startTime = millis();
  bool locked = false;
  while (millis() - startTime < 2000) {  // 2 second timeout
    status = spiRead(ADDR_STATUS);
    if (status != 0xFF && (status & STATUS_LOCKED)) {
      locked = true;
      break;
    }
    delay(3);
  }
  if (!locked) {
    Serial.println("[Master] ERROR: Timeout waiting for STATUS_LOCKED");
    return nullptr;
  }
  // ========== STEP 5: Bulk Read Sensor Data ==========
  spiReadBulk(ADDR_SENSOR_DATA, spiRxBuffer, SPI_BUFFER_SIZE);
  Serial.println("[Master] Ready to process sensor data packet");

  // ========== STEP 6: Release Lock ==========
  delay(1);
  spiWrite(ADDR_CTRL, CTRL_UNLOCK_BUFFERS);
  Serial.println("[Master] write unlock buffers command");
  delay(1);
  // Cast packet directly (data starts at byte 0 - pure sensor packet)
  SensorDataPacket *packet = (SensorDataPacket*)(spiRxBuffer);
  
  // Verify packet integrity
  Serial.printf("[Packet] Sequence: %u | Temp: %.1f°C | Humidity: %.1f%% | Light: %u\n",
    packet->sequence, packet->temperature, packet->humidity, packet->ambientLight);
  
  Serial.printf("[Samples] Accel count: %u | X:%+d Y:%+d Z:%+d mG\n",
    packet->accelSampleCount, packet->accelX, packet->accelY, packet->accelZ);
  return packet;
}

// LED control using protocol address writes
void sendIRLED(bool state) {
  Serial.printf("[Master] Setting IR LED: %s\n", state ? "ON" : "OFF");
  spiWrite(ADDR_IR_LED, state ? 0x01 : 0x00);
}

void sendBrightness(uint8_t brightness) {
  if (brightness > 100) brightness = 100;
  Serial.printf("[Master] Setting LED brightness: %u%%\n", brightness);
  spiWrite(ADDR_BRIGHTNESS, brightness);
}



// Downsample 64x64 RGB565 frame to 16x16 by averaging 4x4 pixel blocks
// Input: rgbFrame64x64 (4096 pixels = 64x64)
// Output: outFrame16x16 (256 pixels = 16x16)
// Returns: pointer to downsampled frame
uint16_t* downsampleRGBFrame(uint16_t* rgbFrame64x64, uint16_t* outFrame16x16) {
  if (!rgbFrame64x64 || !outFrame16x16) {
    return nullptr;
  }
  
  int outIdx = 0;
  
  // Iterate through 16x16 output grid
  for (int outY = 0; outY < 16; outY++) {
    for (int outX = 0; outX < 16; outX++) {
      // Each output pixel represents a 4x4 block from input
      int inY_start = outY * 4;
      int inX_start = outX * 4;
      
      uint32_t sumR = 0, sumG = 0, sumB = 0;
      uint8_t count = 0;
      
      // Average 4x4 block
      for (int dy = 0; dy < 4; dy++) {
        for (int dx = 0; dx < 4; dx++) {
          int inY = inY_start + dy;
          int inX = inX_start + dx;
          
          if (inY < 64 && inX < 64) {
            uint16_t pixel = rgbFrame64x64[inY * 64 + inX];
            
            // Extract RGB565: RRRRRGGGGGGBBBBB
            uint8_t r = (pixel >> 11) & 0x1F;  // 5 bits
            uint8_t g = (pixel >> 5) & 0x3F;   // 6 bits
            uint8_t b = pixel & 0x1F;          // 5 bits
            
            sumR += r;
            sumG += g;
            sumB += b;
            count++;
          }
        }
      }
      
      // Average and recombine
      if (count > 0) {
        uint8_t avgR = sumR / count;
        uint8_t avgG = sumG / count;
        uint8_t avgB = sumB / count;
        
        // Recombine to RGB565
        uint16_t averaged = ((avgR & 0x1F) << 11) | ((avgG & 0x3F) << 5) | (avgB & 0x1F);
        outFrame16x16[outIdx++] = averaged;
      }
    }
  }
  
  return outFrame16x16;
}
