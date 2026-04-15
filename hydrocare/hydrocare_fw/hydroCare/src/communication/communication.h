#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <SPI.h>

// ==================== PROTOCOL STRUCTURE ====================
// Master-Slave SPI Protocol
// 
// Byte 1 (Command+Address):
//   - Bit 7: R/W (1 = Read, 0 = Write)
//   - Bits 6-0: Address (0-127 addressable registers/memory regions)
//
// Byte 2 (Data - Write only):
//   - Write data (1 byte)
//   - Not sent if Read operation
//
// Special Address 0 (ADDR_SENSOR_DATA):
//   - Read from address 0: Triggers large bulk sensor data transfer (>20KB)
//   - Master must request large byte count for full transfer
//   - Slave handles fragmented/continued transfers automatically

// Protocol constants
#define PROTO_CMD_READ  0x80        // MSB = 1 for read
#define PROTO_CMD_WRITE 0x00        // MSB = 0 for write
#define PROTO_ADDR_MASK 0x7F        // Lower 7 bits = address

// Memory Address Space
#define ADDR_SENSOR_DATA    0x00    // Bulk sensor data (>20KB, can span packets)
#define ADDR_STATUS         0x01    // Slave status register (READ-ONLY)
#define ADDR_CTRL           0x02    // Control register (WRITE to trigger/lock)
#define ADDR_IR_LED         0x03    // IR LED control
#define ADDR_BRIGHTNESS     0x04    // LED brightness (0-100)
#define ADDR_AMB_LIGHT      0x05    // Ambient light reading
#define ADDR_SENSOR_COUNT   0x06    // Sensor sample count available
// 0x07-0x7F: Reserved for future use

// Control Register Values (write to ADDR_CTRL)
#define CTRL_TRIGGER_MEASUREMENT  0x01
#define CTRL_LOCK_BUFFERS         0x02
#define CTRL_UNLOCK_BUFFERS       0x03

// Sensor data packet structure (matches slave)
#pragma pack(1)
typedef struct {
  uint16_t sequence;              // Packet sequence number
  uint16_t ambientLight;          // Ambient light value
  float temperature;              // Temperature in °C
  float humidity;                 // Humidity %
  int16_t accelX, accelY, accelZ; // IMU accel
  int16_t gyroX, gyroY, gyroZ;    // IMU gyro
  uint32_t timestamp_ms;          // System uptime
  uint8_t status;                 // Status flags
  uint16_t accelSampleCount;      // Number of samples
  
  // High-speed samples (2kHz sampling over 1 second measurement window)
  int16_t accelX_samples[2000];
  int16_t accelY_samples[2000];
  int16_t accelZ_samples[2000];
  uint16_t microphoneSamples[2000];
  
  // Camera data
  uint16_t rgbFrame[4096];        // RGB565 64x64
  uint16_t irFrame[192];          // IR thermal 16x12
} SensorDataPacket;
#pragma pack()

#define SPI_CLOCK_HZ     10000000   // 10 MHz 
#define SPI_BUFFER_SIZE  25600      // 25.6KB buffer (handles ~24.6KB packet with 2k samples + margin)

// Slave status bits
#define STATUS_MEASURING 0x01
#define STATUS_MEASURED 0x02
#define STATUS_LOCKED 0x04

// Function prototypes
void initSPIComm();
uint8_t spiRead(uint8_t address);
void spiWrite(uint8_t address, uint8_t data);
void spiReadBulk(uint8_t address, uint8_t *buffer, uint16_t numBytes);
SensorDataPacket* readSlaveData();
void sendIRLED(bool state);
void sendBrightness(uint8_t brightness);

uint16_t* downsampleRGBFrame(uint16_t* rgbFrame64x64, uint16_t* outFrame16x16);

#endif
