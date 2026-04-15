#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include "config/config.h"
#include <freertos/FreeRTOS.h>

// ============ ADDRESS-BASED R/W PROTOCOL ============
// Command byte format: [R/W bit (7) | 7-bit address (6-0)]
#define PROTO_CMD_READ  0x80        // R/W=1 (bit 7 set)
#define PROTO_CMD_WRITE 0x00        // R/W=0 (bit 7 clear)
#define PROTO_ADDR_MASK 0x7F        // Lower 7 bits = address

// Address space
#define ADDR_SENSOR_DATA  0x00      // Read: 20480 bytes sensor packet
#define ADDR_STATUS       0x01      // Read: 1 byte status
#define ADDR_CTRL         0x02      // Write: 1 byte control (0x01=trigger, 0x02=lock)
#define ADDR_IR_LED       0x03      // Write: 1 byte (0x00=off, 0x01=on)
#define ADDR_BRIGHTNESS   0x04      // Write: 1 byte (0-100)
#define ADDR_AMB_LIGHT    0x05      // Read: 1 byte ambient light

// Control register values
#define CTRL_TRIGGER_MEASUREMENT 0x01
#define CTRL_LOCK_BUFFERS 0x02
#define CTRL_UNLOCK_BUFFERS 0x03

// Slave status bits
#define STATUS_MEASURING 0x01       // Measurement in progress
#define STATUS_MEASURED 0x02        // Measurement complete (awaiting LOCK)
#define STATUS_LOCKED 0x04          // Buffers locked and ready for bulk read

// Constants
#define SPI_BUFFER_SIZE 25600       // 25.6KB - handles ~24.6KB packet (2k samples) + header + margin

// ============ DATA STRUCTURES ============
// Sensor data packet structure with high-speed samples (~16.6KB)
#pragma pack(1)
typedef struct {
  // Metadata
  uint16_t sequence;              // Packet sequence number
  uint16_t ambientLight;          // Ambient light value (instantaneous)
  float temperature;              // Temperature in °C (average)
  float humidity;                 // Humidity % (average)
  int16_t accelX, accelY, accelZ; // IMU accel (most recent single sample)
  int16_t gyroX, gyroY, gyroZ;    // IMU gyro (unused, zeros)
  uint32_t timestamp_ms;          // System uptime when measurement triggered
  uint8_t status;                 // Status flags
  uint16_t accelSampleCount;      // Number of accel/mic samples in this packet (2000)
  
  // High-speed samples (2kHz sampling over 1 second measurement window)
  int16_t accelX_samples[2000];    // 2000 accel X samples @ 2kHz = 1 second
  int16_t accelY_samples[2000];    // 2000 accel Y samples @ 2kHz = 1 second
  int16_t accelZ_samples[2000];    // 2000 accel Z samples @ 2kHz = 1 second
  uint16_t microphoneSamples[2000];// 2000 microphone samples @ 2kHz = 1 second
  
  // Slow sensor frames (camera data)
  uint16_t rgbFrame[4096];        // RGB565 64x64 (8192 bytes)
  uint16_t irFrame[192];          // IR thermal 16x12 (384 bytes)
} SensorDataPacket;
#pragma pack()

// ============ FUNCTION DECLARATIONS ============
// Initialization and communication
void initSPIComm();
void receiveCommand();
void startMeasurementTask();          // Start background measurement collector task
void startHighSpeedSamplerTask();     // Start 2kHz accel+mic sampler task
void startIRSensorTask();             // Start background IR thermal sensor task (200ms)
void startBMESensorTask();            // Start background BME688 sensor task (200ms)

#endif