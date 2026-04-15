# HydroCare Master-Slave System Architecture

Master fw version :  0.0.21
Slave  fw version :  0.0.13
## System Overview

HydroCare is a **distributed IoT sensor system** with two ESP32-S3 processors:

1. **Slave (Sensor Board)** - Data acquisition and preprocessing
   - Location: `/hydrocare_sensor/hydroCare_Sensor/`
   - Role: Continuous sensor sampling, buffering, caching
   - Output: Binary sensor packets via SPI to master

2. **Master (Main Board)** - Data logging and BLE connectivity
   - Location: `/hydrocare_fw/hydroCare/`
   - Role: Orchestrates measurements, logs to SD card, sends BLE notifications
   - Output: Binary files on SD card (`part_0.bin`, `part_50.bin`, etc.)

---

## Physical Connection (SPI Protocol)

### Hardware Configuration
```
Clock Speed:    10 MHz
SPI Mode:       SPI_MODE0 (CPOL=0, CPHA=0)
Bit Order:      MSBFIRST
CS Active:      LOW
DMA Buffers:    25.6 KB (sufficient for ~24.6 KB packet + overhead)
Max Address:    0x7F (128 locations)
```

### Pinout (Typical ESP32-S3)
```
Master                Slave
─────────────────────────────
GPIO_MOSI_M   ←→   GPIO_MOSI_S    (Command/Data TO slave)
GPIO_MISO_M   ←→   GPIO_MISO_S    (Response FROM slave)
GPIO_SCK_M    ←→   GPIO_SCK_S     (Clock)
GPIO_CS_M     ←→   GPIO_CS_S      (Chip Select - Active LOW)
GND           ←→   GND            (Common ground)
```

---

## Communication Protocol

### Packet Structure (Byte Level)

**Master sends bytes, Slave responds simultaneously on MISO line.**

#### Single-Byte Read (2 bytes total) - Master wants to READ from Slave

```
Byte 0 (Command):
  Master TX:  [R/W=1 | ADDR]        (read command with address)
  Slave TX:   [DUMMY]               (not used)
  Master uses: IGNORES              (reads only for timing)
  
Byte 1 (Data):
  Master TX:  [0x00]                (clock pulse)
  Slave TX:   [DATA]                (actual data from slave)
  Master uses: READS & USES         (this is the value we want)
```

**Example: Master reads STATUS byte**
```c
uint8_t status = spiRead(ADDR_STATUS);
// Returns: slave's current status (from byte 1 response)
```

---

#### Single-Byte Write (3 bytes total) - Master wants to WRITE to Slave

```
Byte 0 (Command):
  Master TX:  [R/W=0 | ADDR]        (write command with address)
  Slave TX:   [DUMMY]               (not used)
  Master uses: IGNORES
  
Byte 1 (Padding):
  Master TX:  [0x00]                (padding byte)
  Slave TX:   [STATUS]              (slave's status)
  Master uses: IGNORES
  
Byte 2 (Data):
  Master TX:  [DATA]                (data master wants to write)
  Slave TX:   [ACK]                 (not used)
  Master uses: IGNORES
  
SLAVE PROCESSING: After CS goes HIGH, slave reads the DATA from byte 2 and processes
```

**Example: Master triggers measurement**
```c
spiWrite(ADDR_CTRL, CTRL_TRIGGER_MEASUREMENT);
// Slave reads byte 2 and executes the trigger
```

---

#### Bulk Read - 25.6 KB Sensor Packet (25,602 bytes total)

```
Byte 0 (Command):
  Master TX:  [R/W=1 | ADDR]               (read command)
  Slave TX:   [DUMMY]                      (not used)
  Master uses: IGNORES
  
Byte 1 (Status):
  Master TX:  [0x00]                       (clock pulse)
  Slave TX:   [STATUS]                     (slave status byte)
  Master uses: IGNORES (or optionally checks)
  
Bytes 2-25,601 (Data Stream - DMA):
  Master TX:  [0x00] × 25,600 bytes        (clock stream via DMA)
  Slave TX:   [SENSOR DATA] × 25,600 bytes (txBuf contents via DMA)
  Master uses: READS & USES ALL            (entire sensor packet captured)
```

**Example: Master reads sensor packet**
```c
spi_slave_transmit(SPI2_HOST, &transaction, timeout);
// Master receives 25,600 bytes of txBuf into rxBuf
// This is one single blocking DMA transfer
```

---

---

## Control Registers (Master-to-Slave Commands)

| Addr | Name | Type | Purpose |
|------|------|------|---------|
| 0x02 | CTRL | Write | Measurement control (trigger/lock/unlock) |
| 0x03 | IR_LED | Write | IR LED control (0=off, 1=on) |
| 0x04 | BRIGHTNESS | Write | LED brightness (0-100%) |

### CTRL Register Values
```c
#define CTRL_TRIGGER_MEASUREMENT   0x01   // Start measurement
#define CTRL_LOCK_BUFFERS          0x02   // Freeze & prepare data
#define CTRL_UNLOCK_BUFFERS        0x03   // Release for next cycle
```

### Status Register (Slave-to-Master Responses)

**Byte 1 of every SPI transaction (status byte):**

This byte is **prefilled in txBuf[1]** and **automatically updated** by the slave in the background:

```c
// In slave's communication.cpp, txBuf is initialized as:
txBuf[0] = 0x00;  // Dummy (always 0x00)
txBuf[1] = 0x00;  // Status byte (updated on state changes)
txBuf[2+] = ...;  // Sensor data (bulk read only)
```

**Status byte contents:**
```c
#define STATUS_IDLE         0x00   // Ready for next trigger
#define STATUS_MEASURING    0x01   // Currently measuring
#define STATUS_MEASURED     0x02   // Measurement complete, data ready
#define STATUS_LOCKED       0x04   // Data locked, can be read
```

**How it works:**
- **Every SPI transaction returns the current status at byte 1** (no extra overhead)
- Status is updated by background tasks on state changes
- Master can poll by reading byte 1 repeatedly
- Master always knows slave state without extra commands

---

## Measurement State Machine

The slave operates in a strict finite state machine:

```
IDLE
  ↓
  [Master sends TRIGGER_MEASUREMENT]
  ↓
MEASURING (background task collects packet)
  ↓
  [Measurement completes automatically (~11ms)]
  ↓
MEASURED (packet in currentData buffer, ready)
  ↓
  [Master sends LOCK_BUFFERS]
  ↓
READY_TRANSFER (data copied to txBuf, can be read)
  ↓
  [Master reads 25.6 KB packet via SPI bulk read]
  ↓
  [Master sends UNLOCK_BUFFERS]
  ↓
IDLE (ready for next cycle)
```

**Critical detail:** Data is LOCKED before master reads it, preventing race conditions with background sampling tasks.

---

## What Data is in Each Packet (24.6 KB Total)

### SensorDataPacket Structure (from slave)

```c
#pragma pack(1)
typedef struct {
  // === METADATA (31 bytes) ===
  uint16_t sequence;           // Packet counter (0, 1, 2, ...)
  uint16_t ambientLight;       // Ambient light sensor
  float temperature;           // Average IR thermal temp (°C)
  float humidity;              // BME688 humidity (%)
  int16_t accelX, accelY, accelZ;   // IMU latest accel
  int16_t gyroX, gyroY, gyroZ;      // IMU latest gyro
  uint32_t timestamp_ms;       // System uptime (ms)
  uint8_t status;              // Status flags
  uint16_t accelSampleCount;   // Always 2000
  
  // === HIGH-SPEED SAMPLES (8000 bytes) ===
  // Continuous 2kHz sampling for 1 second
  int16_t accelX_samples[2000];      // X acceleration @ 2kHz
  int16_t accelY_samples[2000];      // Y acceleration @ 2kHz
  int16_t accelZ_samples[2000];      // Z acceleration @ 2kHz
  uint16_t microphoneSamples[2000];  // Audio @ 2kHz
  
  // === CAMERA FRAMES (8576 bytes) ===
  uint16_t rgbFrame[4096];    // RGB565 64×64 pixels (center crop of 320×240)
  uint16_t irFrame[192];      // Thermal 16×12 pixels, fixed-point:
                               // (raw / 100.0) - 40 = temp_°C
} SensorDataPacket;  // Total: ~24.6 KB (padded to 25.6 KB for SPI buffer)
#pragma pack()
```

### CombinedDataPacket Structure (master layer on top)

The **master** wraps slave data with its own sensor readings:

```c
typedef struct {
  // === MASTER SENSORS (26 bytes) ===
  float batteryLevel;          // Battery voltage
  float batteryPercentage;     // Battery 0-100%
  float ambLight;              // Ambient light
  uint16_t ambLight_Int;       // Ambient light (uint16)
  float PIRValue;              // PIR motion detection
  uint16_t movingDist;         // mmWave moving target distance
  uint8_t movingEnergy;        // mmWave moving target energy
  uint16_t staticDist;         // mmWave static target distance
  uint8_t staticEnergy;        // mmWave static target energy
  uint16_t detectionDist;      // mmWave detection distance
  
  // === SLAVE DATA (24.6 KB) ===
  SensorDataPacket slaveData;  // Complete slave packet
} CombinedDataPacket;  // Total: ~24.6 KB
#pragma pack()
```

---

## How Master Gets Data (Per Loop Cycle)

**Timing: Every 1000ms (1Hz timer, controlled by BLE callback)**

### Step 1: Trigger Measurement on Slave
```cpp
Master sends: WRITE [ADDR_CTRL] = CTRL_TRIGGER_MEASUREMENT
Slave enters: MEASURING state
Slave task starts: ambient light, IMU samples, IR read, RGB capture (11ms total)
```

### Step 2: Wait for Measurement Complete
```cpp
Master polls: READ [ADDR_STATUS] → wait for STATUS_MEASURED
Slave task does: Completes data collection, signals STATUS_MEASURED
Polling continues until slave returns STATUS_MEASURED
```

### Step 3: Lock Buffers (Prepare Data)
```cpp
Master sends: WRITE [ADDR_CTRL] = CTRL_LOCK_BUFFERS
Slave: Copies currentData (with new timestamp/sequence) into txBuf
Slave state: READY_TRANSFER (txBuf now contains the data to transmit)
```

### Step 4: Bulk Read Sensor Packet
```cpp
Master issues: SPI bulk read from ADDR_SENSOR_DATA
Master receives: 25.6 KB of SensorDataPacket from txBuf
Slave: Data streamed via DMA
Duration: ~25 ms @ 10 MHz clock
```

### Step 5: Unlock and Next Cycle
```cpp
Master sends: WRITE [ADDR_CTRL] = CTRL_UNLOCK_BUFFERS
Slave state: IDLE (ready for next TRIGGER)
Master: Returns to main loop
```

**One complete cycle duration:** ~100ms for measurement + 25ms for SPI read = ~125ms net

---

## Master-Side Data Processing

### Step A: Combine Master + Slave Data
```cpp
CombinedDataPacket packet;
packet.batteryLevel = measureBatteryLevel();
packet.ambLight = measureAmbLight();
packet.PIRValue = measurePIR();
packet.movingDist = measuremmWave();
// ... etc (all master sensors)

memcpy(&packet.slaveData, slaveSensorPacket, sizeof(SensorDataPacket));
```

### Step B: Queue for SD Logging (Non-Blocking)
```cpp
xQueueSend(sensorDataQueue, &packet, 0);  // Enqueue, don't block
```
SD logging task (running on Core 1, priority 1) processes queue independently.

### Step C: Process for BLE Notifications
```cpp
downsampleRGBFrame(slaveData->rgbFrame, downsampled16x16);
memcpy(irFrame16x12, slaveData->irFrame, sizeof(irFrame16x12));
notifyAll();  // Send BLE notifications
```

---

## RGB Downsampling for BLE

### Why Downsampling?
- **Slave captures:** 64×64 RGB pixels = 4,096 pixels × 2 bytes = 8,192 bytes per frame
- **BLE MTU:** Limited to ~200 bytes per notification (with overhead)
- **Solution:** Downsampling to 16×16 = 256 pixels × 2 bytes = 512 bytes

### Downsampling Algorithm

```cpp
void downsampleRGBFrame(uint16_t* src64x64, uint16_t* dst16x16) {
  // Input: 64×64 RGB565 frame (4,096 pixels)
  // Output: 16×16 RGB565 frame (256 pixels)
  // Ratio: 4×4 pixels → 1 output pixel (every 4th pixel, every 4th row)
  
  int dstIdx = 0;
  for (int row = 0; row < 64; row += 4) {
    for (int col = 0; col < 64; col += 4) {
      int srcIdx = (row * 64) + col;
      dst16x16[dstIdx++] = src64x64[srcIdx];
    }
  }
  // Result: Simple nearest-neighbor sampling (every 4th pixel)
}
```

### BLE Transmission Flow
```
Downsampled 16×16 frame (512 bytes)
         ↓
    Split into chunks (200 bytes each)
         ↓
    Notify client with 3 BLE notifications:
    - Chunk 1: 200 bytes
    - Chunk 2: 200 bytes  
    - Chunk 3: 112 bytes
         ↓
    Client reassembles into full frame
```

---

## SD Card Data Storage

### File Organization
```
/YYYYMMDD_HHMMSS/                    ← Session folder (created per BLE start)
├── YYYYMMDD_HHMMSS_part_0.bin       ← Packets 0-49
├── YYYYMMDD_HHMMSS_part_50.bin      ← Packets 50-99
├── YYYYMMDD_HHMMSS_part_100.bin     ← Packets 100-149
└── ...
```

### Per-Packet Logging
```cpp
while (xQueueReceive(sensorDataQueue, &packet, portMAX_DELAY)) {
  // Validate packet (sequence != 0, temperature != 0.0)
  // Calculate file path: part_(packets / 50) * 50
  // Write: df.write(&packet, sizeof(CombinedDataPacket))  [~50µs write]
  // Close: df.close()                                      [~25ms close]
  // Total: ~100ms per packet
}
```

### Data Durability
- File is **closed after every packet** (not kept open)
- Ensures data is flushed to SD card immediately
- Trade-off: Close overhead (~25ms) vs. data safety (acceptable)

---

## FreeRTOS Background Tasks

### Slave-Side (hydrocare_sensor/)

| Task | Core | Priority | Function | Frequency |
|------|------|----------|----------|-----------|
| highSpeedSamplerTask | 1 | 1 | 2kHz accel+mic ring buffer | Continuous |
| irSensorBackgroundTask | 1 | 1 | Read IR thermal sensor | 200ms |
| bmeSensorBackgroundTask | 1 | 1 | Read BME688 environment | 200ms |
| measurementCollectorTask | 0 | 2 | Assemble final packet on TRIGGER | On-demand |
| receiveCommand (main loop) | 0 | - | SPI handler (blocking) | Continuous |

### Master-Side (hydrocare_fw/)

| Task | Core | Priority | Function | Stack |
|------|------|----------|----------|-------|
| sdCardLoggingTask | 1 | 1 | Queue consumer, writes binary to SD | 16 KB |
| (main loop) | 0 | - | Sensor acquisition, BLE notify | - |

---

## Performance Metrics

| Operation | Time | Notes |
|-----------|------|-------|
| Slave measurement | ~11 ms | IMU + camera + IR + BME |
| SPI transfer (25.6 KB) | ~25 ms | @ 10 MHz clock |
| SD write (24.6 KB) | ~50 µs | DMA optimized |
| SD close | ~25 ms | Flush to media |
| Master loop cycle | 100-140 ms | < 1000 ms timer requirement ✓ |

---

## Data Quality & Integrity

### Ring Buffer (Slave-Side)
- Size: 5,000 samples @ 2kHz = 2.5 seconds of history
- Prevents race conditions: Measurement reader always far behind sampler
- Thread-safe: Single writer (no mutex needed)

### Double Buffering (Slave Caches)
```cpp
// IR and BME sensors update async (200ms)
// Use double buffers to prevent mid-read updates
irCache[2];    // Master reads from [1 - irWriteIdx]
               // Background task writes to [irWriteIdx]
```

### CombinedDataPacket Validation (Master-Side)
```cpp
if (packet.slaveData.sequence == 0 || packet.slaveData.temperature == 0.0) {
  // Drop invalid packet
}
```

---

## Temperature Conversion (Post-Processing)

### IR Thermal Data
Stored as **fixed-point uint16_t** for space efficiency:
```
Raw in file:    6341
Conversion:     (6341 / 100.0) - 40 = 23.41°C
```

### Average Temperature
Stored directly as `float` in packet:
```
Value in file:  19.5
Already in °C:  19.5°C (no conversion needed)
```

---

## System Workflow (Complete Cycle)

```
[IDLE]
  ↓
[BLE "Start" command arrives]
  ↓
  Master: sessionFolder = timestamp, initSessionFolder()
          sessionInitialized = true
  ↓
[1000ms timer fires (1 Hz)]
  ↓
  Master: readSlaveData() → TRIGGER_MEASUREMENT
             ↓
             Slave: measurementCollectorTask wakes up
                    Collects all sensor data
                    Sets STATUS_MEASURED
             ↓
             Master: polls STATUS until STATUS_MEASURED
  ↓
  Master: sends LOCK_BUFFERS
             ↓
             Slave: copies currentData → txBuf
                    Sets STATE_READY_TRANSFER
  ↓
  Master: reads 25.6 KB packet via SPI bulk read from txBuf
  ↓
  Master: sends UNLOCK_BUFFERS
             ↓
             Slave: state = IDLE
  ↓
  Master: measureBatteryLevel(), measureAmbLight(), measurePIR(), measuremmWave()
  ↓
  Master: Combine master + slave → CombinedDataPacket
  ↓
  Master: xQueueSend() → packet to queue
             ↓
             SD Task: Dequeues packet
                      Open/create part file
                      Write 24.6 KB to SD
                      Close file
                      [~100ms total]
  ↓
  Master: downsampleRGBFrame(), copy IR frame
  ↓
  Master: notifyAll() → BLE RGB + IR notifications
  ↓
  [Wait 1000ms until next timer]
  ↓
[Repeat]
  ↓
[BLE "Stop" command arrives]
  ↓
  Master: sessionInitialized = false
  ↓
[IDLE - ready for next session]
```

---

## Summary

**Master-Slave architecture enables:**
- ✅ **Parallel processing:** Slave samples @ 2kHz, master logs @ 1Hz
- ✅ **Data efficiency:** Binary format, 50-packet files, uint16 compression
- ✅ **Reliability:** Data locked before read, double buffering, validation
- ✅ **Scalability:** Queue-based logging, non-blocking operations
- ✅ **Real-time:** BLE notifications with downsampled frames

All sensor data converges into **CombinedDataPacket (~24.6 KB)** every second, logged to SD card and streamed via BLE.

---

## Post-Processing: Binary to CSV/PNG Conversion

### Overview

The binary `.bin` files on SD card are compact but not human-readable. A **Python post-processing script** converts them into accessible formats:

**Input:** Session folder with part files
```
/YYYYMMDD_HHMMSS/
├── YYYYMMDD_HHMMSS_part_0.bin
├── YYYYMMDD_HHMMSS_part_50.bin
├── YYYYMMDD_HHMMSS_part_100.bin
└── ...
```

**Output:** Organized data folders
```
processed_YYYYMMDD_HHMMSS/
├── sensordata/
│   └── all_sensors.csv         ← Master + slave sensor summary
├── accel_mic/
│   └── accel_mic_stream.csv    ← 2000 accel + 2000 mic samples per packet
├── colored_image/
│   ├── rgb_176078_21.png       ← 64×64 RGB frames (timestamp_packetnum.png)
│   ├── rgb_176520_22.png
│   └── ...
└── irimage/
    ├── ir_176078_21.csv        ← 16×12 thermal grids, **RAW uint16** (needs conversion!)
    ├── ir_176520_22.csv
    └── ...
```

### Python Script Usage

**Simple invocation (only requires path):**
```bash
python uri_bin_extracter.py /path/to/session/folder
```

**Example:**
```bash
python uri_bin_extracter.py /home/user/hydrocare/SD_content/20260328_145322
```

The script automatically:
1. ✅ Finds all `part_*.bin` files in the folder
2. ✅ Reads and combines binary structs (CombinedDataPacket)
3. ✅ Parses all sensor data
4. ✅ Generates CSV files (accelerometer, microphone, sensor summary)
5. ✅ Generates PNG images (RGB frames)
6. ✅ Organizes into intuitive folder structure

### Data Conversion Summary

**all_sensors.csv & accel_mic_stream.csv:** Ready-to-use (temp in °C, accel in mG)

**⚠️ ir_*.csv files ONLY:** Contain raw uint16_t requiring conversion:
```python
temp_celsius = (raw_uint16 / 100.0) - 40  # Example: 6341 → 23.41°C
```

### CSV Format Examples

#### all_sensors.csv
```
timestamp_ms,sequence,battery_pct,ambLight_M,PIR,mmWave_dist,temp,humi,ambLight_S
176078,21,92.1,145,0.32,250,21.5,44.8,156
176520,22,91.9,148,0.28,249,21.6,44.9,158
177520,23,91.7,151,0.25,248,21.7,45.0,151
...
```

One row per packet with sensor summary data:
- `timestamp_ms` - Packet timestamp
- `sequence` - Packet sequence number
- `battery_pct` - Battery percentage (0-100%)
- `ambLight_M` - Ambient light (master sensor)
- `PIR` - PIR motion value
- `mmWave_dist` - mmWave detection distance
- `temp` - Average IR temperature (°C) 
- `humi` - Humidity (%)
- `ambLight_S` - Ambient light (slave sensor)

#### accel_mic_stream.csv
```
timestamp_ms,accelX,accelY,accelZ,mic
176078,-5,0,10,512
176078,0,5,15,510
176078,10,15,20,508
...
176078,20,-10,25,480    ← 2000 rows all with same timestamp_ms
176520,-8,8,12,500
176520,5,10,18,498
...
176520,25,-5,30,470     ← 2000 more rows with next timestamp
```

Each row is ONE accelerometer + microphone sample:
- **2000 rows per packet** (2000 samples @ 2kHz = 1 second of data)
- All rows in a packet share the same `timestamp_ms`
- **accelX, accelY, accelZ** in **milligravity (mG)** - ready to use
- **mic** in raw ADC units (application-dependent scaling)

#### ir_*.csv - **REQUIRES CONVERSION**
```
Example: ir_176078_21.csv (packet 21, timestamp 176078ms)
row,col_0,col_1,col_2,col_3,col_4,col_5,col_6,col_7,col_8,col_9,col_10,col_11,col_12,col_13,col_14,col_15
0,6341,6342,6331,6335,6355,6354,6357,6355,6372,6363,6373,6374,6378,6374,6386,6387
1,6334,6338,6326,6340,6351,6353,6351,6354,6360,6363,6365,6364,6367,6378,6383,6384
2,6337,6334,6327,6342,6352,6358,6347,6358,6360,6357,6365,6367,6374,6370,6387,6372
...
11,6343,6345,6344,6355,6356,6351,6348,6355,6366,6354,6351,6357,6352,6361,6357,6361

After converting (raw / 100.0) - 40:
row,col_0,col_1,col_2,col_3,col_4,col_5,col_6,col_7,col_8,col_9,col_10,col_11,col_12,col_13,col_14,col_15
0,23.41,23.42,23.31,23.35,23.55,23.54,23.57,23.55,23.72,23.63,23.73,23.74,23.78,23.74,23.86,23.87
1,23.34,23.38,23.26,23.40,23.51,23.53,23.51,23.54,23.60,23.63,23.65,23.64,23.67,23.78,23.83,23.84
...
```

### Post-Processing Workflow Example

```python
import pandas as pd
import numpy as np

# Read all_sensors.csv (already converted)
sensors = pd.read_csv('sensordata/all_sensors.csv')
print(sensors[['timestamp_ms', 'temp_avg_c', 'humidity_pct']])

# Read IR thermal data and convert
ir_raw = pd.read_csv('irimage/ir_176078_21.csv', index_col='row')
ir_celsius = (ir_raw / 100.0) - 40  # CONVERT!

print(ir_celsius)  # Now in °C

# Read accel_mic samples
accel_mic = pd.read_csv('accel_mic/accel_mic_stream.csv')
accel_x = accel_mic['accel_x[0]':'accel_x[1999]']  # All accel X samples

# Plot thermal heatmap
import matplotlib.pyplot as plt
plt.imshow(ir_celsius.values, cmap='hot')
plt.colorbar(label='Temperature (°C)')
plt.title('Thermal Heatmap Packet 21')
plt.show()
```

### Requirements

```
Python 3.7+
numpy        (array processing)
pandas       (CSV reading/writing)
PIL/Pillow   (PNG generation)
struct       (binary unpacking, built-in)
```

Install:
```bash
pip install numpy pandas pillow
```

### Key Points

✅ **sensordata/all_sensors.csv** - Ready to use, all values converted  
✅ **accel_mic/accel_mic_stream.csv** - Ready to use, already in mG  
✅ **colored_image/*.png** - RGB images, ready to view  
⚠️ **irimage/*.csv** - **RAW uint16** in CSV, **MUST CONVERT** using formula: `(value / 100.0) - 40 = °C`

The IR CSV files contain raw data to save space. Users must convert them when analyzing. This is clearly indicated in the filename prefix and table headers.


