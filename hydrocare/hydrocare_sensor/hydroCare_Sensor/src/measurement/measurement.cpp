#include "measurement/measurement.h"
#include "esp_camera.h"
#include <Wire.h>
#include "MLX90641.h"
#include "esp_task_wdt.h"

// Prevent 'sensor_t' redefinition conflict between esp_camera.h and Adafruit_Sensor.h
#define sensor_t adafruit_sensor_t
#include <Adafruit_BME680.h>
#undef sensor_t

#define CROP_SIZE 64
#define PIXEL_BYTES (CROP_SIZE * CROP_SIZE * 2) // 8192 bytes
#define R_LOAD 10000.0
#define VREF 3.3
#define NUM_PIXELS 192   // MLX90641: 16×12 thermal pixels
#undef EEPROM_WORDS  // Remove library's definition
#define EEPROM_WORDS 432 // MLX90641 EEPROM size (0x2400-0x272F)

SPIClass spi = SPIClass(HSPI); // Use FSPI on ESP32-S3
// ===== LIS3DH Registers =====
#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define CTRL_REG4 0x23
#define OUT_X_L 0x28

// ===== SPI Settings =====
SPISettings lisSettings(1000000, MSBFIRST, SPI_MODE3);
// LIS3DH uses SPI Mode 3 (CPOL=1, CPHA=1)

uint16_t ambLight = 0;
uint16_t microphone = 0;  // Microphone ADC reading
float ax = 0, ay = 0, az = 0;

// ===== BME688 Variables =====
Adafruit_BME680 bme(AQ_CS, &spi); // Hardware SPI using AQ_CS
float bme_temp = 0;
float bme_hum = 0;
float bme_pres = 0;
float bme_gas = 0;

// MLX90641 refresh rates (Control register 0x800D bits 10:7):
// -----------------------------------------------------------
// Bit    Freq      Sec/frame          POR Delay (ms)  Sample Every (ms)
// 0x00 = 0.5 Hz    2 sec              4080 ms         2400 ms
// 0x01 = 1 Hz      1 sec/frame        2080 ms         1200 ms
// 0x02 = 2 Hz      0.5 sec/frame      1080 ms         600 ms (default)
// 0x03 = 4 Hz      0.25 sec/frame     580 ms          300 ms
// 0x04 = 8 Hz      0.125 sec/frame    330 ms          150 ms
// 0x05 = 16 Hz     0.0625 sec/frame   205 ms           75 ms
// 0x06 = 32 Hz     0.03125 sec/frame  143 ms           38 ms
// 0x07 = 64 Hz     0.015625 sec/frame 112 ms           19 ms

// #define HEATMAP							// Uncomment for simple ASCII heatmap output to Serial Monitor instead of temperatures
// #define DEBUG                             // Show calculated and example values for calibration constants
#define OFFSET 0.0                         // Post-hoc cheap temperature adjustment (shift)
#define I2C_SPEED 100000                   // Set I2C clock speed (safe speed is 100 kHz, up to 400 kHz possible)
#define REFRESH_RATE 0x03                  // 0x00 (0.5 Hz) to 0x07 (64 Hz). Default: 0x03 (4 Hz)
#define SAMPLE_DELAY 300                   // delay between reading samples (see refresh rate table)
#define POR_DELAY SAMPLE_DELAY * 2.0 * 1.2 // delay required after power on reset (see refresh rate table)
#define CAL_INT -45.4209807273067          // Intercept of T_meas vs. T_o calibration curve (post-hoc calibration). My value: -45.4209807273067
#define CAL_SLOPE 2.64896693658985         // Slope of T_meas vs. T_o calibration curve (post-hoc calibration). My value: 2.64896693658985

MLX90641 myIRcam; // declare an instance of class MLX90641

void writeRegister(uint8_t reg, uint8_t value)
{
  spi.beginTransaction(lisSettings);
  digitalWrite(Acc_CS, LOW);

  spi.transfer(reg & 0x3F); // Write command (bit7=0)
  spi.transfer(value);

  digitalWrite(Acc_CS, HIGH);
  spi.endTransaction();
}
uint8_t readRegister(uint8_t reg)
{
  spi.beginTransaction(lisSettings);
  digitalWrite(Acc_CS, LOW);

  spi.transfer(0x80 | reg); // Read command (bit7=1)
  uint8_t value = spi.transfer(0x00);

  digitalWrite(Acc_CS, HIGH);
  spi.endTransaction();

  return value;
}
void readMultiple(uint8_t startReg, uint8_t *buffer, uint8_t len)
{
  spi.beginTransaction(lisSettings);
  digitalWrite(Acc_CS, LOW);
  delayMicroseconds(50);

  spi.transfer(0xC0 | startReg); // Read + Auto increment (bit7=1, bit6=1)

  for (int i = 0; i < len; i++)
    buffer[i] = spi.transfer(0x00);

  delayMicroseconds(50);
  digitalWrite(Acc_CS, HIGH);
  spi.endTransaction();
}
void readAcceleration()
{

  uint8_t rawData[6];
  readMultiple(OUT_X_L, rawData, 6);

  int16_t x = (int16_t)(rawData[1] << 8 | rawData[0]);
  int16_t y = (int16_t)(rawData[3] << 8 | rawData[2]);
  int16_t z = (int16_t)(rawData[5] << 8 | rawData[4]);

  // High-resolution mode: 12-bit left-aligned
  x >>= 4;
  y >>= 4;
  z >>= 4;

  // ±2g sensitivity = 1 mg/LSB
  ax = x * 0.001f;
  ay = y * 0.001f;
  az = z * 0.001f;
}
void measureAmbLight()
{

  uint16_t raw = analogRead(AmbLight);

  float voltage = (raw / 4095.0) * VREF;

  float current = voltage / R_LOAD; // in Amps
  float current_uA = current * 1e6; // convert to microamps

  ambLight = (uint16_t)current_uA; // calibration factor (adjust!)
  
  // Debug output to verify sensor is working
  Serial.printf("[AmbLight] RAW=%d V=%.3fV I=%.2fuA LUX=%u\n", raw, voltage, current_uA, ambLight);
}

void measureMicrophone()
{
  // Read microphone ADC value (fast, <100µs)
  uint16_t raw = analogRead(IA_Out);
  float voltage = (raw / 4095.0) * VREF;
  microphone = (uint16_t)(voltage * 1000);  // Store in millivolts or raw ADC (adjust scale as needed)
}

void measureCamera()
{

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb)
  {
    Serial.println("CAPTURE_ERR");
    delay(2000);
    return;
  }

  int startX = (fb->width - CROP_SIZE) / 2;
  int startY = (fb->height - CROP_SIZE) / 2;

  // Header
  Serial.write(0xFF);
  Serial.write(0xAA);

  for (int row = 0; row < CROP_SIZE; row++)
  {
    for (int col = 0; col < CROP_SIZE; col++)
    {
      // Offset for RGB565 (2 bytes per pixel)
      int src = ((startY + row) * fb->width + (startX + col)) * 2;

      // Combine two bytes into one 16-bit pixel
      uint16_t pixel = (fb->buf[src] << 8) | fb->buf[src + 1];

      // Convert RGB565 to RGB888
      uint8_t r = (pixel >> 11) & 0x1F;
      uint8_t g = (pixel >> 5) & 0x3F;
      uint8_t b = (pixel & 0x1F);

      // Scale to 8-bit (0-255)
      r = (r * 255) / 31;
      g = (g * 255) / 63;
      b = (b * 255) / 31;

      // Send 3 bytes to Serial (HTML expects RGB888)
      Serial.write(r);
      Serial.write(g);
      Serial.write(b);
    }
  }

  // Footer
  Serial.write(0xBB);
  Serial.write(0xFF);

  Serial.flush();
  esp_camera_fb_return(fb);
}
void measureIRTemp()
{
  myIRcam.clearNewDataBit();
  myIRcam.readTempC(); // read the temperature

  // Data is now collected silently in myIRcam.T_o[]
  // No need to print - master will receive data via SPI
  // Just do the minimal calculation for average
  
  float avg = 0.0;
  for (int i = 0; i < NUM_PIXELS; i++)
  {
    avg += myIRcam.T_o[i];
  }
  avg /= (float)NUM_PIXELS;
  
  // Only print summary (quick, non-blocking)
  //Serial.printf("[Measurement] IR Thermal: Avg=%.1f°C, Ta=%.1f°C\n", avg, myIRcam.Ta);
}

void measureBME688()
{
  if (!bme.performReading()) {
    Serial.println("[BME688] Failed to perform reading!");
    return;
  }
  bme_temp = bme.temperature;
  bme_hum = bme.humidity;
  bme_pres = bme.pressure / 100.0; // convert to hPa
  bme_gas = bme.gas_resistance / 1000.0; // convert to KOhms
  //Serial.printf("[BME688] Temp: %.2f °C, Hum: %.2f %%, Pres: %.2f hPa, Gas: %.2f KOhms\n", bme_temp, bme_hum, bme_pres, bme_gas);
}

void initIRTemp()
{

  Wire.begin(SDA, SCL);     // SDA, SCL for the ESP32 (SDA: GPIO 21, SDL: GPIO22). Change these to your I2C pins if using a different bus.
  Wire.setClock(I2C_SPEED); // set I2C clock speed (slower=more stable)
  if (myIRcam.setRefreshRate(REFRESH_RATE))
  { // set the page refresh rate (sampling frequency)
    Serial.println("Refresh rate adjusted.");
  }
  else
  {
    Serial.println("Error on adjusting refresh rate.");
  }
  delay(POR_DELAY); // Power on reset delay (POR), see table above
  Serial.println("MLX90641 ESP32 Calibrated Read");
  // Read full EEPROM (0x2400..0x272F)
  if (!myIRcam.readEEPROMBlock(0x2400, EEPROM_WORDS, myIRcam.eeData))
  {
    Serial.println("EEPROM read failed!");
    while (1)
      delay(1000);
  }

  // Mark bad pixels separately here (row indexes 0...11, col indexes 0..15)
  // myIRcam.badPixels[pixelAddr(9,14)]=true;    // mark pixel bad at row 9, column 14
  // myIRcam.badPixels[pixelAddr(11,0)]=true;    // mark pixel bad at row 11, column 0

  // Check EEPROM data:
#ifdef DEBUG
  Serial.println("setup() First 16 words of EEPROM:");
  for (int i = 0; i < 16; i++)
  {
    Serial.println("EEPROM value at address: 0x" + String(0x2400 + i, HEX) + ", value: 0x" + String(eeData[i], HEX));
  }
  Serial.println("setup() Suspicious EEPROM value check:");
  for (int i = 0; i < EEPROM_WORDS; i++)
  {
    if (myIRcam.eeData[i] == 0x0000 || myIRcam.eeData[i] == 0xFFFF)
    {
      Serial.println("EEPROM value suspicious at address: 0x" + String(0x2400 + i, HEX) + ", value: 0x" + String(eeData[i], HEX));
    }
  }
#endif
  myIRcam.Vdd = myIRcam.readVdd(); // This should be close to 3.3V. Can read once in setup.
  myIRcam.Ta = myIRcam.readTa();   // should happen inside the loop
  Serial.print("Ambient temperature on start: ");
  Serial.println(myIRcam.Ta, 1);     // This should be close to ambient temperature (21°C?)
  myIRcam.readPixelOffset();         // only needs to be read once
  myIRcam.readAlpha();               // read sensitivities (fills alpha_pixel[])
  myIRcam.readKta();                 // read Kta coefficients (fills Kta[])
  myIRcam.readKv();                  // read Kv coefficients (fills Kv[])
  myIRcam.KsTa = myIRcam.readKsTa(); // read KsTa coefficient
  Serial.println("Finished: read KsTA.");
  myIRcam.readCT();                              // read 8 corner temperatures
  myIRcam.readKsTo();                            // read 8 KsTo coefficients
  myIRcam.readAlphaCorrRange();                  // read sensitivity correction coefficients
  myIRcam.Emissivity = myIRcam.readEmissivity(); // read Emissivity coefficient
  // myIRcam.Emissivity = 0.95;                    // un-comment to over-write Emissivity with hard-coded value here (e.g. 0.95)
  myIRcam.alpha_CP = myIRcam.readAlpha_CP();    // read Sensitivity alpha_CP coefficient
  myIRcam.pix_OS_ref_CP = myIRcam.readOff_CP(); // read offset CP (also called pix_OS_ref_CP)
  myIRcam.Kv_CP = myIRcam.readKv_CP();          // read Kv CP
  myIRcam.KTa_CP = myIRcam.readKTa_CP();        // read KTa_CP
  myIRcam.TGC = myIRcam.readTGC();              // read TGC - do this last (leaves setup function for some odd reason)
  /*#ifdef DEBUG                  // uncomment this if you need a pixel map (or consult the datasheet)
  Serial.println("Printing pixel address memory map: ");
  for (int i = 0; i < 192; i++) {
    Serial.print(i);
    Serial.print(", 0x0");
    Serial.print(myIRcam.pix_addr_S0(i), HEX);
    Serial.print(", 0x0");
    Serial.println(myIRcam.pix_addr_S1(i), HEX);
    delay(100);
  }
  #endif*/
}
void initIMU()
{

  spi.begin(SCK_Perip, MISO_Perip, MOSI_Perip, Acc_CS);

  delay(100);

  uint8_t who = readRegister(WHO_AM_I);
  Serial.print("WHO_AM_I: 0x");
  Serial.println(who, HEX); // Should print 0x33

  // 100 Hz, all axes enabled
  writeRegister(CTRL_REG1, 0x57);

  // ±2g, High resolution
  writeRegister(CTRL_REG4, 0x08);

    // 100 Hz, all axes enabled
  who = readRegister(CTRL_REG1);
  Serial.print("WHO_AM_I: 0x");
  Serial.println(who, HEX); // Should print 0x33
  who = readRegister(CTRL_REG4);
  Serial.print("WHO_AM_I: 0x");
  Serial.println(who, HEX); // Should print 0x33
  
  // Give sensor time to stabilize after configuration
  delay(150);  // LIS3DH needs time to start outputting valid data
  
  // Test initial read to verify sensor is outputting data
  readAcceleration();
  Serial.printf("Initial accel - X:%.3f Y:%.3f Z:%.3f\n", ax, ay, az);
  spi.endTransaction();

  // Keep SPI open for continuous sensor operation
}

void initBME688()
{
  if (!bme.begin()) {
    Serial.println("[BME688] Could not find a valid BME688 sensor, check wiring!");
  } else {
    // Set up oversampling and filter initialization
    // Lower oversampling = faster readings. 1X is minimum (single sample, no averaging)
    bme.setTemperatureOversampling(BME680_OS_1X);  // was 8X - 8x faster now
    bme.setHumidityOversampling(BME680_OS_1X);     // was 2X - 2x faster now
    bme.setPressureOversampling(BME680_OS_1X);     // was 4X - 4x faster now
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 50); // 320°C for 50ms (was 150ms - use lower time for speed)
    Serial.println("[BME688] Initialized successfully");
  }
}

void initCamera()
{

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CSI_D0;
  config.pin_d1 = CSI_D1;
  config.pin_d2 = CSI_D2;
  config.pin_d3 = CSI_D3;
  config.pin_d4 = CSI_D4;
  config.pin_d5 = CSI_D5;
  config.pin_d6 = CSI_D6;
  config.pin_d7 = CSI_D7;
  config.pin_xclk = CSI_MCLK;
  config.pin_pclk = CSI_PCLK;
  config.pin_vsync = CSI_VSYNC;
  config.pin_href = CSI_HSYNC;
  config.pin_sccb_sda = TWI_SDA;
  config.pin_sccb_scl = TWI_SCK;
  config.pin_pwdn = -1;
  config.pin_reset = -1;
  config.xclk_freq_hz = 16000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_QVGA; // 320×240, we crop center 64×64
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_DRAM;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed: 0x%x\n", err);
    // while (true) delay(1000);
  }
  else
    Serial.println("CAM_READY");
}