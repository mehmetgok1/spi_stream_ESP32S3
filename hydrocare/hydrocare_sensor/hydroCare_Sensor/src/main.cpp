#include <Arduino.h>
#include "config/config.h"
#include "measurement/measurement.h"
#include "leds/leds.h"
#include "communication/communication.h"

void setup() {
  initPins();
  initPeripherals();
  initCamera();
  powerLEDInit();
  initIMU();
  initIRTemp();
  initBME688(); // Ensure this is after initIMU so the SPI bus is ready
  initSPIComm();
  
  // Start background tasks
  startHighSpeedSamplerTask();  // 2kHz accel + mic sampling on Core 1 (Priority 1) - 2000 samples/second
  startIRSensorTask();          // IR thermal sensor every 200ms on Core 1 (Priority 1)
  startBMESensorTask();         // BME688 sensor every 200ms on Core 1 (Priority 1)
  startMeasurementTask();       // Measurement collector on Core 0 (Priority 2)
  
  Serial.println("✓ Setup Complete - Ready for SPI with cached sensors (IR+BME@200ms, Accel@2kHz/2k samples/sec)");
}

void loop() {
  receiveCommand();
}
