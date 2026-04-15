#include "measurement/measurement.h"
#include "ui/ui.h"

HardwareSerial mmwave(2);

#define RADAR_BAUD    256000
#define RADAR_SERIAL  Serial1

static const uint8_t DATA_FRAME_HEADER[4] = {0xF4, 0xF3, 0xF2, 0xF1};
static const uint8_t DATA_FRAME_TAIL[4]   = {0xF8, 0xF7, 0xF6, 0xF5};

static uint8_t  rxBuf[256];
static uint16_t rxLen = 0;

uint16_t movingDist = 0, staticDist = 0, detectionDist = 0;
uint8_t  movingEnergy = 0,  staticEnergy = 0;

#define R_LOAD  10000.0
#define VREF    3.3

float batteryLevel = 0.0, batteryPercentage = 0.0;
float PIRValue = 0.0;
float ambLight=0;
uint16_t ambLight_Int=0;
bool chargingStatus = false, buttonStatus = false;
uint16_t startTime = 0;


void parseAndPrintFrame(const uint8_t* frame, uint16_t len) {
  if (len < 18)          return;
  if (frame[7] != 0xAA)  return;
  if (frame[17] != 0x55) return;

  uint8_t  dataType   = frame[6];
  uint8_t  state      = frame[8];
  uint8_t  raw9       = frame[9];
  uint8_t  raw10      = frame[10];
  uint16_t combined   = frame[9] | (frame[10] << 8);
  uint16_t asGate75   = raw9 * 75;   // if frame[9] is gate index × 0.75m
  uint16_t asGate20   = raw9 * 20;   // if frame[9] is gate index × 0.2m

  // Also print raw moving/static/detection bytes
  movingDist    = frame[9]  | (frame[10] << 8);
  movingEnergy  = frame[11];
  staticDist    = frame[12] | (frame[13] << 8);
  staticEnergy  = frame[14];
  detectionDist = frame[15] | (frame[16] << 8);

  /*
  Serial.print(movingDist);
  Serial.print(',');
  Serial.print(movingEnergy);
  Serial.print(',');
  Serial.print(staticDist);
  Serial.print(',');
  Serial.print(staticEnergy);
  Serial.print(',');
  Serial.println(detectionDist);
  */
}
void readFrame() {
  while (RADAR_SERIAL.available()) {
    uint8_t b = RADAR_SERIAL.read();
    rxBuf[rxLen++] = b;
    if (rxLen > 250) rxLen = 0;

    if (rxLen >= 4) {
      uint16_t s = rxLen - 4;
      if (rxBuf[s]   == DATA_FRAME_HEADER[0] && rxBuf[s+1] == DATA_FRAME_HEADER[1] &&
          rxBuf[s+2] == DATA_FRAME_HEADER[2] && rxBuf[s+3] == DATA_FRAME_HEADER[3]) {
        memmove(rxBuf, rxBuf + s, 4);
        rxLen = 4;
      }
    }

    if (rxLen >= 6 && rxBuf[0] == DATA_FRAME_HEADER[0]) {
      uint16_t dataLen  = rxBuf[4] | (rxBuf[5] << 8);
      uint16_t totalLen = 4 + 2 + dataLen + 4;
      if (rxLen >= totalLen) {
        if (rxBuf[totalLen-4] == DATA_FRAME_TAIL[0] && rxBuf[totalLen-3] == DATA_FRAME_TAIL[1] &&
            rxBuf[totalLen-2] == DATA_FRAME_TAIL[2] && rxBuf[totalLen-1] == DATA_FRAME_TAIL[3]) {
          parseAndPrintFrame(rxBuf, totalLen);
          rxLen = 0;
        } else {
          memmove(rxBuf, rxBuf + 1, --rxLen);
        }
      }
    }
  }
}
void measuremmWave(){

  readFrame();
}
void initmmWave(){

  digitalWrite(Sensor_EN, LOW);
  delay(100);
  RADAR_SERIAL.begin(RADAR_BAUD, SERIAL_8N1, mmWave_RX, mmWave_TX);
  delay(100);

}
void measurePIR(){
  
  PIRValue = analogRead(PIR);
}
void measureBatteryLevel(){

  digitalWrite(Batt_EN, LOW);
  digitalWrite(CE_En, HIGH);

  batteryLevel = 0;
  for(int i=0;i<Batt_Meas_Count;i++){
    
    int measBattery = analogRead(Batt_LVL);
    batteryLevel = batteryLevel + measBattery;
  }
  digitalWrite(Batt_EN, HIGH);
  digitalWrite(CE_En, LOW);

  batteryLevel = batteryLevel / Batt_Meas_Count;
  batteryLevel = (batteryLevel * Batt_VoltDiv_Mult*DigitalSupply) / 4095;
  batteryLevel = batteryLevel * Batt_Const_X - Batt_Const_Y;

  //Serial.print("Battery Level: ");
  //Serial.println(batteryLevel);

  // High clamp
  if (batteryLevel >= 4.20f)
      batteryPercentage = 100.0f;
  // 4.20 → 3.95  (100 → 75)
  else if (batteryLevel < 4.20f && batteryLevel >= 3.95f)
      batteryPercentage = 75.0f + (batteryLevel - 3.95f) * (25.0f / 0.25f);
  // 3.95 → 3.80 (75 → 45)
  else if (batteryLevel < 3.95f && batteryLevel >= 3.80f)
      batteryPercentage = 45.0f + (batteryLevel - 3.80f) * (30.0f / 0.15f);
  // 3.80 → 3.70 (45 → 28)
  else if (batteryLevel < 3.80f && batteryLevel >= 3.70f)
      batteryPercentage = 28.0f + (batteryLevel - 3.70f) * (17.0f / 0.10f);
  // 3.70 → 3.50 (28 → 6)
  else if (batteryLevel < 3.70f && batteryLevel >= 3.50f)
      batteryPercentage = 6.0f + (batteryLevel - 3.50f) * (22.0f / 0.20f);
  // 3.50 → 3.20 (6 → 1)
  else if (batteryLevel < 3.50f && batteryLevel >= 3.20f)
      batteryPercentage = 1.0f + (batteryLevel - 3.20f) * (5.0f / 0.30f);
  // Low clamp
  else if (batteryLevel < 3.20f)
      batteryPercentage = 0.0f;

}
void measureAmbLight(){
  
  uint16_t raw = analogRead(AmbLight);

  float voltage = (raw / 4095.0) * VREF;

  float current = voltage / R_LOAD; // in Amps
  float current_uA = current * 1e6; // convert to microamps

  ambLight = current_uA * 6.67;      // calibration factor (adjust!)
}
void checkUSB(){

  while(digitalRead(USB_Voltage) == 1)
  {
    chargingStatus = true;
    measureBatteryLevel();
    uiChargingScenario();
  }
  if(digitalRead(USB_Voltage) == 0){
    chargingStatus = false;
  }
}
void checkButton(){
  if(digitalRead(Button) == 1){
    delay(25);
    if(digitalRead(Button) == 1)
      buttonStatus = true;
    else
      buttonStatus = false;
  }
  else
    buttonStatus = false;
}