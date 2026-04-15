#include <Arduino.h>
#include "config/config.h"

const int pwmChannel = 0;
const int pwmFreq = 5000;      // 5 kHz (good for LED, no visible flicker)
const int pwmResolution = 10;  // 10-bit resolution (0-1023)

void powerLED(uint16_t brightness){

  if(brightness == 100)
    digitalWrite(ledCntrl, LOW);
  else{
    brightness = constrain(brightness, 0 , 100);      // Converts 0-100 % to 1023-0 pwm value
    int pwmValue = map(brightness, 0, 100, 1023, 0);  // because of PMOS, it works inversely
    ledcWrite(pwmChannel, pwmValue);
  }
}

void IRLED(bool status){

    if(status == 0)
        digitalWrite(ledCntrlIR, HIGH);
    else if(status == 1)
        digitalWrite(ledCntrlIR, LOW);
}

void powerLEDInit(){

  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(ledCntrl, pwmChannel);

  // Start with LED off
  ledcWrite(pwmChannel, 1023);
}