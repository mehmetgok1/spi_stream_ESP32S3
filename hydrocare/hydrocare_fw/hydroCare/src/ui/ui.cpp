#include <Arduino.h>
#include "ui/ui.h"
#include <Adafruit_NeoPixel.h>

#define Neopixel    38
#define NUMPIXELS   1

Adafruit_NeoPixel pixels(NUMPIXELS, Neopixel, NEO_GRB + NEO_KHZ800);

uint8_t red=0, green=0, blue=0;

void initLed()
{
    pixels.begin();
    delay(10);
}
void setLED(){

    pixels.setPixelColor(0, pixels.Color(red, green, blue));
    pixels.show();
}
void uiInit(){

    red=0; green=5; blue=0;
    setLED();
}
void uiChargingScenario(){

    for(int j=0; j<5; j++){
        for(int i=0; i<=100; i++){
            red = i;
            setLED();
            delay(10);
            if(digitalRead(USB_Voltage) == 0){
                red = 0;
                setLED();
                break;
            }
        }
        for(int i=100; i>=0; i--){
            red = i;
            setLED();
            delay(10);
            if(digitalRead(USB_Voltage) == 0){
                red = 0;
                setLED();
                break;
            }
        }
    }
}
void uiOTAStarted(){
    red = 5; green = 5; blue = 5; setLED();
}