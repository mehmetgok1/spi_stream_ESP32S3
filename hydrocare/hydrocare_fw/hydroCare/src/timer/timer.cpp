#include "timer/timer.h"
#include "ble/ble.h"
#include "ui/ui.h"
#include "measurement/measurement.h"

int timerValue = 1000000;  // 1 second (1,000,000 microseconds at 1 MHz tick rate)
bool timerStream = 0;

hw_timer_t* timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);

  timerStream = 1;

  portEXIT_CRITICAL_ISR(&timerMux);
}
void initTimer()
{
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, timerValue, true);
  timerAlarmDisable(timer);
}

void setTimer()
{
  timerAlarmWrite(timer, timerValue, true);
  timerAlarmEnable(timer);
}

void disableTimer()
{
  timerAlarmDisable(timer);
}