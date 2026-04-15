#ifndef TIMER_HANDLER_H
#define TIMER_HANDLER_H

#include <Arduino.h>

extern int timerValue;
extern bool timerStream;

void initTimer();
void setTimer();
void disableTimer();

#endif // TIMER_HANDLER_H