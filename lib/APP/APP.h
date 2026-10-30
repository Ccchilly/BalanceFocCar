#ifndef APP_H
#define APP_H

#include <Arduino.h>

typedef struct appControl
{
    volatile float Velocity = 0.0f;
    volatile float SteerVelocity = 0.0f;
    volatile bool Connected = false;
    volatile unsigned long LastReportMs = 0;
} AppControl_t;

class AppTaskInit
{
  public:
    void startTask();
};

extern AppTaskInit APP;
extern AppControl_t appCTRL;

#endif
