#ifndef _SUPERCAR_H__
#define _SUPERCAR_H__

typedef struct carControl
{
    float MPUangleX = 0.0f;
    float MPUgyroX = 0.0f;
    float MA_Velocity = 0.0f;
    float MB_Velocity = 0.0f;
    float TargetVelocity = 0.0f;
    float SteerVelocity = 0.0f;
    float VelocityError = 0.0f;
    float TargetAngle = 0.0f;
    float MotorVelocity = 0.0f;
} CarControl_t;

class SuperCar
{
  public:
    void startTask();
    void running();
    void requestArmToggle();
    void requestDisarm();
    bool isArmed() const;

  private:
    void armIfSafe();
    void disarm(const char *reason);
    void printTelemetry(float relativeAngle);

    volatile bool armToggleRequested = false;
    volatile bool disarmRequested = false;
    volatile bool armed = false;
    bool ready = false;
    float uprightZero = 0.0f;
    unsigned long lastTelemetryMs = 0;
};

extern SuperCar BalanceCar;

#endif
